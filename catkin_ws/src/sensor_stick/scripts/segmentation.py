#!/usr/bin/env python
import numpy as np
import math
import os
import sys
import pickle
#import collections
#import types
#import traceback
#import pprint
#pp = pprint.PrettyPrinter(indent=4)
from sklearn.preprocessing import LabelEncoder


# ROS imports
import rospy
#import tf
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from sensor_stick.srv import GetNormals

#from sensor_stick.marker_tools import *
import sensor_stick.marker_tools as marker_tools
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from visualization_msgs.msg import Marker

import pcl
# Local imports
import pcl_helper

#import sensor_stick.pcl_helper
import pclproc

#====================== GLOBALS =====================
# Clearly this module wants to be a class
g_pcl_sub = None

g_pcl_debug_pub = None
g_pcl_objects_pub = None
g_pcl_table_pub = None
g_pcl_cluster_pub = None
g_object_markers_pub = None
g_detected_objects_pub = None

g_model = None
g_clf = None
g_encoder = None
g_scaler = None

g_callBackCount = -1
g_callBackSkip =  40# How many callbacks to skip until actual processing. Default is 0

# For debug testing only
g_doRunRosNode = True # For invoking RunRosNode() when run from pycharm
g_doTests = False # Invokes Test_Process_msgPCL() when file is run
g_testmsgPCLFilename = "./Assets/msgPCL" # + "num..pypickle" # File containing a typical Ros msgPCL, used by doTests
g_testrawPCLFilename = "./Assets/rawPCL" # + "num.pypickle" # File containing a typical rawPCL as unpacked my pcl_helper used by doTests
g_dumpCountTestmsgPCL = 0 # How many debug msgPCL files to dump. Normally 0
g_dumpCountTestrawPCL = 0 # How many debug rawPCL files to dump. Normally 0

#--------------------------------- get_normals()
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# --------------------------------- Debug_Publish()
def Debug_Publish(pclIn):
    pclpcIn = pcl_helper.XYZ_to_XYZRGB(pclIn, (0,255,0))
    msgPCL = pcl_helper.pcl_to_ros(pclpcIn)
    g_pcl_debug_pub.publish(msgPCL)

#--------------------------------- ProcessPCL()
def Process_rawPCL(pclpcRawIn):
    DebugDumpMsgPCL(pclpcRawIn)
    pclRecs = [] # For dev/debug display. Container for point cloud records: tuple (pclObj, pclName# )
    pclRecs.append((pclpcRawIn, "pclpcRawIn"))


    pclRecsDownSampled = pclproc.PCLProc_DownSampleVoxels(pclpcRawIn)
    pclRecs += pclRecsDownSampled
    pclpcDownSampled, pclpcDownSampledName = pclRecsDownSampled[0]

    # Region of Interest: PassThrough Filter
    recsPassThruZ = pclproc.PCLProc_PassThrough(pclpcDownSampled, 'z', 0.6, 1.1)
    pclRecs += recsPassThruZ
    pclpcPassZ = recsPassThruZ[0][0]

    recsPassThruY = pclproc.PCLProc_PassThrough(pclpcPassZ, 'y', -0.5, 0.5)
    pclRecs += recsPassThruY
    pclpcPassY = recsPassThruY[0][0]

    # RANSAC Table/Object separation
    pclRecsRansac = pclproc.PCLProc_Ransac(pclpcPassY)
    pclRecs += pclRecsRansac
    pclpcPassZIn, pclpcPassZOut = pclRecsRansac[0][0], pclRecsRansac[1][0]
    pclpcTable, pclpcObjects = pclpcPassZIn, pclpcPassZOut # Rename for clarity

    # Noise reduction
    pclpRecsNoNoise  = pclproc.PCLProc_Noise(pclpcObjects)
    pclRecs += pclpRecsNoNoise
    pclpNoColorNoNoise = pclpRecsNoNoise[0][0]
    Debug_Publish(pclpNoColorNoNoise)


    # Euclidean Clustering
    pclpObjectsNoColor = pcl_helper.XYZRGB_to_XYZ(pclpcObjects)
    #clusterIndices, pclpcClusters = pclproc.PCLProc_ExtractClusters(pclpObjectsNoColor)
    clusterIndices, pclpClusters = pclproc.PCLProc_ExtractClusters(pclpNoColorNoNoise)

    labelRecs = []

    for index, pts_list in enumerate(clusterIndices):
        # Get points for a single object in the overall cluster
        pcl_cluster = pclpcObjects.extract(pts_list)
        msgPCL_cluster = pcl_helper.pcl_to_ros(pcl_cluster) # Needed for histograms... would refactor

        # Extract histogram features
        chists = pclproc.compute_color_histograms(msgPCL_cluster, doConvertToHSV=True)
        normals = get_normals(msgPCL_cluster)
        nhists = pclproc.compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # CLASSIFY, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = g_clf.predict(g_scaler.transform(feature.reshape(1, -1)))
        label = g_encoder.inverse_transform(prediction)[0]

        # Accumulate label records for publishing (and labeling detected objects)
        label_pos = list(pclpcObjects[pts_list[0]])
        #label_pos[2] += 0.3
        labelRecs.append((label, label_pos, index))

    return labelRecs, pclpcObjects, pclpcTable, pclpClusters

#--------------------------------- CB_msgPCL()
def CB_msgPCL(msgPCL):
    """
    ROS "/sensor_stick/point_cloud" subscription Callback handler
    Handle the PointCloud ROS msg received by the "/sensor_stick/point_cloud"
    This function is almost entirely unpacking/packing ROS messages and publishing.
    The the unpacked input pcl is processed by Process_rawPCL(pclpcRawIn)
    which returns the values that need to be packed and published
    :param msgPCL:
    :return:
    """
    global g_callBackCount
    g_callBackCount += 1

    if (g_callBackCount % g_callBackSkip != 0):
        return;

    print "\rCBCount= {:05d}".format(g_callBackCount),
    sys.stdout.flush()

    DebugDumpMsgPCL(msgPCL)
    #g_pcl_debug_pub.publish(msgPCL)

    # Extract pcl Raw from Ros msgPCL
    pclpcRawIn = pcl_helper.ros_to_pcl(msgPCL)

    #------- PROCESS RAW PCL-------------------------
    labelRecs, pclpcObjects, pclpcTable, pclpcClusters = Process_rawPCL(pclpcRawIn)

    detected_objects_labels = [] # For ros loginfo only
    detected_objects = [] # For publish - for PROJ3!

    for (labelText, labelPos, labelIndex) in labelRecs:
        detected_objects_labels.append(labelText)
        g_object_markers_pub.publish(marker_tools.make_label(labelText, labelPos, labelIndex ))
        # Add  detected object to the list of detected objects.
        do = DetectedObject()
        do.label = labelText
        do.cloud = pclpcClusters
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Package Processed pcls into Ros msgPCL
    msgPCLObjects = pcl_helper.pcl_to_ros(pclpcObjects)
    msgPCLTable = pcl_helper.pcl_to_ros(pclpcTable)
    msgPCLClusters = pcl_helper.pcl_to_ros(pclpcClusters)

    # Publish everything
    # This is the output you'll need to complete the upcoming project!
    g_detected_objects_pub.publish(detected_objects) # THIS IS THE CRUCIAL STEP FOR PROJ3
    g_pcl_objects_pub.publish(msgPCLObjects)
    g_pcl_table_pub.publish(msgPCLTable)
    g_pcl_cluster_pub.publish(msgPCLClusters)


#====================== Main() =====================
def RunRosNode():
    '''
    ROS  clustering/segmentation node initialization
    '''
    print("ROS clustering/segmentation node initializatiing...")

    global g_pcl_sub

    global g_pcl_debug_pub
    global g_pcl_objects_pub
    global g_pcl_table_pub
    global g_pcl_cluster_pub
    global g_object_markers_pub
    global g_detected_objects_pub

    global g_model
    global g_clf
    global g_encoder
    global g_scaler

    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    #pclSrcNodeName = "/sensor_stick/point_cloud" # For Proj3 Exercise3 SensorStick
    pclSrcNodeName = "/pr2/world/points" # For Proj3 NOISE
    #pclSrcNodeName = "/camera/depth_registered/points" # For Proj3 NO NOISE
    #g_pcl_sub = rospy.Subscriber(pclSrcNodeName, pc2.PointCloud2, CB_msgPCL, queue_size=1)
    g_pcl_sub = rospy.Subscriber(pclSrcNodeName, pcl_helper.PointCloud2, CB_msgPCL, queue_size=1)

    # Create Publishers
    g_pcl_debug_pub = rospy.Publisher("/pcl_debug", pcl_helper.PointCloud2, queue_size=1)
    g_pcl_objects_pub = rospy.Publisher("/pcl_objects", pcl_helper.PointCloud2, queue_size=1)
    g_pcl_table_pub = rospy.Publisher("/pcl_table", pcl_helper.PointCloud2, queue_size=1)
    g_pcl_cluster_pub = rospy.Publisher("/pcl_cluster", pcl_helper.PointCloud2, queue_size=1)
    g_object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=8)
    g_detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    g_model = pickle.load(open('model.sav', 'rb'))
    g_clf = g_model['classifier']
    g_encoder = LabelEncoder()
    g_encoder.classes_ = g_model['classes']
    g_scaler = g_model['scaler']

    # Initialize color_list
    pcl_helper.get_color_list.color_list = []

    while not rospy.is_shutdown():
        print("ROS clustering/segmentation node running")
        rospy.spin()



###################################### TESTS ###########################
###################################### TESTS ###########################
###################################### TESTS ###########################
def DebugDumpRawPCL(pclpcRawIn):
    global g_dumpCountTestrawPCL
    # DevDebug save rawPCL to file for debug
    if (g_dumpCountTestrawPCL > 0):
        g_dumpCountTestrawPCL -= 1
        fileNameOut = g_testrawPCLFilename + str(g_dumpCountTestrawPCL)  + ".pypickle"
        pickle.dump(pclpcRawIn, open(fileNameOut, "wb"))

def DebugDumpMsgPCL(msgPCL):
    global g_dumpCountTestmsgPCL
    # DevDebug save msgPCL to file for debug
    if (g_dumpCountTestmsgPCL > 0):
        g_dumpCountTestmsgPCL -= 1
        fileNameOut = g_testmsgPCLFilename + str(g_dumpCountTestmsgPCL)  + ".pypickle"
        pickle.dump(msgPCL, open(fileNameOut, "wb"))


#--------------------------------- Test_Process_rawPCL()
def Test_Process_rawPCL():
    dumpIndex = 0
    fileNameIn = g_testrawPCLFilename + str(dumpIndex) + ".pypickle"
    pclpcRawIn = pickle.load( open(fileNameIn, "rb" ) )
    pclpcObjects, pclpcTable, pclpcClusters = Process_rawPCL(pclpcRawIn)


# ============ Auto invoke Test_PCLProc_*
if (g_doTests):
    Test_Process_rawPCL()


#====================== Main Invocation RunRosNode() =====================
if ((__name__ == '__main__') & g_doRunRosNode):
    RunRosNode()
