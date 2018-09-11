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
import geometry_msgs.msg as msgGeo
import std_msgs.msg as msgStd

import sensor_msgs.point_cloud2 as pc2
from sensor_stick.srv import GetNormals

#from sensor_stick.marker_tools import *
import sensor_stick.marker_tools as marker_tools
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from visualization_msgs.msg import Marker
from rospy_message_converter import message_converter
import yaml

import pcl
# Local imports
import pcl_helper

#import sensor_stick.pcl_helper
import pclproc

#====================== GLOBALS =====================
# Clearly this module wants to be a class
g_callBackSkip =  5# How many callbacks to skip until actual processing. Default is 0
g_callBackCount = -1

g_WORLD_NUM  = 1

g_numHistBinsHSV = 64
g_numHistBinsNormals = 100

g_assetsDir = '/home/cl/AAAProjects/AAAUdacity/roboND/Proj3_3dPerception/Proj3_Root/catkin_ws/src/sensor_stick/scripts/Assets/'
g_yamlOutDirName = g_assetsDir + 'yamlOut/'
g_yamlOutFileName = g_yamlOutDirName + 'world_{}.yaml'.format(g_WORLD_NUM)
g_classifierModelDir = '/home/cl/AAAProjects/AAAUdacity/roboND/Proj3_3dPerception/Proj3_Root/catkin_ws/src/sensor_stick/scripts/Assets/Training/P3World1/'
g_classifierModelFileNameBase =  "P3World1_caps75_colorbins64_normalbin100_fitlinear_2018-09-10T20:42:07.clfModel"
g_classifierModelFileName = g_classifierModelDir + g_classifierModelFileNameBase

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

# For debug testing only
g_doCommandRobot = False
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
    clusterIndices, pclpClusters = pclproc.PCLProc_ExtractClusters(pclpObjectsNoColor)
    #clusterIndices, pclpClusters = pclproc.PCLProc_ExtractClusters(pclpNoColorNoNoise)

    labelRecs = []

    for index, pts_list in enumerate(clusterIndices):
        # Get points for a single object in the overall cluster
        pcl_cluster = pclpcObjects.extract(pts_list)
        msgPCL_cluster = pcl_helper.pcl_to_ros(pcl_cluster) # Needed for histograms... would refactor

        # Extract histogram features
        chists = pclproc.compute_color_histograms(msgPCL_cluster, g_numHistBinsHSV, doConvertToHSV=True)
        normals = get_normals(msgPCL_cluster)
        nhists = pclproc.compute_normal_histograms(normals, g_numHistBinsNormals)
        feature = np.concatenate((chists, nhists))

        # CLASSIFY, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = g_clf.predict(g_scaler.transform(feature.reshape(1, -1)))
        label = g_encoder.inverse_transform(prediction)[0]

        # Accumulate label records for publishing (and labeling detected objects)
        label_pos = list(pclpcObjects[pts_list[0]])
        label_pos[2] += 0.2
        labelRecs.append((label, label_pos, index, pcl_cluster)) # Don't like passing pcl_cluster in this Records...

    return labelRecs, pclpcObjects, pclpcTable, pclpClusters

# Helper function to create a yaml friendly dictionary from ROS messages
#--------------------------------- pr2_mover()
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
#--------------------------------- pr2_mover()
def send_to_yaml(fileNameOut, dict_list):
    fileNameOutFQ = os.path.abspath(fileNameOut)
    print("Saving output yaml file {}".format(fileNameOutFQ))
    dirName = os.path.dirname(fileNameOutFQ)
    if not os.path.exists(dirName):
        os.makedirs(dirName)

    data_dict = {"object_list": dict_list}
    with open(fileNameOutFQ, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# function to load parameters and request PickPlace service
#--------------------------------- pr2_mover()
def pr2_mover(detected_objects):
    object_list = detected_objects

    # TODO: Initialize variables
    test_scene_num = msgStd.Int32()
    arm_name = msgStd.String()
    object_name = msgStd.String()
    pick_pose = msgGeo.Pose()
    place_pose = msgGeo.Pose()

    test_scene_num.data = g_WORLD_NUM
    detected = 0
    dict_list = []

    # TODO: Get/Read parameters
    # get parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    dictGroupColorToBoxParam = {}
    for boxParam in dropbox_param:
        groupColorStr = boxParam['group']
        dictGroupColorToBoxParam[groupColorStr] = boxParam

    # TODO: Loop through the pick list
    for objIndex in range(0, len(object_list_param)):

        # TODO: Parse parameters into individual variables
        object_group = object_list_param[objIndex]['group']

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for object in object_list:
            if object_list_param[objIndex]['name'] == object.label:
                # labels.append(object.label)
                points_arr = pcl_helper.ros_to_pcl(object.cloud).to_array()
                centroids = np.mean(points_arr, axis=0)[:3]
                pick_pose.position.x = np.asscalar(centroids[0])
                pick_pose.position.y = np.asscalar(centroids[1])
                pick_pose.position.z = np.asscalar(centroids[2])

                object_name.data = object_list_param[objIndex]['name']

                detected += 1
                break

        # TODO: Create 'place_pose' for the object
        targetBox = dictGroupColorToBoxParam[object_group]
        dropbox_pose =targetBox['position']
        place_pose.position.x = dropbox_pose[0]
        place_pose.position.y = dropbox_pose[1]
        place_pose.position.z = dropbox_pose[2]

        # TODO: Assign the arm to be used for pick_place
        arm_name.data = targetBox['name']

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        # Populate various ROS messages
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)


        # TODO: Rotate PR2 in place to capture side tables for the collision map
        if g_doCommandRobot:
            try:
                rospy.wait_for_service('pick_place_routine')
                #pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # TODO: Insert your message variables to be sent as a service request
                #resp = pick_place_routine(g_WORLD_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

                #print ("Response: ",resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml(g_yamlOutFileName, dict_list)
    print("Detected: %s/%s" % (detected, len(object_list_param)))


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

    for (labelText, labelPos, labelIndex, pcl_cluster) in labelRecs:
        detected_objects_labels.append(labelText)
        g_object_markers_pub.publish(marker_tools.make_label(labelText, labelPos, labelIndex ))
        # Add  detected object to the list of detected objects.
        do = DetectedObject()
        do.label = labelText
        do.cloud = pcl_helper.pcl_to_ros(pcl_cluster)
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

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

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
    g_model = pickle.load(open(g_classifierModelFileName, 'rb'))
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
