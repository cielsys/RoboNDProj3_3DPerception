import pickle
#print("CWD:", os.getcwd())
#absPath = os.path.abspath('../src/sensor_stick')
#print("absPath", absPath)
#sys.path.insert(0, absPath)
#sys.path.insert(0, "/home/cl/AAAProjects/AAAUdacity/roboND/Proj3_3dPerception/RoboND-Perception-Exercises/catkin_ws3/src/sensor_stick/src/sensor_stick/")
#print("sys.path",sys.path)
#from sensor_stick import pcl_helper
#import sensor_stick
import sensor_stick.pcl_helper as pcl_helper

#--------------------------------- pickle templates
def Camera_SaveCalFile(calFileName, dictCameraCalVals):
    pickle.dump(dictCameraCalVals, open(calFileName, "wb" ) )

def LoadFile_msgPCL(fileNameIn = "msgPCL.pypickle"):
    retVal = pickle.load( open(fileNameIn, "rb" ) )
    return(retVal)

    # Not really needed here because we have no noise
    # Much like the previous filters, we start by creating a filter object:
    #outlier_filter = cloud_filtered.make_statistical_outlier_filter(1)

    # Set the number of neighboring points to analyze for any given point
    #outlier_filter.set_mean_k(50)

    # Set threshold scale factor
    #x = 1.0

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    #outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    #cloud_filtered = outlier_filter.filter()

    # Create a VoxelGrid filter object for our input point cloud
    vox = pclpcIn.make_voxel_grid_filter()
    voxelSize = 0.01
    vox.set_leaf_size(voxelSize, voxelSize, voxelSize)

    # TODO: Voxel Grid Downsampling
    # Call the filter function to obtain the resultant downsampled point cloud
    pclpcVoxels = vox.filter()
    pclRecs.append((pclpcVoxels, "pclpcDownSampled"))


objName = "Ind{}_{}".format(index, label)
print(objName)
dirNameOut = "./Assets/pcdClassifierOut/"
pclRecs = [(pcl_cluster, objName)]
# pclproc.SavePCLs(pclRecs, dirNameOut, useTimeStamp=True)


#--------------------------------- Process_msgPCL()
def Process_msgPCL(msgPCL):
    global g_dumpCountTestmsgPCL
    global g_dumpCountTestrawPCL

    # DevDebug save msgPCL to file for debug
    if (g_dumpCountTestmsgPCL > 0):
        g_dumpCountTestmsgPCL -= 1
        fileNameOut = g_testmsgPCLFilename + str(g_dumpCountTestmsgPCL)  + ".pypickle"
        pickle.dump(msgPCL, open(fileNameOut, "wb"))

    # Extract pcl Raw from Ros msgPCL
    pclpcRawIn = pcl_helper.ros_to_pcl(msgPCL)

    # DevDebug save rawPCL to file for debug
    if (g_dumpCountTestrawPCL > 0):
        g_dumpCountTestrawPCL -= 1
        fileNameOut = g_testrawPCLFilename + str(g_dumpCountTestrawPCL)  + ".pypickle"
        pickle.dump(pclpcRawIn, open(fileNameOut, "wb"))

    #------- PROCESS RAW PCL-------------------------
    labelRecs, pclpcObjects, pclpcTable, pclpcClusters = Process_rawPCL(pclpcRawIn)

    # Package Processed pcls into Ros msgPCL
    msgPCLObjects = pcl_helper.pcl_to_ros(pclpcObjects)
    msgPCLTable = pcl_helper.pcl_to_ros(pclpcTable)
    msgPCLClusters = pcl_helper.pcl_to_ros(pclpcClusters)

    detected_objects_labels = []
    detected_objects = []

    for (labelText, labelPos, labelIndex) in labelRecs:
        detected_objects_labels.append(labelText)
        g_object_markers_pub.publish(marker_tools.make_label(labelText, labelPos, labelIndex ))
        # Add  detected object to the list of detected objects.
        do = DetectedObject()
        do.label = labelText
        do.cloud = pclpcClusters
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish everything
    # This is the output you'll need to complete the upcoming project!
    g_detected_objects_pub.publish(detected_objects) # THIS IS THE CRUCIAL STEP FOR PROJ3

    return msgPCLObjects, msgPCLTable, msgPCLClusters


#--------------------------------- Test_Process_msgPCL()
def Test_Process_msgPCL():
    dumpIndex = 0
    fileNameIn = g_testmsgPCLFilename + str(dumpIndex) + ".pypickle"
    msgPCL = pickle.load( open(fileNameIn, "rb" ) )
    msgPCLObjects, msgPCLTable, pclpcClusters = Process_msgPCL(msgPCL)


    white_cloud = pcl_helper.XYZRGB_to_XYZ(pclpcObjects)
