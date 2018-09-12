#!/usr/bin/env python
import numpy as np
import pickle
import sys
import os
import datetime
import rospy

# Ros package imports
from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

# Local imports
import pclproc
import pcl_helper

#====================== GLOBALS =====================
g_trainingFileDir = "./Assets/Training/"
g_numCapsDefault = 200
g_modelSetName = "P3World3"

g_numHistBinsHSV = 96
g_numHistBinsNormals = 100

g_numRetriesPerCapture = 3

#--------------------------------- SelectModelNames()
# /home/cl/AAAProjects/AAAUdacity/roboND/Proj3_3dPerception/Proj3_Root/catkin_ws/src/RoboND-Perception-Project/pr2_robot/config/
def SelectModelNames(modelSetName, numCapsDefault):

    dictModelSetNames = {
        "P3Exer3" : [('beer', numCapsDefault), ('bowl', numCapsDefault), ('create', numCapsDefault), ('disk_part', numCapsDefault), ('hammer', numCapsDefault), ('plastic_cup', numCapsDefault), ('soda_can', numCapsDefault)],
        "P3World1": [('biscuits', numCapsDefault), ('soap', numCapsDefault), ('soap2', numCapsDefault)],
        "P3World2": [('biscuits', numCapsDefault), ('soap', numCapsDefault), ('soap2', numCapsDefault),('book', numCapsDefault), ('glue', numCapsDefault)],
        "P3World3": [('biscuits', numCapsDefault), ('soap', numCapsDefault), ('soap2', numCapsDefault),('book', numCapsDefault), ('glue', numCapsDefault), ('snacks', numCapsDefault),('eraser', numCapsDefault), ('sticky_notes', numCapsDefault)]
    }
    modelNames = dictModelSetNames[modelSetName]

    return(modelNames)


#--------------------------------- SelectModelNames()
def GetTrainingSetName(modelSetName, numCapsDefault, numHistBinsHSV, numHistBinsNormals):
    strDT = "{:%Y-%m-%dT%H:%M:%S}".format(datetime.datetime.now())

    trainingDirNameOut = g_trainingFileDir + modelSetName
    trainingFileNameOut = trainingDirNameOut + "/{}_caps{}_colorbins{}_normalbin{}_{}.captures".format(modelSetName, numCapsDefault, numHistBinsHSV, numHistBinsNormals, strDT)
    return(trainingDirNameOut, trainingFileNameOut)

# --------------------------------- SaveTrainingModel()
def SaveTrainingModel(labeledFeaturesModelAccum, trainingFileNameOut):
    print("Saving training set file {}".format(os.path.abspath(trainingFileNameOut)))
    dirName = os.path.dirname(trainingFileNameOut)
    if not os.path.exists(dirName):
        os.makedirs(dirName)
    pickle.dump(labeledFeaturesModelAccum, open(trainingFileNameOut, 'wb'))

#--------------------------------- get_normals()
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

#--------------------------------- Test_ImgRec_CalHistogram()
def CaptureCloud(numRetriesPerCapture = 3):
    try_count = 0
    sample_was_good = False

    while not sample_was_good and try_count < numRetriesPerCapture:
        print(str(try_count)),
        sys.stdout.flush()

        sample_cloud = capture_sample()
        sample_cloud_arr = pcl_helper.ros_to_pcl(sample_cloud).to_array()

        # Check for invalid clouds.
        if sample_cloud_arr.shape[0] == 0:
            print('*BAD,'),
            try_count += 1
        else:
            print('good,'),
            sample_was_good = True
        sys.stdout.flush()

    return sample_was_good, sample_cloud

#--------------------------------- CaptureFeaturesOfModel()
def CaptureFeaturesOfModel(modelName, numCapturesReq, numHistBinsHSV, numHistBinsNormals):
    labeledFeaturesModel = []
    spawn_model(modelName)
    numRetriesPerCapture = g_numRetriesPerCapture

    print('Capclouds({}):'.format(numCapturesReq)),
    for index in range(int(numCapturesReq)):
        sample_was_good, sample_cloud = CaptureCloud(numRetriesPerCapture)
        if (sample_was_good):
            # Extract histogram features
            try:
                chists = pclproc.compute_color_histograms(sample_cloud, numHistBinsHSV, doConvertToHSV=True)
                normals = get_normals(sample_cloud)
                nhists = pclproc.compute_normal_histograms(normals, numHistBinsNormals)

                feature = np.concatenate((chists, nhists))
                labeledFeaturesModel.append([feature, modelName])
            except Exception as exc:
                print ("Ignoring bad histogram...")

    delete_model()
    return labeledFeaturesModel


#--------------------------------- Test_ImgRec_CalHistogram()
def CaptureFeaturesOfModelList(modelNames, numHistBinsHSV, numHistBinsNormals):
    labeledFeaturesModelAccum = []


    for modelName, numCaps in modelNames:
        print ("CaptureFeatures({:11}):".format(modelName) ),
        sys.stdout.flush()
        labeledFeaturesModel = CaptureFeaturesOfModel(modelName, numCaps, numHistBinsHSV, numHistBinsNormals)
        print " numCaptured = ", str(len(labeledFeaturesModel))
        labeledFeaturesModelAccum += labeledFeaturesModel

    return labeledFeaturesModelAccum

#--------------------------------- Main()
def Main(modelSetName, numCapsDefault, numHistBinsHSV, numHistBinsNormals):
    rospy.init_node('capture_node')
    initial_setup()# Disable gravity and delete the ground plane
    print("capture_features.py Main() started.")

    modelNames = SelectModelNames(modelSetName, numCapsDefault)
    labeledFeaturesModelAccum = CaptureFeaturesOfModelList(modelNames, numHistBinsHSV, numHistBinsNormals)

    trainingDirNameOut, trainingFileNameOut = GetTrainingSetName(modelSetName, numCapsDefault, numHistBinsHSV, numHistBinsNormals)
    SaveTrainingModel(labeledFeaturesModelAccum, trainingFileNameOut)

#--------------------------------- Main() Invocation
if __name__ == '__main__':
    numCapsDefault = g_numCapsDefault
    modelSetName = g_modelSetName
    numHistBinsHSV = g_numHistBinsHSV
    numHistBinsNormals = g_numHistBinsNormals

    Main(modelSetName, numCapsDefault, numHistBinsHSV, numHistBinsNormals)


# object_list:
#   - name: biscuits
#     group: green
#   - name: soap
#     group: green
#   - name: soap2
#     group: red
#         "P3World1" : [('biscuits', numCapsDefault), ('soap', numCapsDefault), ('soap2', numCapsDefault)]
# object_list:
#   - name: biscuits
#     group: green
#   - name: soap
#     group: green
#   - name: book
#     group: red
#   - name: soap2
#     group: red
#   - name: glue
#     group: red
#         "P3World2" : [('biscuits', numCapsDefault), ('soap', numCapsDefault), ('soap2', numCapsDefault), ('book', numCapsDefault), ('glue', numCapsDefault)],
# object_list:
#   - name: sticky_notes
#     group: red
#   - name: book
#     group: red
#   - name: snacks
#     group: green
#   - name: biscuits
#     group: green
#   - name: eraser
#     group: red
#   - name: soap2
#     group: green
#   - name: soap
#     group: green
#   - name: glue
#     group: red
#         "P3World3" : [('biscuits', numCapsDefault), ('soap', numCapsDefault), ('soap2', numCapsDefault), ('book', numCapsDefault), ('glue', numCapsDefault), ('snacks', numCapsDefault), ('eraser', numCapsDefault), ('sticky_notes', numCapsDefault)]

