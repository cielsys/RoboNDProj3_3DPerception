#!/usr/bin/env python
import matplotlib

def Backend_Switch(whichBackEnd):
    oldBackEnd = matplotlib.get_backend()
    matplotlib.use(whichBackEnd, warn=False, force=True)
    newBackEnd = matplotlib.get_backend()
    print "Switched matplotlib backend from {} to {}".format(oldBackEnd, newBackEnd)

Backend_Switch('QT4Agg')
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import glob
import time
#import math
#import os
#import sys
#import pickle
#import collections
#import types
#import traceback
#import pprint
#pp = pprint.PrettyPrinter(indent=4)

from sklearn import svm
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split # scikit-learn >= 0.18
#from sklearn.cross_validation import train_test_split # for scikit-learn version <= 0.17

#====================== GLOBALS =====================
# For testing only
g_doTests = True
g_doPlots = True

#--------------------------------- ImgRecog_CalcHistogram()
def ImgRecog_ExtractHistFeaturesFromImage(imgIn, numBins, binRange=(0, 256), doConvertToHSV=True):
    """
    Compute color histogram features
    :param img:
    :param numBins:
    :param binRange:
    :return:
    """

    if (doConvertToHSV):
        imgIn = cv2.cvtColor(imgIn, cv2.COLOR_RGB2HSV)

    # Compute the histogram of the RGB or HSV channels separately
    histR = np.histogram(imgIn[:, :, 0], bins=numBins, range=binRange)
    histG = np.histogram(imgIn[:, :, 1], bins=numBins, range=binRange)
    histB = np.histogram(imgIn[:, :, 2], bins=numBins, range=binRange)


    # Concatenate the histograms into a single feature vector
    histConsolidated = np.concatenate((histR[0], histG[0], histB[0])).astype(np.float64)

    # Normalize the result
    histNormedFeatures = histConsolidated / np.sum(histConsolidated)

    # Return the feature vector
    histograms = [histR, histG, histB]
    return histNormedFeatures, histograms


#--------------------------------- ImgRecog_TrainSVCPointClusters()
def ImgRecog_TrainSVC2DPointClusters(clustersX, clustersY, labels):
    """
    Use scikit SVM
    :param clustersX:
    :param clustersY:
    :param labels:
    :param numClusters:
    :return:
    """
    #  see http://scikit-learn.org/stable/modules/generated/sklearn.svm.SVC.html#sklearn.svm.SVC

    # Convert to a training dataset in sklearn format
    X = np.float32((np.concatenate(clustersX), np.concatenate(clustersY))).transpose()
    y = np.float32((np.concatenate(labels)))

    # Create an instance of SVM and fit the data.
    kernel = "poly" # linear poly rbf sigmoid precomputed
    if (kernel != "linear"):
        print("Fitting classifier with", kernel, "kernel... Could take a long time...")
    newSVCClassifier = svm.SVC(kernel=kernel).fit(X, y)

    # Create a mesh that we will use to colorfully plot the decision surface
    # Plotting Routine courtesy of: http://scikit-learn.org/stable/auto_examples/svm/plot_iris.html#sphx-glr-auto-examples-svm-plot-iris-py
    # Note: this coloring scheme breaks down at > 7 clusters or so

    h = 0.25  # step size in the mesh
    x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1  # -1 and +1 to add some margins
    y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
    xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))

    # Classify each block of the mesh (used to assign its color)
    Z = newSVCClassifier.predict(np.c_[xx.ravel(), yy.ravel()])

    return X, y, Z, xx, yy, kernel


#--------------------------------- ImgRecog_ExtractHistogramFeaturesFromImageFiles()
def ImgRecog_ExtractHistFeaturesFromImageFiles(imageFileNames, numBins, binRange=(0, 256)):
    """
    Extract features from a list of images
    """

    listHistNormedFeatures = []
    for imageFileName in imageFileNames:

        imgRaw = mpimg.imread(imageFileName)
        histNormedFeatures, histograms = ImgRecog_ExtractHistFeaturesFromImage(imgRaw, numBins, binRange, doConvertToHSV=True)

        # Append the new feature vector to the features list
        listHistNormedFeatures.append(histNormedFeatures)

    return listHistNormedFeatures


#--------------------------------- ImgRecog_TrainBinarySVCImageClassifier()
def ImgRecog_TrainBinarySVCImageClassifier(carsFilenamesIn, notCarsFilenamesIn, numBins, binRange=(0, 256)):
    """

    :param carsFilenamesIn:
    :param notCarsFilenamesIn:
    :param numBins:
    :param binRange:
    :return:
    """

    featuresCar = ImgRecog_ExtractHistFeaturesFromImageFiles(carsFilenamesIn, numBins, binRange)
    featuresNotCar = ImgRecog_ExtractHistFeaturesFromImageFiles(notCarsFilenamesIn, numBins, binRange)

    # Create an array stack of feature vectors
    X = np.vstack((featuresCar, featuresNotCar)).astype(np.float64)

    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X)

    # Apply the scaler to X
    scaled_X = X_scaler.transform(X)

    # Define the labels vector
    y = np.hstack((np.ones(len(featuresCar)), np.zeros(len(featuresNotCar))))

    # Split up data into randomized training and test sets
    rand_state = np.random.randint(0, 100)
    X_train, X_test, y_train, y_test = train_test_split(scaled_X, y, test_size=0.2, random_state=rand_state)

    # Use a linear SVC
    newSVCClassifier = SVC(kernel='linear')

    # Check the training time for the SVC
    t0=time.time()
    newSVCClassifier.fit(X_train, y_train)
    t1 = time.time()
    tTrain = round(t1 - t0, 2)

    # Check the score of the SVC
    testAccuracy = round(newSVCClassifier.score(X_test, y_test), 4)

    # Check the prediction time for a single sample
    t0=time.time()
    n_predict = 10
    prediction = newSVCClassifier.predict(X_test[0:n_predict])
    t1 = time.time()
    tPredict = round(t1 - t0, 5)

    print('Dataset includes', len(carsFilenamesIn), 'cars and', len(carsFilenamesIn), 'not-cars')
    print('Using', numBins,'histogram bins')
    print('Feature vector length:', len(X_train[0]))
    print(tTrain, 'Seconds to train SVC...')
    print('Test Accuracy of SVC = ', testAccuracy)
    print('My SVC predicts: ', prediction)
    print('For these', n_predict, 'labels: ', y_test[0:n_predict])
    print(tPredict, 'Seconds to predict', n_predict, 'labels with SVC')

    return newSVCClassifier


###################################### TESTS ###########################
###################################### TESTS ###########################
###################################### TESTS ###########################


#--------------------------------- Test_ImgRecog_SupportVectorMachine()
def GenerateLabeledClusters(numClusters, pts_minmax=(100, 500), x_mult=(2, 7), y_mult=(2, 7), x_off=(0, 50), y_off=(0, 50), randomSeed=424):
    """
    Generate clusters of 2D points
    :param numClusters: number of clusters to generate
    :param pts_minmax: range of number of points per cluster
    :param x_mult: range of multiplier to modify the size of cluster in the x-direction
    :param y_mult: range of multiplier to modify the size of cluster in the y-direction
    :param x_off: range of cluster position offset in the x-direction
    :param y_off: range of cluster position offset in the y-direction
    :param randomSeed: Seed for random generator
    :return:
    """

    np.random.seed(randomSeed)  # Change the number to generate a different cluster.

    # Initialize some empty lists to receive cluster member positions
    clustersX = []
    clustersY = []
    labels = []
    # Generate random values given parameter ranges
    n_points = np.random.randint(pts_minmax[0], pts_minmax[1], numClusters)
    x_multipliers = np.random.randint(x_mult[0], x_mult[1], numClusters)
    y_multipliers = np.random.randint(y_mult[0], y_mult[1], numClusters)
    x_offsets = np.random.randint(x_off[0], x_off[1], numClusters)
    y_offsets = np.random.randint(y_off[0], y_off[1], numClusters)

    # Generate random clusters given parameter values
    for idx, npts in enumerate(n_points):

        xpts = np.random.randn(npts) * x_multipliers[idx] + x_offsets[idx]
        ypts = np.random.randn(npts) * y_multipliers[idx] + y_offsets[idx]
        clustersX.append(xpts)
        clustersY.append(ypts)
        labels.append(np.zeros_like(xpts) + idx)

    # Return cluster positions and labels
    return clustersX, clustersY, labels


#--------------------------------- PlotHistogram()
def PlotHistogram(histograms):

    # Generating bin centers
    bin_edges = histograms[0][1]
    bin_centers = (bin_edges[1:] + bin_edges[0:len(bin_edges) - 1]) / 2

    # Plot a figure with all three bar charts
    fig = plt.figure(figsize=(12, 3))
    plt.subplot(131)
    plt.bar(bin_centers, histograms[0][0])
    plt.xlim(0, 256)
    plt.title('R or H Histogram')

    plt.subplot(132)
    plt.bar(bin_centers, histograms[1][0])
    plt.xlim(0, 256)
    plt.title('G or S Histogram')

    plt.subplot(133)
    plt.bar(bin_centers, histograms[2][0])
    plt.xlim(0, 256)
    plt.title('B or V Histogram')

    plt.show()

#--------------------------------- PlotHistogramFeatures()
def PlotHistogramFeatures(histNormedFeatures):
    fig = plt.figure(figsize=(12,6))
    plt.plot(histNormedFeatures)
    plt.title('RGB or HSV Feature Vector', fontsize=30)
    plt.tick_params(axis='both', which='major', labelsize=20)
    fig.tight_layout()
    plt.show()

#--------------------------------- PlotSVM()
def PlotSVM(X, y, Z, xx, yy, ker):

    # Put the result into a color plot
    Z = Z.reshape(xx.shape)
    plt.contourf(xx, yy, Z, cmap=plt.cm.coolwarm, alpha=0.8)

    # Plot the training points
    plt.scatter(X[:, 0], X[:, 1], c=y, cmap=plt.cm.coolwarm, edgecolors='black')

    plt.xlim(xx.min(), xx.max())
    plt.ylim(yy.min(), yy.max())
    plt.xticks(())
    plt.yticks(())
    plt.title('SVC with ' + ker + ' kernel', fontsize=20)
    plt.show()

#--------------------------------- Test_ImgRec_CalHistogram()
def Test_ImgRecog_CalcHistogram():
    dirNameIn = "./Assets/ImagesIn/tableobjects/"
    fileNameBaseIn = "beer_clean.jpg" # udacan hammer bowl dispart create beer
    fileNameIn = dirNameIn + fileNameBaseIn

    imgRaw = mpimg.imread(fileNameIn)
    histNormedFeatures, histograms = ImgRecog_ExtractHistFeaturesFromImage(imgRaw, numBins=32, binRange=(0, 256), doConvertToHSV=True)

    if (g_doPlots):
        plt.imshow(imgRaw)
        PlotHistogram(histograms)
        PlotHistogramFeatures(histNormedFeatures)

#--------------------------------- Test_ImgRecog_SupportVectorMachine()
def Test_ImgRecog_TrainSVC2DPointClusters():
    numClusters = 6
    clustersX, clustersY, labels = GenerateLabeledClusters(numClusters)
    X, y, Z, xx, yy, ker = ImgRecog_TrainSVC2DPointClusters(clustersX, clustersY, labels)
    PlotSVM(X, y, Z, xx, yy, ker)

#--------------------------------- Test_ImgRecog_TrainBinarySVCImageClassifier()
def Test_ImgRecog_TrainBinarySVCImageClassifier():
    # Read in car and non-car images
    carsDirNameIn = "./Assets/ImagesIn/cars/"
    notCarsDirNameIn = "./Assets/ImagesIn/notcars/"
    wildCard = '*.jpeg'

    carsFilenamesIn = glob.glob(carsDirNameIn + wildCard)
    notCarsFilenamesIn = glob.glob(notCarsDirNameIn + wildCard)

    numBins = 32
    binRange = (0, 256)
    newSVCClassifier = ImgRecog_TrainBinarySVCImageClassifier(carsFilenamesIn, notCarsFilenamesIn, numBins, binRange)

# ============ Auto invoke Test_ImgRecog*
if (g_doTests):
    Test_ImgRecog_CalcHistogram()
    Test_ImgRecog_TrainSVC2DPointClusters()
    Test_ImgRecog_TrainBinarySVCImageClassifier()
