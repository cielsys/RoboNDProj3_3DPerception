#!/usr/bin/env python
# import subprocess
import datetime
import matplotlib
import numpy as np
import cv2
from sklearn.cluster import DBSCAN
import pcl

# ROS imports
import sensor_msgs.point_cloud2 as pc2

# Local imports
import pcl_helper

#====================== GLOBALS =====================

# For testing only
g_doTests = False

#----------------------- PlottingBackend_Switch()
def PlottingBackend_Switch(whichBackEnd):
    import matplotlib
    matplotlib.use(whichBackEnd, warn=False, force=True)
    from matplotlib import pyplot as plt
    print "Switched to:", matplotlib.get_backend()

#PlottingBackend_Switch('QT4Agg')
import matplotlib.pyplot as plt

#----------------------- PCLProc_DownSampleVoxels()
def PCLProc_DownSampleVoxels(pclpcIn):
    # Create a VoxelGrid filter object for our input point cloud
    vox = pclpcIn.make_voxel_grid_filter()
    voxelSize = 0.01
    vox.set_leaf_size(voxelSize, voxelSize, voxelSize)
    # Call the filter function to obtain the resultant downsampled point cloud
    pclpcDownSampled = vox.filter()
    pclRecs = [(pclpcDownSampled, "pclpcDownSampled")]
    return pclRecs

#----------------------- PCLProc_PassThrough()
def PCLProc_PassThrough(pclpcIn, chAxis, min, max):
    pclRecs = [] # For dev/debug display. Container for point cloud records: tuple (pclObj, pclName)

    # Create a PassThrough filter object.
    filPassthrough = pclpcIn.make_passthrough_filter()
    filPassthrough.set_filter_field_name(chAxis)
    filPassthrough.set_filter_limits(min, max)

    # Finally use the filter function to obtain the resultant point cloud.
    pclpcPass = filPassthrough.filter()
    pclRecs.append((pclpcPass, "pclpcPass_" + chAxis))

    return pclRecs

#----------------------- PCLProc_Ransac()
def PCLProc_Ransac(pclpcIn):
    """
    RANSAC plane segmentation
    :param pclpcIn:
    :return:
    """
    pclRecs = [] # For dev/debug display. Container for point cloud records: tuple (pclObj, pclName)


    # Create the segmentation object
    seg = pclpcIn.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    pclpcRansacInliers = pclpcIn.extract(inliers, negative=False)
    pclpcRansacOutliers = pclpcIn.extract(inliers, negative=True)

    pclRecs.append((pclpcRansacInliers, "pclpcRansacInliers"))
    pclRecs.append((pclpcRansacOutliers, "pclpcRansacOutliers"))

    return(pclRecs)

#----------------------- PCLProc_Noise()
def PCLProc_Noise(pclpIn):
    pclRecs = [] # For dev/debug display. Container for point cloud records: tuple (pclObj, pclName)

    print("pcl.__file__", pcl.__file__)
    print("type pcl", type(pclpIn))

    if (not type(pclpIn) is pcl._pcl.PointCloud):
        pclpIn = pcl_helper.XYZRGB_to_XYZ(pclpIn)

    fil = pclpIn.make_statistical_outlier_filter()
    numNeighborsToCheck = 50
    threshScaleFactor = 0.5
    fil.set_mean_k(numNeighborsToCheck)
    fil.set_std_dev_mul_thresh(threshScaleFactor)

    pclpNoiseInliers = fil.filter()
    fil.set_negative(True)
    pclpNoiseOutliers = fil.filter()

    pclRecs.append((pclpNoiseInliers, "pclpNoiseInliers"))
    pclRecs.append((pclpNoiseOutliers, "pclpNoiseOutliers"))
    return(pclRecs)


#------------------------------ PCLProc_KMeans()
def PCLProc_KMeans(ptsIn):
    # Define k-means parameters
    # Number of clusters to define
    numClusters = 7
    # Maximum number of iterations to perform
    max_iter = 10
    # Accuracy criterion for stopping iterations
    epsilon = 1.0
    # Define criteria in OpenCV format
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, epsilon)
    # Call k-means algorithm on your dataset
    compactness, label, center = cv2.kmeans(ptsIn, numClusters, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    # NERVOUS - The provided call to kmeans had 'None' param not wanted in this version
    # compactness, label, center = cv2.kmeans(data, k_clusters, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    # Define some empty lists to receive k-means cluster points
    kmeansClustersx = []
    kmeansClustersy = []

    # Extract k-means clusters from output
    for idx in range(numClusters):
        kmeansClustersx.append(ptsIn[label.ravel() == idx][:, 0])
        kmeansClustersy.append(ptsIn[label.ravel() == idx][:, 1])

    return kmeansClustersx, kmeansClustersy


# ------------------------------ PCLProc_DBScan()
def PCLProc_DBScan(ptsIn):
    # Define max_distance (eps parameter in DBSCAN())
    max_distance = 1
    db = DBSCAN(eps=max_distance, min_samples=10).fit(ptsIn)

    # Extract a mask of core cluster members
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True

    # Extract labels (-1 is used for outliers)
    labels = db.labels_
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    unique_labels = set(labels)

    return core_samples_mask, labels, unique_labels


#--------------------------------- PCLProc_ExtractClusters()
def PCLProc_ExtractClusters(pclpObjectsIn):

    kdTreeCluster = pclpObjectsIn.make_kdtree()

    # Create a cluster extraction object
    clusterExtractor = pclpObjectsIn.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold & clusterSize min,max (in points)
    clusterExtractor.set_ClusterTolerance(0.015)
    clusterExtractor.set_MinClusterSize(120)
    clusterExtractor.set_MaxClusterSize(1500)

    # Search the k-d tree for clusters
    clusterExtractor.set_SearchMethod(kdTreeCluster)

    # Extract indices for each of the discovered clusters
    clusterIndices = clusterExtractor.Extract()

    # Assign a color corresponding to each segmented object in scene
    clusterColor = pcl_helper.get_color_list(len(clusterIndices))
    clusterColorPointList = []
    for j, indices in enumerate(clusterIndices):
        for i, indice in enumerate(indices):
            clusterColorPointList.append([pclpObjectsIn[indice][0],
                                          pclpObjectsIn[indice][1],
                                          pclpObjectsIn[indice][2],
                                          pcl_helper.rgb_to_float(clusterColor[j])])

    # Create new cloud containing all clusters, each with unique color
    pclpcClusters = pcl.PointCloud_PointXYZRGB()
    pclpcClusters.from_list(clusterColorPointList)
    return clusterIndices, pclpcClusters


#--------------------------------- PCLProc
def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0 * rgb_list[0] / 255, 1.0 * rgb_list[1] / 255, 1.0 * rgb_list[2] / 255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


#--------------------------------- PCLProc
def compute_color_histograms(cloud, numBins=32, binRange=(0, 256), doConvertToHSV=True):
    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = pcl_helper.float_to_rgb(point[3])
        if doConvertToHSV:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])

    # Compute the histogram of the RGB or HSV channels separately
    histR = np.histogram(channel_1_vals, bins=numBins, range=binRange)
    histG = np.histogram(channel_2_vals, bins=numBins, range=binRange)
    histB = np.histogram(channel_3_vals, bins=numBins, range=binRange)
    histograms = [histR, histG, histB]


    # Concatenate the histograms into a single feature vector
    histConsolidated = np.concatenate((histR[0], histG[0], histB[0])).astype(np.float64)

    # Normalize the result
    histNormedFeatures = histConsolidated / np.sum(histConsolidated)

    # Generate random features for demo mode.
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96)
    return histNormedFeatures


#--------------------------------- compute_normal_histograms
def compute_normal_histograms(normal_cloud, numBins=32, binRange=(0, 256)):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,field_names=('normal_x', 'normal_y', 'normal_z'),skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # Compute the histogram of the RGB or HSV channels separately
    histR = np.histogram(norm_x_vals, bins=numBins, range=binRange)
    histG = np.histogram(norm_y_vals, bins=numBins, range=binRange)
    histB = np.histogram(norm_z_vals, bins=numBins, range=binRange)
    histograms = [histR, histG, histB]


    # Concatenate the histograms into a single feature vector
    histConsolidated = np.concatenate((histR[0], histG[0], histB[0])).astype(np.float64)

    # Normalize the result
    histNormedFeatures = histConsolidated / np.sum(histConsolidated)

    # Generate random features for demo mode.
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96)
    return histNormedFeatures


###################################### TESTS ###########################
###################################### TESTS ###########################
###################################### TESTS ###########################

# Define a function to generate clusters
def Test_GenerateClusters(numClusters, pts_minmax=(10, 100), x_mult=(1, 4), y_mult=(1, 3), x_off=(0, 50), y_off=(0, 50)):
    """
    # n_clusters = number of clusters to generate
    # pts_minmax = range of number of points per cluster
    # x_mult = range of multiplier to modify the size of cluster in the x-direction
    # y_mult = range of multiplier to modify the size of cluster in the y-direction
    # x_off = range of cluster position offset in the x-direction
    # y_off = range of cluster position offset in the y-direction
    """

    # Initialize some empty lists to receive cluster member positions
    testClustersx = []
    testClustersy = []
    # Genereate random values given parameter ranges
    n_points = np.random.randint(pts_minmax[0], pts_minmax[1], numClusters)
    x_multipliers = np.random.randint(x_mult[0], x_mult[1], numClusters)
    y_multipliers = np.random.randint(y_mult[0], y_mult[1], numClusters)
    x_offsets = np.random.randint(x_off[0], x_off[1], numClusters)
    y_offsets = np.random.randint(y_off[0], y_off[1], numClusters)

    # Generate random clusters given parameter values
    for idx, npts in enumerate(n_points):
        xpts = np.random.randn(npts) * x_multipliers[idx] + x_offsets[idx]
        ypts = np.random.randn(npts) * y_multipliers[idx] + y_offsets[idx]
        testClustersx.append(xpts)
        testClustersy.append(ypts)

    # Convert to a single dataset in OpenCV format
    testClusters = np.float32((np.concatenate(testClustersx), np.concatenate(testClustersy))).transpose()

    # Return cluster positions
    return testClusters, testClustersx, testClustersy


#----------------------- SavePCLs()
def SavePCLs(pclRecs, dirNameOut, useTimeStamp=True):
    if (useTimeStamp):
        strDT = "_{:%Y-%m-%dT%H:%M:%S}".format(datetime.datetime.now())
    else:
        strDT = ""

    for pclObj, pclName in pclRecs:
        extOut = ".pcd"
        fileNameOutBase = pclName +  strDT + extOut
        fileNameOut= dirNameOut + fileNameOutBase
        pcl.save(pclObj, fileNameOut)
        print("Saving file ", fileNameOut)
        #subprocess.call(["pcl_viewer", fileNameOut])

#----------------------- Test_PCLProc_Ransac()
def Test_PCLProc_Ransac():
    pclRecs = [] # For dev/debug display. Container for point cloud records: tuple (pclObj, pclName)
    # Load Point Cloud file
    dirNameIn = "./Assets/pcdIn/"
    fileNameBaseIn = 'tabletop.pcd'
    fileNameIn = dirNameIn + fileNameBaseIn
    pclpcInRaw = pcl.load_XYZRGB(fileNameIn)
    pclRecs.append((pclpcInRaw, "pclpcInRaw"))

    pclRecsDownSampled = PCLProc_DownSampleVoxels(pclpcInRaw)
    pclpcDownSampled, pclpcDownSampledName = pclRecsDownSampled[0]
    pclRecs += pclRecsDownSampled

    pclRecsRansac = PCLProc_Ransac(pclpcDownSampled)
    pclRecs += pclRecsRansac

    dirNameOut = "./Assets/pcdOut/"
    SavePCLs(pclRecs, dirNameOut, useTimeStamp=True)

# ----------------------- Test_PCLProc_Noise()
def Test_PCLProc_Noise():
    pclRecs = [] # For dev/debug display. Container for point cloud records: tuple (pclObj, pclName)

    # Load Point Cloud file
    dirNameIn = "./Assets/pcdIn/"
    fileNameBaseIn = 'table_scene_lms400.pcd'
    fileNameIn = dirNameIn + fileNameBaseIn

    pclpRaw = pcl.load(fileNameIn)
    pclRecs.append((pclpRaw, "pclpcInRaw"))

    pclRecsDownSampled = PCLProc_DownSampleVoxels(pclpRaw)
    pclpDownSampled, pclpDownSampledName = pclRecsDownSampled[0]
    pclRecs += pclRecsDownSampled

    pclRecsNoise = PCLProc_Noise(pclpDownSampled)
    pclRecs += pclRecsNoise

    dirNameOut = "./Assets/pcdOut/"
    SavePCLs(pclRecs, dirNameOut, useTimeStamp=True)

#------------------------------ PlotClustersKMeans()
def PlotClustersKMeans(ptsIn, ptsInx, ptsIny, kmeansClustersx, kmeansClustersy):
    # Plot up a comparison of original clusters vs. k-means clusters
    fig = plt.figure(figsize=(12, 6))
    plt.subplot(121)
    min_x = np.min(ptsIn[:, 0])
    max_x = np.max(ptsIn[:, 0])
    min_y = np.min(ptsIn[:, 1])
    max_y = np.max(ptsIn[:, 1])
    for idx, xpts in enumerate(ptsInx):
        plt.plot(xpts, ptsIny[idx], 'o')
        plt.xlim(min_x, max_x)
        plt.ylim(min_y, max_y)
        plt.title('Original Clusters', fontsize=20)
    plt.subplot(122)

    for idx, xpts in enumerate(kmeansClustersx):
        plt.plot(xpts, kmeansClustersy[idx], 'o')
        plt.xlim(min_x, max_x)
        plt.ylim(min_y, max_y)
        plt.title('k-means Clusters', fontsize=20)
    fig.tight_layout()
    plt.subplots_adjust(left=0.03, right=0.98, top=0.9, bottom=0.05)
    plt.show()


#------------------------------ Test_PCLProc_KMeans()
def Test_PCLProc_KMeans():
    numClusters = 7
    ptsIn, ptsInx, ptsIny = Test_GenerateClusters(numClusters)

    # INVOCATION
    kmeansClustersx, kmeansClustersy = PCLProc_KMeans(ptsIn)
    PlotClustersKMeans(ptsIn, ptsInx, ptsIny, kmeansClustersx, kmeansClustersy)

#------------------------------ PlotClustersDBScan()
def PlotClustersDBScan(ptsIn, ptsInx, ptsIny, core_samples_mask, labels, unique_labels):
    n_clusters = 50
    # Plot up the results!
    min_x = np.min(ptsIn[:, 0])
    max_x = np.max(ptsIn[:, 0])
    min_y = np.min(ptsIn[:, 1])
    max_y = np.max(ptsIn[:, 1])

    fig = plt.figure(figsize=(12,6))
    plt.subplot(121)
    plt.plot(ptsIn[:,0], ptsIn[:,1], 'ko')
    plt.xlim(min_x, max_x)
    plt.ylim(min_y, max_y)
    plt.title('Original Data', fontsize = 20)

    plt.subplot(122)

    # The following is just a fancy way of plotting core, edge and outliers
    # Credit to: http://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html#sphx-glr-auto-examples-cluster-plot-dbscan-py
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = (labels == k)

        xy = ptsIn[class_member_mask & core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=7)

        xy = ptsIn[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor='k', markersize=3)
    plt.xlim(min_x, max_x)
    plt.ylim(min_y, max_y)
    plt.title('DBSCAN: %d clusters found' % n_clusters, fontsize = 20)
    fig.tight_layout()
    plt.subplots_adjust(left=0.03, right=0.98, top=0.9, bottom=0.05)
    plt.show()

#------------------------------ Test_PCLProc_DBScan
def Test_PCLProc_DBScan():
    numClusters = 7
    testClusters, testClustersx, testClustersy = Test_GenerateClusters(numClusters)

    # INVOCATION
    core_samples_mask, labels, unique_labels = PCLProc_DBScan(testClusters)
    PlotClustersDBScan(testClusters, testClustersx, testClustersy, core_samples_mask, labels, unique_labels)

# ============ Auto invoke Test_PCLProc_*
if (g_doTests):
    #Test_PCLProc_Ransac()
    Test_PCLProc_Noise()
    #Test_PCLProc_KMeans()
    #Test_PCLProc_DBScan()

