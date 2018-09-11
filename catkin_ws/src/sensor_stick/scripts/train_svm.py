#!/usr/bin/env python
import pickle
import itertools
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
from sklearn import svm
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn import cross_validation
from sklearn import metrics

import os

#====================== GLOBALS =====================
g_fitKernelType = 'linear'
g_modelPickleversion = "1.0"

g_trainingFileNameIn = "./Assets/Training/P3World1/P3World1_caps75_colorbins64_normalbin100_2018-09-10T20:42:07.captures"

g_doPlots = True
g_doValidation = True


#--------------------------------- SaveClassifierModel()
def SaveClassifierModel(classifierModel, classifierFileNameOut):
    classifierFileNameOutFQ = os.path.abspath(classifierFileNameOut)
    print("Saving classifier modle file {}".format(classifierFileNameOutFQ))
    dirName = os.path.dirname(classifierFileNameOutFQ)
    if not os.path.exists(dirName):
        os.makedirs(dirName)
    pickle.dump(classifierModel, open(classifierFileNameOutFQ, 'wb'))

#--------------------------------- GetTrainingSetName()
def GetTrainingSetName():
    trainingFileNameIn = g_trainingFileNameIn
    return(trainingFileNameIn)

#--------------------------------- SaveClassifierModel()
def GetClassifierFileNameFromTrainingFileName(trainingFileNameIn):
    extOut = '.clfModel'
    trainingFileNameInNoExt = os.path.splitext(trainingFileNameIn)[0]
    suffixTemplate = "_2018-09-10T20:42:07"
    suffixLen = len(suffixTemplate)

    classifierFileNameOutStart = trainingFileNameInNoExt[:-suffixLen] + "_fit" + g_fitKernelType
    classifierFileNameOutEnd = trainingFileNameInNoExt[-suffixLen:] + extOut
    classifierFileNameOut = classifierFileNameOutStart + classifierFileNameOutEnd
    return(classifierFileNameOut)

#--------------------------------- MainTrain()
def MainTrain(fitKernelType, trainingSetFileNameIn):
    trainingSetFileNameInFQ = os.path.abspath(trainingSetFileNameIn)
    print("Reading training set {}".format(trainingSetFileNameInFQ))

    # Load training data from disk
    trainingSet = pickle.load(open(trainingSetName, 'rb'))

    # Format the features and labels for use with scikit learn
    feature_list = []
    label_list = []

    for item in trainingSet:
        if np.isnan(item[0]).sum() < 1:
            feature_list.append(item[0])
            label_list.append(item[1])

    print('Features in Training Set: {}'.format(len(trainingSet)))
    print('Invalid Features in Training set: {}'.format(len(trainingSet)-len(feature_list)))

    X = np.array(feature_list)

    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X)

    # Apply the scaler to X
    X_train = X_scaler.transform(X)
    y_train = np.array(label_list)

    # Convert label strings to numerical encoding
    encoder = LabelEncoder()
    y_train = encoder.fit_transform(y_train)

    # Create classifier
    clf = svm.SVC(kernel=fitKernelType)

    #Train the classifier
    clf.fit(X=X_train, y=y_train)

    trainingParams = {'X_train' : X_train, 'y_train' : y_train, 'encoder' : encoder, }
    classifierModel = {'pickleversion': g_modelPickleversion, 'trainingSetFileNameIn': trainingSetFileNameIn, 'classifier': clf, 'classes': encoder.classes_, 'scaler': X_scaler, 'kernel': fitKernelType}

    return(trainingParams, classifierModel)



###################################### TESTS ###########################
###################################### TESTS ###########################
###################################### TESTS ###########################
#--------------------------------- plot_confusion_matrix()
def plot_confusion_matrix(cm, classes,normalize=False, title='Confusion matrix', cmap=plt.cm.Blues):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    ax = plt.gca()
    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]

    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    #plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)

    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        plt.text(j, i, '{0:.2f}'.format(cm[i, j]),
                 horizontalalignment="center",
                 color="white" if cm[i, j] > thresh else "black")

    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)

    cbar = plt.colorbar(cax=cax)

    plt.ylabel('True label')
    plt.xlabel('Predicted label')

    plt.tight_layout()


# --------------------------------- Validate()
def Validate(trainingParams, classifierModel):
    encoderclasses = classifierModel['classes']
    clf = classifierModel['classifier']
    X_train = trainingParams['X_train']
    y_train = trainingParams['y_train']


    # Set up 5-fold cross-validation
    kf = cross_validation.KFold(len(X_train), n_folds=5, shuffle=True, random_state=1)

    # Perform cross-validation
    scores = cross_validation.cross_val_score(cv=kf, estimator=clf, X=X_train, y=y_train, scoring='accuracy')
    print('Scores: ' + str(scores))
    print('Accuracy: %0.2f (+/- %0.2f)' % (scores.mean(), 2 * scores.std()))

    # Gather predictions
    predictions = cross_validation.cross_val_predict(cv=kf, estimator=clf, X=X_train, y=y_train)

    accuracy_score = metrics.accuracy_score(y_train, predictions)
    print('accuracy score: ' + str(accuracy_score))

    confusion_matrix = metrics.confusion_matrix(y_train, predictions)

    #class_names = encoderclasses.tolist()

    if g_doPlots:
        plotTitle = os.path.basename(GetClassifierFileNameFromTrainingFileName(classifierModel['trainingSetFileNameIn']))

        plt.figure()
        # Plot non-normalized confusion matrix

        plt.subplot(121)
        plot_confusion_matrix(confusion_matrix, classes= encoderclasses, title='Confusion matrix, without normalization')

        # Plot normalized confusion matrix
        plt.subplot(122)
        plot_confusion_matrix(confusion_matrix, classes= encoderclasses, normalize=True, title='Normalized confusion matrix')

        plt.suptitle(plotTitle, fontsize=20)
        plt.subplots_adjust(left=0.05, right=0.95, top=0.95)# wspace=0.95,
        plt.show()


#====================== Main Invocation RunRosNode() =====================
if (__name__ == '__main__'):
    trainingSetName = g_trainingFileNameIn
    trainingParams, classifierModel = MainTrain(g_fitKernelType, trainingSetName)
    clasifierFileNameOut = GetClassifierFileNameFromTrainingFileName(g_trainingFileNameIn)
    SaveClassifierModel(classifierModel, clasifierFileNameOut)

    if g_doValidation:
        Validate(trainingParams, classifierModel)
