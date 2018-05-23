#!/usr/bin/env python

##############
#### Your name: Priyanka Kulkarni
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
from sklearn.feature_extraction import image
from sklearn import svm
import pickle
import cv2
import cozmo
import time
import asyncio
from cozmo.util import degrees, Pose, radians
from matplotlib import pyplot as plt



class ImageClassifier:
    
    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)
        
        #create one large array of image data
        data = io.concatenate_images(ic)
        
        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]
        
        return(data,labels)

    def extract_image_features(self, data):
        '''
        This function featurizes the incoming data to be fed to the classifier
        '''
        feature_data = []
        for images in data:
            if len(images.shape) < 3:
                '''
                if the image matrix is already in 2D, keep the matrix as is
                '''
                img = images
            else:
                '''
                if the image is a 3D matrix, convert to grayscale
                '''
                img = cv2.cvtColor(images, cv2.COLOR_BGR2GRAY)

            '''
            Apply gaussian blur to filter out/ smooth unnecessary elements in the image
            '''
            img = cv2.GaussianBlur(img, (7, 7), 0)
            '''
            Use skimage's hog as a featurization method with tuned hyper-parameters, use L2 regularization
            '''
            fd = feature.hog(img, orientations=9, pixels_per_cell=(16, 16), cells_per_block=(4, 4), block_norm='L2',
                             visualise=None, transform_sqrt=False, feature_vector=True)
            feature_data.append(fd)
        feature_data = np.asarray(feature_data)
        return (feature_data)

    def train_classifier(self, train_data, train_labels):

        clf = svm.SVC(gamma=0.001, C=1., kernel = 'linear')
        '''
        Using skilearns SVM to train the multiclass classifier with tuned hyperparamaters    
        '''
        clf.fit(train_data, train_labels)
        self.classifer = clf


    def predict_labels(self, data):
        clf = self.classifer
        '''
        Predict labels for incoming data
        '''
        predicted_labels =clf.predict(data)
        return predicted_labels


def main():
    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)

    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n", metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))

    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n", metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))


if __name__ == "__main__":
    main()
