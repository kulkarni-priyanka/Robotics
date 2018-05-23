#!/usr/bin/env python

##############
#### Your name: Joyce Brombaugh
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
import matplotlib.pyplot as plt
from sys import argv
from sklearn.model_selection import cross_val_score
import time
import cozmo
import cv2
import numpy as np
import cozmo
import asyncio

from cozmo.util import degrees, distance_mm


class ImageClassifier:

    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir + "*.bmp", load_func=self.imread_convert)

        # create one large array of image data
        data = io.concatenate_images(ic)

        # extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]

        return (data, labels)

    def extract_image_features(self, data):
        # (orientation, pixels_per_cell, cells_per_block, block_norm)
        # Please do not modify the header above
        feature_data = []
        # extract feature vector from image data
        file = 0
        for d in data:
            if len(d.shape) < 3:
                gray = d
            else:
                gray = color.rgb2gray(d)
            blur = filters.gaussian(gray, sigma=1, output=gray)
            fd = feature.hog(blur, orientations=8, pixels_per_cell=(16, 16), cells_per_block=(9, 9), block_norm='L2',
                             feature_vector=True)
            feature_data.append(fd)

        # Please do not modify the return type below
        return (feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above

        # train model and save the trained model to self.classifier

        self.classifier = svm.SVC(C=1, kernel='linear')
        self.classifier.fit(train_data, train_labels)
        '''scores = cross_val_score(self.classifier, train_data, train_labels, cv=5)
        print("Scores: ", scores)
        print("Avg: ", sum(scores)/len(scores))'''

    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels

        predicted_labels = self.classifier.predict(data)

        # Please do not modify the return type below
        return predicted_labels

async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''
    '''The run method runs once the Cozmo SDK is connected.'''

    # Move lift down and tilt the head up


    img_clf = ImageClassifier()
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    raw = np.concatenate((train_raw, test_raw))
    labels = np.concatenate((train_labels, test_labels))
    # convert images into features
    train_data = img_clf.extract_image_features(raw)
    img_clf.train_classifier(train_data, labels)
    sliding_window = []
    id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}

    try:
        while True:
            predicted_labels = []
            id = None
            # get camera image

            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)


            # convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            image = np.array([opencv_image])
            test_data = img_clf.extract_image_features(image)
            np.save("img_inspection.npy", test_data)

            predicted_labels = img_clf.predict_labels(test_data)
            print(predicted_labels)
            action = robot.stop_all_motors()
            robot.move_lift(3)
            time.sleep(1)
            robot.move_lift(-3)
            robot.say_text("Found " + str(predicted_labels[0])).wait_for_completed()
            robot.set_all_backpack_lights(cozmo.lights.red_light)
            time.sleep(1)
            print("Done")
            break;


    except asyncio.TimeoutError:
        print("Didn't find a ball")
    finally:
        print("Hello")
        # look_around.stop()



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

    first = np.load('img_inspection.npy')
    live_test = img_clf.extract_image_features(first)
    predicted_labels = img_clf.predict_labels(live_test)
    print(predicted_labels)

    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n", metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))


if __name__ == "__main__":
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
    #main()
