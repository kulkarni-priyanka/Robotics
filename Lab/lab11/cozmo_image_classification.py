#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np
import cozmo
import time
from cozmo.util import degrees, distance_mm
import operator
import lab11.imgclassification

async def run(robot: cozmo.robot.Robot):
   
    await robot.set_lift_height(0).wait_for_completed()
    await robot.set_head_angle(degrees(0)).wait_for_completed()
    '''
    Above two steps will move lift down and tilt the head up
    '''

    clf = lab11.imgclassification.ImageClassifier()
    (train_orig, train_labels) = clf.load_data_from_folder('./train/')
    (test_orig, test_labels) = clf.load_data_from_folder('./test/')
    '''
    Since we want to leverage maximum data for prediction, I am combining bot train and test images to form a larger dataset
    '''
    data = np.concatenate((train_orig, test_orig))
    labels = np.concatenate((train_labels, test_labels))

    train_data = clf.extract_image_features(data) #Featurizing teh combined dataset
    clf.train_classifier(train_data, labels) #building model

    label_detected = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}

    counter = 0
    final_label = ''

    try:
        while True:

            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30) #getting camera image

            if(counter > 10):
                '''
                get max of 10 image predictions before deciding the label so as to avoid any false positives
                '''
                final_label = max(label_detected.items(), key=operator.itemgetter(1))[0]
            else:
                #convert camera image to opencv format
                opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
                image = np.array([opencv_image])
                #extract features
                test_data = clf.extract_image_features(image)
                #predict label
                predicted_labels = clf.predict_labels(test_data)
                label_detected[predicted_labels[0]] +=1
                counter +=1

            if(final_label !=''):
                robot.stop_all_motors()
                #move lift up and dowm
                robot.move_lift(3)
                time.sleep(1)
                robot.move_lift(-3)
                #declare what the label is
                await robot.say_text("I see " + final_label).wait_for_completed()
                #perform animation/action based on label
                if(final_label =='drone'):
                    robot.set_all_backpack_lights(cozmo.lights.red_light)
                    time.sleep(1)
                    robot.set_all_backpack_lights(cozmo.lights.off_light)
                elif(final_label =='hands'):
                    robot.set_all_backpack_lights(cozmo.lights.blue_light)
                    time.sleep(1)
                    robot.set_all_backpack_lights(cozmo.lights.off_light)
                elif (final_label == 'inspection'):
                    await robot.set_head_angle(degrees(45)).wait_for_completed()
                    await robot.set_head_angle(degrees(0)).wait_for_completed()
                elif (final_label == 'order'):
                    robot.set_all_backpack_lights(cozmo.lights.white_light)
                    time.sleep(1)
                    robot.set_all_backpack_lights(cozmo.lights.off_light)
                elif (final_label == 'place'):
                    robot.move_lift(3)
                    time.sleep(1)
                    robot.move_lift(-3)
                elif (final_label == 'plane'):
                    robot.set_all_backpack_lights(cozmo.lights.green_light)
                    time.sleep(1)
                    robot.set_all_backpack_lights(cozmo.lights.off_light)

                elif (final_label == 'truck'):
                    await robot.play_anim(name="anim_poked_giggle").wait_for_completed()

                time.sleep(2)
                '''
                Reset all values
                '''
                final_label = ''
                counter = 0
                label_detected = dict.fromkeys(label_detected, 0)





    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
