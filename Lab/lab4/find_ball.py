#!/usr/bin/env python3

import cv2
import sys
import copy

import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit('install Pillow to run this code')


def find_ball(opencv_image, debug=False):
    """Find the ball in an image.

        Arguments:
        opencv_image -- the image
        debug -- an optional argument which can be used to control whether
                debugging information is displayed.

        Returns [x, y, radius] of the ball, and [0,0,0] or None if no ball is found.
    """

    ball = None
    opencv_image = cv2.GaussianBlur(opencv_image, (7, 7), 0)
    circles = cv2.HoughCircles(opencv_image, cv2.HOUGH_GRADIENT, 1, 30, param1=140, param2=20)
    img = opencv_image
    img1 = img

    # Create mask
    height, width = img.shape

    cnt = 0
    if circles is not None:
        min_pix_val = 255
        selected_circle = None
        for i in circles[0, :]:
            mask = np.zeros((height, width), np.uint8)
            avg_pixel_val = get_avg_color(i, mask, img1)
            if avg_pixel_val < min_pix_val: #find the cropped circle with minimum average pixel value
                min_pix_val = avg_pixel_val
                selected_circle = i

            cnt += 1

        if (min_pix_val > 50): #if the average pixel value of the cropped circle is > 50 (not closer to black), set it to None
            selected_circle = None #this is done to get rid of false positive

        ball = selected_circle

    else:
        ball = None

    return ball


def get_avg_color(i, mask, img1):
    '''

    :param i: circle returened by HoughCircles
    :param mask: mask image
    :param img1: original image
    :return:
    '''
    i[2] = i[2] + 4
    # Draw on mask
    cv2.circle(mask, (i[0], i[1]), i[2], (255, 255, 255), thickness=-1)

    # Copy that image using that mask
    masked_data = cv2.bitwise_and(img1, img1, mask=mask)

    # Apply Threshold
    _, thresh = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)

    # Find Contour
    contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv2.boundingRect(contours[0])

    # Crop masked_data
    crop = masked_data[y:y + h, x:x + w]

    #
    # cv2.imshow('detected Edge', img1)
    # cv2.imshow('Cropped Ball', crop)
    # print("Avergae pixel value :" + str(np.average(crop)))
    # cv2.waitKey(0)

    return (np.average(crop)) #return average color of the cropped image


def display_circles(opencv_image, circles, best=None):
    """Display a copy of the image with superimposed circles.

       Provided for debugging purposes, feel free to edit as needed.

       Arguments:
        opencv_image -- the image
        circles -- list of circles, each specified as [x,y,radius]
        best -- an optional argument which may specify a single circle that will
                be drawn in a different color.  Meant to be used to help show which
                circle is ranked as best if there are multiple candidates.

    """
    # make a copy of the image to draw on
    circle_image = copy.deepcopy(opencv_image)
    circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)

    for c in circles:
        # draw the outer circle
        cv2.circle(circle_image, (c[0], c[1]), c[2], (255, 255, 0), 2)
        # draw the center of the circle
        cv2.circle(circle_image, (c[0], c[1]), 2, (0, 255, 255), 3)
        # write coords
        cv2.putText(circle_image, str(c), (c[0], c[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (255, 255, 255), 2, cv2.LINE_AA)

    # highlight the best circle in a different color
    if best is not None:
        # draw the outer circle
        cv2.circle(circle_image, (best[0], best[1]), best[2], (0, 0, 255), 2)
        # draw the center of the circle
        cv2.circle(circle_image, (best[0], best[1]), 2, (0, 0, 255), 3)
        # write coords
        cv2.putText(circle_image, str(best), (best[0], best[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (255, 255, 255), 2, cv2.LINE_AA)

    # display the image
    pil_image = Image.fromarray(circle_image)
    pil_image.show()


if __name__ == "__main__":
    pass
