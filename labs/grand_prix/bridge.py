"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Grand Prix
"""

########################################################################################
# Imports
########################################################################################

import sys
from typing import Tuple
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

# rc = racecar_core.create_racecar()
#PURPLE = ((90, 120, 120), (120, 255, 255)) 
PURPLE = ((130, 140, 0), (140, 255, 255))
ORANGE = ((10, 50, 150), (30, 255, 255))
CROP_FLOOR = None
MIN_CONTOUR_AREA = 100

# Add any global variables here

########################################################################################
# Functions
########################################################################################

def get_area_contours(contours) :
    sum = 0
    for contour in contours :
        if contour is not None : sum += rc_utils.get_contour_area(contour)

    return sum

def get_two_largest_contours(contours):
    if len(contours) == 0:
        return None, None
    
    contoursAreas = []
    for contour in contours :
        contoursAreas.append(rc_utils.get_contour_area(contour))

    maxContour = contours[contoursAreas.index(max(contoursAreas))]
    maxContour2 = None

    contoursAreas.pop(contoursAreas.index(max(contoursAreas)))

    if len(contoursAreas) > 0 :  maxContour2 = contours[contoursAreas.index(max(contoursAreas))]

    return maxContour, maxContour2


def start(robot: racecar_core.Racecar):
    global rc, CROP_FLOOR
    rc = robot
    # Have the car begin at a stop
    rc.drive.stop()

    CROP_FLOOR = ((rc.camera.get_height() - 170, 0), (rc.camera.get_height(), rc.camera.get_width()))

    # Print start message
    print(">> Grand Prix Part 9: The Leap of Faith")


def update():
# Find the largest contour
    image = rc.camera.get_color_image()

    # Get the AR markers BEFORE we crop it
    markers = rc_utils.get_ar_markers(image)
   
    # Crop the image
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    orange_contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
    purple_contours = rc_utils.find_contours(image, PURPLE[0], PURPLE[1])

    orange_largest = get_two_largest_contours(orange_contours)
    purple_largest = get_two_largest_contours(purple_contours)

    orange_area = get_area_contours(orange_largest)
    purple_area = get_area_contours(purple_largest)

    if orange_largest[0].any() != None and orange_largest[1].any() != None : orange_centers = rc_utils.get_contour_center(orange_largest[0]), rc_utils.get_contour_center(orange_largest[1])
    if purple_largest[0].any() != None and purple_largest[1].any() != None : purple_centers = rc_utils.get_contour_center(purple_largest[0]), rc_utils.get_contour_center(purple_largest[1])

    if orange_area > purple_area:
        waypoint = (0,0)
        print("orange")
    else : 
        waypoint = (0,0)
        print("purple")

    waypoint = tuple(0.5*x for x in waypoint)
    print(waypoint)

    rc_utils.draw_circle(waypoint)
    rc.display.show_color_image(image)
    
    # temp manual controls
    manual_speed = 0
    manual_angle = 0

    manual_speed -= rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    manual_speed += rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    manual_angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    rc.drive.set_speed_angle(manual_speed, manual_angle)
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
