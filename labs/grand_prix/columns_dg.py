"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import enum
import sys
import cv2 as cv
import numpy as np
import math

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

# Add any global variables here

########################################################################################
# Functions
########################################################################################

LEFT_WINDOW = (-50, -40) # center : 45
RIGHT_WINDOW = (40, 50) # center : 45
DRIVE_SPEED = 1.0
# MIN_SPEED = 0.2

class Mode(IntEnum) :
    TurnLeft = 0,
    TurnRight = 1,
    WallFollow = 2

robot_mode = None

def start(robot: racecar_core.Racecar):
    global rc
    rc = robot
    # Have the car begin at a stop
    rc.drive.stop()
    rc.drive.set_max_speed(0.25)

    # Print start message


def update():
    global robot_mode, speed, angle
    # Follow the wall to the right of the car without hitting anything.
    scan = rc.lidar.get_samples()

    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    if len(markers) != 0:
        corners = markers[0].get_corners()
        if (corners[0][0] - corners[2][0]) * (corners[0][1] - corners[2][1]) < 4000:
            print("DETECTED")
            orientation = markers[0].get_orientation()
    else : orientation = None

    
    # if rc_utils.get_lidar_closest_point(scan, (-15, 15))[1] < 70:
    #     print("NONE")
    #     orientation = None

    if orientation == None: # for no AR code
        robot_mode = Mode.WallFollow
    elif orientation.value == 1: # for left
        robot_mode = Mode.TurnLeft
    elif orientation.value == 3: # for right
        robot_mode = Mode.TurnRight

    left_angle, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW)
    right_angle, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW)

    rc.display.show_lidar(scan, 128, 1000, [(left_angle, left_dist), (right_angle, right_dist)])

    if robot_mode == Mode.WallFollow :
        error = right_dist - left_dist
        maxError = 12
        kP = 0.5   
        speed = DRIVE_SPEED

        # speed = rc_utils.clamp(math.cos(0.5 * math.pi * angle) * DRIVE_SPEED  + MIN_SPEED, -1, 1) # smoothened version of -abs(angle) + 1
        # https://www.desmos.com/calculator/24qctllaj1
        
        print("Error: " + str(error))

    elif robot_mode == Mode.TurnLeft :
        error = corners[0][0] - rc.camera.get_width() / 2
        maxError = rc.camera.get_width() / 2
        kP = 1.0
        speed = DRIVE_SPEED

    elif robot_mode == Mode.TurnRight :
        error = corners[0][0] - rc.camera.get_width() / 2
        maxError = rc.camera.get_width() / 2
        kP = 1.0
        speed = DRIVE_SPEED
    
    angle = rc_utils.clamp(kP * error / maxError, -1, 1)

    rc.drive.set_speed_angle(speed, angle)
