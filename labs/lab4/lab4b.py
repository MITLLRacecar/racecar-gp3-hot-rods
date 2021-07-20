"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

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

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################

FRONT_WINDOW = (-10,10)
LEFT_WINDOW = (-50, -40) # center : -45
RIGHT_WINDOW = (40, 50) # center : 45
MAX_DEVIATION = 60
DRIVE_SPEED = 1.0
MIN_SPEED = 0.2

speed = 0
angle = 0
error = 0
front_dist = 0
left_dist = 0
right_dist = 0

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()
    rc.drive.set_max_speed(0.25)

    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global front_dist, left_dist, right_dist, speed, angle, error
    
    # TODO: Follow the wall to the right of the car without hitting anything.
    scan = rc.lidar.get_samples()
    front_angle, front_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    left_angle, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW)
    right_angle, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW)

    error = right_dist - left_dist

    rc.display.show_lidar(scan, 128, 1000, [(left_angle, left_dist), (right_angle, right_dist)])

    maxError = 12
    kP = 0.5

    angle = rc_utils.clamp(kP * error / maxError, -1, 1)
    speed = DRIVE_SPEED
    # speed = rc_utils.clamp(math.cos(0.5 * math.pi * angle) * DRIVE_SPEED  + MIN_SPEED, -1, 1) # smoothened version of -abs(angle) + 1
    # https://www.desmos.com/calculator/24qctllaj1
    
    print("Speed: " + str(speed))

    print("Error: " + str(error))

    rc.drive.set_speed_angle(speed, angle)

def go() :
    global speed, angle, robot_state


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
