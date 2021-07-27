"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Grand Prix
- Detect white AR tag to stop line following and begin wall following
"""


########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
fom enum import Enum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
RIGHT_WINDOW = 45
LEFT_WINDOW = 675
FRONT_WINDOW = (-10, 10)

########################################################################################
# Functions
########################################################################################


def start():
    # Have the car begin at a stop
    rc.drive.stop()
    rc.drive.set_max_speed(0.35)

    # Print start message
    print(">> Final Challenge - Grand Prix - wall_following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    potential_colors = [
    ((0, 0, 240), (255, 20, 255), "white")
    ]

    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    scan = rc.lidar.get_samples()
    right_dist = rc_utils.get_lidar_average_distance(scan, RIGHT_WINDOW, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, LEFT_WINDOW, 10)
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)

    if markers:
        for border in markers:
            border.detect_colors(color_image, potential_colors)
            if border.get_id() == 0:
                angle = rc_utils.remap_range(right_dist - left_dist, -70, 70, -1, 1)
                angle = rc_utils.clamp(angle, -1, 1)
                speed = rc_utils.remap_range(forward_dist, 25, 80, 0.1, 1)
                speed = rc_utils.clamp(speed, -1, 1)
                rc.drive.set_speed_angle(speed, angle)
                rc.display.show_lidar(scan)
    
    pass

    





########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
