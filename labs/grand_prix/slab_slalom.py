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
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

#rc = racecar_core.create_racecar()

# Add any global variables here
TOP_LEFT = None
BOTTOM_RIGHT = None

########################################################################################
# Functions
########################################################################################


def start(robot: racecar_core.Racecar):
    global rc, BOTTOM_RIGHT
    rc = robot

    TOP_LEFT = (rc.camera.get_height() // 2, rc.camera.get_width())
    BOTTOM_RIGHT = (rc.camera.get_height(), rc.camera.get_width())

    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> 8 - Slab Slalom")


def update():
    color_image = rc.camera.get_color_image()
    cropped_image = rc_utils.crop(color_image, )
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
