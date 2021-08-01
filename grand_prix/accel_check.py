"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
"""

########################################################################################
# Imports
########################################################################################

import sys
from typing import Tuple
import cv2 as cv
import numpy as np
from numpy.testing._private.utils import jiffies

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

class State(IntEnum) :
    approaching = 0 # Go around red cone
    passing = 1 # Go around blue cone
    stopping = 2 # Finish line
    searching = 3 # Manual control until we re-aquire

class Cone(IntEnum) :
    red = 0
    blue = 1

robotState = State.approaching
coneVisible = None
coneApproaching = None

# The HSV ranges (min, max)
RED = ((165, 160, 180), (179, 200, 255))
BLUE = ((95, 150, 150), (120, 255, 255))
ALLCOLOR = ((0,50,50), (130,255,255))
WHITE = ((90, 30, 250), (110, 45, 255))
# FINALS

MAX_SPEED = 0.6
MIN_CONTOUR_AREA = 600

depthImage = None
colorImage = None
waypointCenter = (0,0)
coneCenter = None
speed = 0
angle = 0
counter = 0
coneCounter = 0
finishLine = False
distanceToCone = 0

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    print(rc.physics.get_angular_velocity()[2])
    print(rc.physics.get_linear_acceleration()[2])
    rc.drive.set_speed_angle(rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - rc.controller.get_trigger(rc.controller.Trigger.LEFT), rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0])
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
