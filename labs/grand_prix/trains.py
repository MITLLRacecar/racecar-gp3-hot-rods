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
from enum import Enum

########################################################################################
# Global variables
########################################################################################

#rc = racecar_core.create_racecar()

class states(Enum):
    stopped = 0
    ready = 1
    going = 2


# Add any global variables here
state = states.stopped
last_distance = 1000
speed = 0
timer = 0

########################################################################################
# Functions
########################################################################################


def start(robot: racecar_core.Racecar):
    global rc
    rc = robot

    # Have the car begin at a stop
    rc.drive.set_max_speed(0.75)
    rc.drive.stop()

    # Print start message
    print(">> Grand Prix Part 7: Passing Through Trains")

def update():

    """
    PseudoCode Solution:

    Four Stage Machine:
    1. Stopped
    2. Ready
    3. Going

    Need to be 150 distance away from train (to gather enough speed). (Stopped --> Ready)

    Then, if distance suddenly changes, then accelerate. This indicates that a
    train has just left the center. (Ready --> Going)

    Finally, if something is closer than 200 distance in front of the train, then stop.
    (Going --> Stopped)
    """

    # Defining global variables
    global state
    global last_distance
    global speed
    global timer

    # Getting the three sensor inputs from the car
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    scan = rc.lidar.get_samples()

    distance = scan[0]

    if distance == 0:
        distance = 10000
    print(distance, rc.physics.get_angular_velocity()[2])

    # State machine
    if state == states.stopped:
        if (distance > 150 and distance < 250) or distance > 400: # if the train is far away enough but close enough to actually exist
            state = states.ready
            print("THE STATE HAS BEEN CHANGED")
        elif scan[360] > 30: speed = -0
    elif state == states.ready:
        speed = 0
        print("distance change", distance - last_distance)
        if last_distance - distance < -100:
            state = state.going
            timer = 0
        last_distance = distance
    else: # this is state.going
        speed = 1
        timer += rc.get_delta_time()
        if timer > 0.8 or (distance < 300): #and rc.physics.get_angular_velocity()[2] > 0.5):
            state = states.stopped
    
    print(state)

    # !!! Manual Controls
    """
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    """

    # Setting the speed and the angle to actually move the car
    if speed == 0:
        print("STOPPED")
        rc.drive.stop()
    else:
        rc.drive.set_speed_angle(speed, 0)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
