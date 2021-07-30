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
from numpy.lib.npyio import savez_compressed

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum
import math

########################################################################################
# Global variables
########################################################################################

# Add any global variables here

class State(IntEnum):
    goto_start = 0
    approach_train_1 = 1
    wait_train_1 = 2
    pass_train_1 = 3
    approach_train_2 = 4
    wait_train_2 = 5
    pass_train_2 = 6
    approach_train_3 = 7
    wait_train_3 = 8
    pass_train_3 = 9
    done = 10

cur_state: State = State.goto_start

########################################################################################
# Functions
########################################################################################

def start(robot: racecar_core.Racecar):
    global rc
    global speed
    global angle
    global cur_state
    global FRONT
    global LEFT
    global RIGHT
    global NEAR_RIGHT
    global NEAR_LEFT
    global FAR_RIGHT
    global FAR_LEFT
    global VERY_FAR_LEFT
    global VERY_FAR_RIGHT
    global WINDOW

    rc = robot
    # Have the car begin at a stop
    rc.drive.stop()

    rc.drive.set_max_speed(1.0)

    FRONT = 0
    LEFT = 270
    RIGHT = 90
    NEAR_RIGHT = 15
    NEAR_LEFT = 345
    FAR_RIGHT = 30
    FAR_LEFT = 330
    VERY_FAR_LEFT = 300
    VERY_FAR_RIGHT = 60
    WINDOW = 3
    speed = 0.0
    angle = 0.0

    # Print start message
    print(">> Final Challenge - Grand Prix")

def update():

    global speed
    global angle
    global cur_state
    global FRONT
    global LEFT
    global RIGHT
    global NEAR_RIGHT
    global NEAR_LEFT
    global FAR_RIGHT
    global FAR_LEFT
    global VERY_FAR_LEFT
    global VERY_FAR_RIGHT
    global WINDOW

    scan = rc.lidar.get_samples()

    front_dist = rc_utils.get_lidar_average_distance(scan, FRONT, WINDOW)
    left_dist = rc_utils.get_lidar_average_distance(scan, LEFT, WINDOW)
    right_dist = rc_utils.get_lidar_average_distance(scan, RIGHT, WINDOW)
    near_right_dist = rc_utils.get_lidar_average_distance(scan, NEAR_RIGHT, WINDOW)
    near_left_dist = rc_utils.get_lidar_average_distance(scan, NEAR_LEFT, WINDOW)
    far_right_dist = rc_utils.get_lidar_average_distance(scan, FAR_RIGHT, WINDOW)
    far_left_dist = rc_utils.get_lidar_average_distance(scan, FAR_LEFT, WINDOW)
    very_far_left_dist = rc_utils.get_lidar_average_distance(scan, VERY_FAR_LEFT, WINDOW)
    very_far_right_dist = rc_utils.get_lidar_average_distance(scan, VERY_FAR_RIGHT, WINDOW)

    front_dist = front_dist * math.cos(FRONT * math.pi / 180)
    near_right_dist = near_right_dist * math.cos(NEAR_RIGHT * math.pi / 180)
    near_left_dist = near_left_dist * math.cos(NEAR_LEFT * math.pi / 180)
    far_right_dist = far_right_dist * math.cos(FAR_RIGHT * math.pi / 180)
    far_left_dist = far_left_dist * math.cos(FAR_LEFT * math.pi / 180)

    if front_dist == 0.0:
        front_dist = 999.0
    if left_dist == 0.0:
        left_dist = 999.0
    if right_dist == 0.0:
        right_dist = 999.0
    if near_right_dist == 0.0:
        near_right_dist = 999.0
    if near_left_dist == 0.0:
        near_left_dist = 999.0
    if far_right_dist == 0.0:
        far_right_dist = 999.0
    if far_left_dist == 0.0:
        far_left_dist = 999.0
    if very_far_right_dist == 0.0:
        very_far_right_dist = 999.0
    if very_far_left_dist == 0.0:
        very_far_left_dist = 999.0

    wide_train_dist = min(far_right_dist, far_left_dist, front_dist, near_right_dist, near_left_dist)
    narrow_train_dist = min(front_dist, near_right_dist, near_left_dist)

    print(format(cur_state, '2d') + ": " + format(left_dist, '5.1f') + " " + format(right_dist, '5.1f') + " | " + format(very_far_left_dist, '5.1f') + " " + format(very_far_right_dist, '5.1f') + " | " + format(far_left_dist, '5.1f') + " " + format(near_left_dist, '5.1f') + " " + format(front_dist, '5.1f') + " " + format(near_right_dist, '5.1f') + " " + format(far_right_dist, '5.1f') + " | " + format(wide_train_dist, '5.1f') + " " + format(narrow_train_dist, '5.1f'))

    if cur_state == State.goto_start:
        rc.drive.set_speed_angle(0.5, 0.0)
        if very_far_left_dist < 175.0 and very_far_right_dist < 175.0:
            cur_state = State.approach_train_1

    elif cur_state == State.approach_train_1:
        angle = rc_utils.clamp(0.03 * (very_far_right_dist - very_far_left_dist), -1.0, +1.0)
        rc.drive.set_speed_angle(0.15, angle)
        if wide_train_dist < 200.0:
            cur_state = State.wait_train_1

    elif cur_state == State.wait_train_1:
        angle = rc_utils.clamp(0.03 * (very_far_right_dist - very_far_left_dist), -1.0, +1.0)
        rc.drive.set_speed_angle(0.0, angle)
        if front_dist < 200.0 and near_right_dist > 225.0:
            cur_state = State.pass_train_1

    elif cur_state == State.pass_train_1:
        rc.drive.set_speed_angle(1.0, 0.0)
        if very_far_left_dist > 100.0 and very_far_right_dist > 100.0 and wide_train_dist > 150.0:
            cur_state = State.approach_train_2

    elif cur_state == State.approach_train_2:
        angle = rc_utils.clamp(0.03 * (very_far_right_dist - very_far_left_dist), -1.0, +1.0)
        rc.drive.set_speed_angle(0.15, angle)
        if narrow_train_dist < 125.0:
            cur_state = State.wait_train_2

    elif cur_state == State.wait_train_2:
        angle = rc_utils.clamp(0.03 * (right_dist - left_dist), -1.0, +1.0)
        rc.drive.set_speed_angle(0.0, angle)
        if front_dist < 150.0 and near_left_dist > 225.0:
            cur_state = State.pass_train_2

    elif cur_state == State.pass_train_2:
        rc.drive.set_speed_angle(1.0, 0.0)
        if very_far_left_dist > 100.0 and very_far_right_dist > 100.0 and wide_train_dist > 150.0:
            cur_state = State.approach_train_3

    elif cur_state == State.approach_train_3:
        angle = rc_utils.clamp(0.03 * (very_far_right_dist - very_far_left_dist), -1.0, +1.0)
        rc.drive.set_speed_angle(0.5, angle)
        if narrow_train_dist < 125.0:
            cur_state = State.wait_train_3

    elif cur_state == State.wait_train_3:
        angle = rc_utils.clamp(0.03 * (right_dist - left_dist), -1.0, +1.0)
        rc.drive.set_speed_angle(0.0, angle)
        if front_dist < 150.0 and near_right_dist > 225.0:
            cur_state = State.pass_train_3

    elif cur_state == State.pass_train_3:
        rc.drive.set_speed_angle(1.0, 0.0)
        if very_far_left_dist > 100.0 and very_far_right_dist > 100.0 and wide_train_dist > 150.0:
            cur_state = State.done
 
    elif cur_state == State.done:
        rc.drive.stop()


    # print(cur_state)

    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()