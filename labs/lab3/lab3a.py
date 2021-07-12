"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

from numpy.core.arrayprint import IntegerFormat
from numpy.core.numerictypes import maximum_sctype
from numpy.lib import utils
from numpy.lib.function_base import _gradient_dispatcher


sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

from enum import IntEnum

class State(IntEnum):
    search = 0
    obstacle = 1
    ramp = 2
    ledge = 3

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here


########################################################################################
# Functions
########################################################################################

obstacle_distance = 65
stopping = False
speed = 0
max_speed = 0.8
angle = 0
robot_state = State.search
ledge_threshold = 350
floor_point = (rc.camera.get_height()  * 15 // 24 , rc.camera.get_width() // 2)
ramp_point = (rc.camera.get_height()  * 11 // 24 , rc.camera.get_width() // 2)
blurred_image = None

def start():
    
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()


    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print the distance at the center of the depth image"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global obstacle_distance
    global stopping
    global angle
    global speed
    global robot_state
    global floor_point
    global ramp_point
    global max_speed
    global blurred_image

    # Calculate the distance of the object directly in front of the car
    depth_image = rc.camera.get_depth_image()
    center_distance = rc_utils.get_depth_image_center_distance(depth_image)

    # TODO (warmup): Prevent forward movement if the car is about to hit something.
    top_left_inclusive = (0, 0)
    bottom_right_exclusive = (rc.camera.get_height() * 16//24, rc.camera.get_width()) #13//24 or 15//24

    cropped_image = rc_utils.crop(depth_image, top_left_inclusive, bottom_right_exclusive)
    blurred_image = cv.GaussianBlur(cropped_image, (1, 1), 0)
    min_distance = blurred_image[rc_utils.get_closest_pixel(blurred_image)[0]][rc_utils.get_closest_pixel(blurred_image)[1]]
    print("min_distance: " + str(min_distance))
    print("floor_point: " + str(blurred_image[floor_point[0]][floor_point[1]]))
    print("ramp_point: " + str(blurred_image[ramp_point[0]][ramp_point[1]]))
    print("isRamp: " + str(isRamp()))
    print(str(robot_state))

    if min_distance < obstacle_distance and min_distance > 0 and isRamp() == False:
        robot_state = State.obstacle # stop
    elif min_distance == 0 and blurred_image[floor_point[0], floor_point[1]] == 0: #isRamp() == True or 
        robot_state = State.ramp # coast
    elif blurred_image[floor_point[0], floor_point[1]] > ledge_threshold :
        robot_state = State.ledge # stop
    else:
        robot_state = State.search # allowing controls

    if robot_state == State.search or rc.controller.is_down(rc.controller.Button.RB) : 
        # Use the triggers to control the car's speed
        rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        speed = rt - lt
        speed = rc_utils.clamp(speed, -max_speed, max_speed)
        
        # Use the left joystick to control the angle of the front wheels
        angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
        rc.drive.set_speed_angle(speed, angle)
        
    elif robot_state != State.ramp :
        if rc.physics.get_angular_velocity()[2] > 0.01:
            rc.drive.set_speed_angle(-1,0)
        else :
            rc.drive.set_speed_angle(0,0)
    else:
        rc.drive.set_speed_angle(max_speed,0)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Display the current depth image
    rc.display.show_depth_image(blurred_image, points=[rc_utils.get_closest_pixel(blurred_image), floor_point, ramp_point])

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.

    # TODO (stretch goal): Tune safety stop so that the car is still able to drive up
    # and down gentle ramps.
    # Hint: You may need to check distance at multiple points.

def isRamp() : 
    global floor_point
    global ramp_point
    global blurred_image
    min_deviation = 15
    max_deviation = ledge_threshold
    floor_val = blurred_image[floor_point[0]][floor_point[1]]
    ramp_val = blurred_image[ramp_point[0]][ramp_point[1]]
    
    if abs(floor_val - ramp_val) > min_deviation and abs(floor_val - ramp_val) < max_deviation and floor_val != 0 and ramp_val != 0 :
        return True
    else : return False

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
