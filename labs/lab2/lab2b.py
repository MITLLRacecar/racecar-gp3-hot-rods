"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2B - Color Image Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 100, 100), (20, 255, 255))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
contour_area_error = 0

min_area = 27900.0 # 29 cm
max_area = 27000.0 # 31 cm
goal_area = 27570.5

area_admissable_error = 40 # 29-31 cm is about 900
area_stopping_range = 15000 # roughly ~ 4 cm 
angle_admissable_error = 0.1 # out of [-1, 1] range

class State(IntEnum):
    search = 0
    approach = 1
    stop = 2

robot_state: State = State.search
########################################################################################
# Functions
########################################################################################


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global robot_state

    robot_state = State.search

    # Initialize variables
    speed = 0
    angle = 0
    #Maximum speed is 1 @ 100 CM or greater, Minimum is -0.5 at 0 CM.

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(">> Lab 2B - Color Image Cone Parking")

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global robot_state
    global contour_area_error
    # Search for contours in the current color image
    update_contour()
    contour_area_error = goal_area - contour_area

    # TODO: Park the car 30 cm away from the closest orange cone

    if robot_state == State.search:
        search()

        if contour_area != 0:
            robot_state = State.approach
            
    elif robot_state == State.approach:
        approach()
    
        if contour_area_error < area_admissable_error and rc.physics.get_linear_acceleration()[2] == 0 and speed < 0.02 : 
            robot_state = State.stop

        if contour_area == 0:
            robot_state = State.search

    elif robot_state == State.stop:
        stop()

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

def search():
    global speed
    global angle
    speed = 1
    angle = 1

def approach():
    global speed
    global angle
    speed = throttle_controller()
    angle = angle_controller()

def stop(): 
    global speed
    global angle
    speed = 0
    angle = 0

def angle_controller():
    kP = 1
    angle = 0
    if contour_center is not None: 
        error = contour_center[1] - rc.camera.get_width() / 2
        angle = kP * 2 * error/rc.camera.get_width()
    return angle

def throttle_controller(): 
    #If within contour range, brake until acceleration = 0. 
    global contour_area_error
    global area_stopping_range
    max_speed = 0.30
    kP = 3
    speed = 0
    
    if contour_area != 0: 
        # if we are farther than the stopping range, apply full power
        if contour_area_error > area_stopping_range : speed = max_speed
        # elif contour_area < goal_area: speed = -.1
        else :
            # else scale our power by our error clamped within the stopping range
            # error as a percentage instead of area
            scale = 1 / area_stopping_range
            speed = kP * contour_area_error * scale

    if speed > max_speed : speed = max_speed
    elif speed < -max_speed : speed = -max_speed
    return speed

def update_slow():
    global speed
    global angle
    print("Robot Status: " + str(robot_state))
    print("Speed: " + str(speed))
    print("Angle: " + str(angle))
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
