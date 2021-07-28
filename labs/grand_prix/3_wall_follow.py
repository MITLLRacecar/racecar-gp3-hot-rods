"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Grand Prix
Line Following / In between lines: Dynamic line color, separate line color for turns
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

rc = racecar_core.create_racecar()

class State():
    green_line_following = 1
    bridge_line_following = 2
    turn_line_following = 3

# Add any global variables here

MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
SECOND_CROP_FLOOR = ((200, 0), (350, rc.camera.get_width()))
CROP_FLOOR = ((350, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Colors, stored as a pair (hsv_min, hsv_max)
GREEN = ((60, 50, 50), (80, 255, 255))
PURPLE = ((90, 120, 120), (120, 255, 255)) 
ORANGE = ((0, 50, 50), (0, 255, 255)) 
color_list = [GREEN, PURPLE, ORANGE]

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cur_state = State.green_line_following

RIGHT_WINDOW = 45
LEFT_WINDOW = 675
FRONT_WINDOW = (-10, 10)

########################################################################################
# Functions
########################################################################################


def start():
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Final Challenge - Grand Prix")

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global green_contour_center
    global green_contour_area
    global purple_contour_center
    global purple_contour_area
    global orange_contour_center
    global orange_contour_area
    global color_list

    image = rc.camera.get_color_image()

    if image is None:
        green_contour_center = None
        green_contour_area = 0
        purple_contour_center = None
        purple_contour_area = 0
        orange_contour_center = None
        orange_contour_area = 0
    else:
        # Search for colored tape contours

        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        for color in color_list:
            # Find all of the green contours
            green_contours = rc_utils.find_contours(image, color[0])

            # Find all of the purple contours
            purple_contours = rc_utils.find_contours(image, color[1])

            # Find all of the orange contours
            orange_contours = rc_utils.find_contours(image, color[2])

            # Select the largest green contour
            green_contour = rc_utils.get_largest_contour(green_contours, MIN_CONTOUR_AREA)

            # Select the largest purple contour
            purple_contour = rc_utils.get_largest_contour(purple_contours, MIN_CONTOUR_AREA)

            # Select the largest orange contour
            orange_contour = rc_utils.get_largest_contour(orange_contours, MIN_CONTOUR_AREA)

            if green_contour is not None:
                # Calculate contour information
                green_contour_center = rc_utils.get_contour_center(green_contour)
                green_contour_area = rc_utils.get_contour_area(green_contour)

                # Draw green_contour onto the image
                rc_utils.draw_contour(image, green_contour)
                rc_utils.draw_circle(image, green_contour_center)

            elif purple_contour is not None:
                # Calculate contour information
                purple_contour_center = rc_utils.get_contour_center(purple_contour)
                purple_contour_area = rc_utils.get_contour_area(purple_contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, purple_contour)
                rc_utils.draw_circle(image, purple_contour_center)

            elif orange_contour is not None:
                # Calculate contour information
                orange_contour_center = rc_utils.get_contour_center(orange_contour)
                orange_contour_area = rc_utils.get_contour_area(orange_contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, orange_contour)
                rc_utils.draw_circle(image, orange_contour_center)
                

            else:
                green_contour_center = None
                green_contour_area = 0
                purple_contour_center = None
                purple_contour_area = 0
                orange_contour_center = None
                orange_contour_area = 0

def update():

    global cur_state
    global color_list
    global green_contour_center
    global green_contour_area
    global purple_contour_center
    global purple_contour_area
    global orange_contour_center
    global orange_contour_area
    global potential_colors

    speed = 0
    angle = 0
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    # line following variable
    n = 50

    
    if cur_state == State.green_line_following:
        update_contour()
        rc.display.show_color_image(color_image)
        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if green_contour_center is not None:
            angle = rc_utils.remap_range(green_contour_center[1], 0, rc.camera.get_width(), -1, 1)

            angle = rc_utils.clamp(angle, -1, 1)
            speed = rc_utils.clamp(speed, 0, 1)
            rc.drive.set_speed_angle(speed, angle)
            rc.display.show_color_image(color_image)

        if markers:
            cur_state = State.bridge_line_following

    if cur_state == State.bridge_line_following:
        for border in markers:
            border.detect_colors(color_image, color_list)

            if border.get_id() == 1:
                update_contour()
                rc.display.show_color_image(color_image)
                # Choose an angle based on contour_center
                # If we could not find a contour, keep the previous angle
                if purple_contour_center is not None:
                    image = rc_utils.crop(image, (rc.camera.get_width() // 2), rc.camera.get_width())
                    angle = rc_utils.remap_range(purple_contour_center[1] - n, 0, rc.camera.get_width(), -1, 1)

                    angle = rc_utils.clamp(angle, -1, 1)
                    speed = rc_utils.clamp(speed, 0, 1)
                    rc.drive.set_speed_angle(speed, angle)
                    rc.display.show_color_image(color_image)

                    if orange_contour_center is not None:
                        angle = rc_utils.remap_range(purple_contour_center[1] - n, 0, rc.camera.get_width(), -1, 1)

                    angle = rc_utils.clamp(angle, -1, 1)
                    speed = rc_utils.clamp(speed, 0, 1)
                    rc.drive.set_speed_angle(speed, angle)
                    rc.display.show_color_image(color_image)


        


    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
