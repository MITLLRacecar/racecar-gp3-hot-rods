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
    purple_bridge_line_following = 2
    orange_turn_line_following = 3
    orange_bridge_line_following = 4
    purple_turn_line_following = 5

# Add any global variables here

MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
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
    global left_purple_contour_center
    global left_purple_contour_area
    global left_orange_contour_center
    global left_orange_contour_area
    global right_purple_contour_center
    global right_purple_contour_area
    global right_orange_contour_center
    global right_orange_contour_area
    global color_list

    image = rc.camera.get_color_image()

    if image is None:
        green_contour_center = None
        green_contour_area = 0
        left_purple_contour_center = None
        left_purple_contour_area = 0
        left_orange_contour_center = None
        left_orange_contour_area = 0
        right_purple_contour_center = None
        right_purple_contour_area = 0
        right_orange_contour_center = None
        right_orange_contour_area = 0
    else:
        # Search for colored tape contours

        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        left_image = rc_utils.crop(image, (0, 0), (rc.camera.get_height(), rc.camera.get_width() // 2))
        right_image = rc_utils.crop(image, (0, rc.camera.get_width() // 2), (rc.camera.get_height(), rc.camera.get_width()))

        for color in color_list:
            # Find all of the green contours
            green_contours = rc_utils.find_contours(image, color[0])

            # Find all of the purple contours
            left_purple_contours = rc_utils.find_contours(left_image, color[1])
            right_purple_contours = rc_utils.find_contours(right_image, color[1])

            # Find all of the orange contours
            left_orange_contours = rc_utils.find_contours(left_image, color[2])
            right_orange_contours = rc_utils.find_contours(right_image, color[2])

            # Select the largest green contour
            green_contour = rc_utils.get_largest_contour(green_contours, MIN_CONTOUR_AREA)

            # Select the largest purple contour
            left_purple_contour = rc_utils.get_largest_contour(left_purple_contours, MIN_CONTOUR_AREA)
            right_purple_contour = rc_utils.get_largest_contour(right_purple_contours, MIN_CONTOUR_AREA)

            # Select the largest orange contour
            left_orange_contour = rc_utils.get_largest_contour(left_orange_contours, MIN_CONTOUR_AREA)
            right_orange_contour = rc_utils.get_largest_contour(right_orange_contours, MIN_CONTOUR_AREA)


            if green_contour is not None:
                # Calculate contour information
                green_contour_center = rc_utils.get_contour_center(green_contour)
                green_contour_area = rc_utils.get_contour_area(green_contour)

                # Draw green_contour onto the image
                rc_utils.draw_contour(image, green_contour)
                rc_utils.draw_circle(image, green_contour_center)

            if left_purple_contour is not None:
                # Calculate contour information
                left_purple_contour_center = rc_utils.get_contour_center(left_purple_contour)
                left_purple_contour_area = rc_utils.get_contour_area(left_purple_contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, left_purple_contour)
                rc_utils.draw_circle(image, left_purple_contour_center)

            if right_purple_contour is not None:
                # Calculate contour information
                right_purple_contour_center = rc_utils.get_contour_center(right_purple_contour)
                right_purple_contour_area = rc_utils.get_contour_area(right_purple_contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, right_purple_contour)
                rc_utils.draw_circle(image, right_purple_contour_center)

            
            if left_orange_contour is not None:
                # Calculate contour information
                left_orange_contour_center = rc_utils.get_contour_center(left_orange_contour)
                left_orange_contour_area = rc_utils.get_contour_area(left_orange_contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, left_orange_contour)
                rc_utils.draw_circle(image, left_orange_contour_center)

            if right_orange_contour is not None:
                # Calculate contour information
                right_orange_contour_center = rc_utils.get_contour_center(right_orange_contour)
                right_orange_contour_area = rc_utils.get_contour_area(right_orange_contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, right_orange_contour)
                rc_utils.draw_circle(image, right_orange_contour_center)
                

            else:
                green_contour_center = None
                green_contour_area = 0
                left_purple_contour_center = None
                left_purple_contour_area = 0
                left_orange_contour_center = None
                left_orange_contour_area = 0
                right_purple_contour_center = None
                right_purple_contour_area = 0
                right_orange_contour_center = None
                right_orange_contour_area = 0

def update():

    global cur_state
    global color_list
    global green_contour_center
    global green_contour_area
    global left_purple_contour_center
    global left_purple_contour_area
    global left_orange_contour_center
    global left_orange_contour_area
    global right_purple_contour_center
    global right_purple_contour_area
    global right_orange_contour_center
    global right_orange_contour_area
    global potential_colors

    speed = 1
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)

    
    if cur_state == State.green_line_following:
        update_contour()
        rc.display.show_color_image(color_image)
        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if green_contour_center is not None:
            angle = rc_utils.remap_range(green_contour_center[1], 0, rc.camera.get_width(), -1, 1)
            angle = rc_utils.clamp(angle, -1, 1)
            speed = 1
            rc.drive.set_speed_angle(speed, angle)
            rc.display.show_color_image(color_image)

        if markers:
            for border in markers:
                border.detect_colors(color_image, color_list)
                if border.get_id() == 1:
                    cur_state = State.purple_bridge_line_following
                if border.get_id() == 2:
                    cur_state = State.orange_bridge_line_following

    if cur_state == State.purple_bridge_line_following:
        update_contour()
        rc.display.show_color_image(color_image)
        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if left_purple_contour_center is not None and right_purple_contour_center is not None:
            line_difference = right_purple_contour_center[1] - left_purple_contour_center [1]
            angle = rc_utils.remap_range(left_purple_contour_center[1] + line_difference, 0, rc.camera.get_width(), -1, 1)
                    
            angle = rc_utils.clamp(angle, -1, 1)
            speed = 1
            rc.drive.set_speed_angle(speed, angle)
            rc.display.show_color_image(color_image)

        elif left_purple_contour_center is not None and right_purple_contour_center is None:
            angle = rc_utils.remap_range(left_purple_contour_center[1] + 50, 0, rc.camera.get_width(), -1, 1)
                    
            angle = rc_utils.clamp(angle, -1, 1)
            speed = 1
            rc.drive.set_speed_angle(speed, angle)
            rc.display.show_color_image(color_image)

        elif left_purple_contour_center is None and right_purple_contour_center is not None:
            angle = rc_utils.remap_range(right_purple_contour_center[1] - 50, 0, rc.camera.get_width(), -1, 1)
                    
            angle = rc_utils.clamp(angle, -1, 1)
            speed = 1
            rc.drive.set_speed_angle(speed, angle)
            rc.display.show_color_image(color_image)

        if left_orange_contour_center is not None and right_orange_contour_center is not None:
            cur_state = State.orange_turn_line_following

    if cur_state == State.purple_bridge_line_following:       
            if border.get_id() == 2:
                update_contour()
                rc.display.show_color_image(color_image)
                # Choose an angle based on contour_center
                # If we could not find a contour, keep the previous angle
                if orange_contour_center is not None:
                    image = rc_utils.crop(image, (rc.camera.get_width() // 2), rc.camera.get_width())
                    angle = rc_utils.remap_range(orange_contour_center[1] - n, 0, rc.camera.get_width(), -1, 1)

                    angle = rc_utils.clamp(angle, -1, 1)
                    speed = rc_utils.clamp(speed, 0, 1)
                    rc.drive.set_speed_angle(speed, angle)
                    rc.display.show_color_image(color_image)

                    if purple_contour_center is not None:
                        angle = rc_utils.remap_range(purple_contour_center[1] - n, 0, rc.camera.get_width(), -1, 1)

                    angle = rc_utils.clamp(angle, -1, 1)
                    speed = rc_utils.clamp(speed, 0, 0.5)
                    rc.drive.set_speed_angle(speed, angle)
                    rc.display.show_color_image(color_image)

                    if orange_contour_center is None and purple_contour_center is None:
                        if green_contour_center is not None:
                            cur_state = State.green_line_following

    if cur_state == State.turn_line_following:
        angle = rc_utils.remap_range(purple_contour_center[1] - n, 0, rc.camera.get_width(), -1, 1)

                    angle = rc_utils.clamp(angle, -1, 1)
                    speed = rc_utils.clamp(speed, 0, 0.5)
                    rc.drive.set_speed_angle(speed, angle)
                    rc.display.show_color_image(color_image)

                    if purple_contour_center is None and orange_contour_center is None:
                        if green_contour_center is not None:
                            cur_state = State.green_line_following

            
pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()