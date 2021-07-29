"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
"""

# TODO QUEUE UP A TURN WITH A DOUBLE CROP

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

#rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = None

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 120, 120), (120, 255, 255))  # The HSV range for the color blue
# TODO (challenge 1): add HSV ranges for other colors
GREEN = ((60, 50, 50), (80, 255, 255))
RED = ((0, 50, 50), (0, 255, 255)) # original value was (0, 100, 100)

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
color_list = [GREEN]

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
    global color_list

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # Priority: Red, Green, Blue
        # (currently we only search for blue)
        # Crop the image to the floor directly in front of the car

        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        if rc.controller.was_pressed(rc.controller.Button.Y):
            color_list = []
            print("cleared list")
        if rc.controller.was_pressed(rc.controller.Button.X):
            if BLUE not in color_list:
                color_list.append(BLUE)
            print("added blue")
        elif rc.controller.was_pressed(rc.controller.Button.A):
            if GREEN not in color_list:
                color_list.append(GREEN)
            print("added green")
        elif rc.controller.was_pressed(rc.controller.Button.B):
            if RED not in color_list:
                color_list.append(RED)
            print("added red")

        for color in color_list:
            # Find all of the blue contours
            contours = rc_utils.find_contours(image, color[0], color[1])
            # Select the largest contour
            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is not None:
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)
                break

            else:
                contour_center = None
                contour_area = 0

        # Display the image to the screen
        # rc.display.show_color_image(image)


def start(robot: racecar_core.Racecar):
    global rc
    rc = robot

    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global CROP_FLOOR

    # Initialize variables
    speed = 0
    angle = 0
    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(0.75)

    # Print start message
    print( " Wall follow")


def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
) -> float:
    """
    Remaps a value from one range to another range.

    Args:
        val: A number form the old range to be rescaled.
        old_min: The inclusive 'lower' bound of the old range.
        old_max: The inclusive 'upper' bound of the old range.
        new_min: The inclusive 'lower' bound of the new range.
        new_max: The inclusive 'upper' bound of the new range.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.
    """
    # TODO: remap val to the new range
    '''
    old_range = old_max - old_min
    new_range = new_max - new_min
    return (((val - old_min) * new_range) / old_range) + new_min
    '''
    ratio = (val - old_min) / (old_max - old_min)
    new_range = new_max - new_min
    return (ratio * new_range) + new_min

def update():
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update)#, update_slow)
#     rc.go()
