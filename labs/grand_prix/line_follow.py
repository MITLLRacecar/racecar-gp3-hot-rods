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

# rc = racecar_core.create_racecar()

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
MAX_SPEED = 0.68
id = 0

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
    global CROP_FLOOR

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
        print(image.shape)
        rc.display.show_color_image(image)

def start(robot: racecar_core.Racecar):
    global rc
    rc = robot

    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global CROP_FLOOR
    global id

    # Initialize variables
    speed = 0
    angle = 0
    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    if len(markers) > 0: 
        marker = markers[0]
        id = marker.get_id()
        if id == 4: # Cones
            print("CROP CHANGED")
            CROP_FLOOR = ((465, 50), (rc.camera.get_height(), rc.camera.get_width() - 50))

    print(">> Line Following")

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
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global MAX_SPEED
    global CROP_FLOOR
    global id
    rc.drive.set_max_speed(MAX_SPEED)
    print(MAX_SPEED)

    # Search for contours in the current color image
    update_contour()

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        # Current implementation: bang-bang control (very choppy)
        # TODO (warmup): Implement a smoother way to follow the line
        kP = 2
        # Slow down if cone ar code is detected
        angle = rc_utils.clamp(kP * remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1), -1, 1)

        if id == 4: # Cones
            kP = 2.5
            print("CLAMPING")
            angle = rc_utils.clamp(kP * remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1), -1, 1)

    speed = 1



    # Slow down if something is in front
    depth_image = rc.camera.get_depth_image()
    scan = rc.lidar.get_samples()

    if id == 6:
        scan = rc.lidar.get_samples()
        scan = (scan - 0.01) % 10000
        if scan[0] < 160:
            print("SLOWING DOWN")
            speed = 0.2

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.LB):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.RB):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update)#, update_slow)
#     rc.go()
