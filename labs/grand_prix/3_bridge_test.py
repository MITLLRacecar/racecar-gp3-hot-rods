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
import pprint

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
#PURPLE = ((90, 120, 120), (120, 255, 255)) 
PURPLE = ((120, 140, 0), (150, 255, 255))
ORANGE = ((10, 150, 150), (30, 255, 255))
CROP_FLOOR = ((480 - 250, 0), (480, 640))

# Add any global variables here

########################################################################################
# Functions
########################################################################################

def get_two_largest_contours(contours, min_area: int = 100):
    """
    Finds the largest contour with size greater than min_area.

    Args:
        contours: A list of contours found in an image.
        min_area: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no contour was larger than min_area.
    """
    if len(contours) == 0:
        # TODO: What should we return if the list of contours is empty?
        return None, None
    
    # TODO: Return the largest contour, but return None if no contour is larger than min_area
    contoursAreas = []
    for i in range(len(contours)):
        contoursAreas.append(cv.contourArea(contours[i]))

    maxContourArea = max(contoursAreas)
    final_i = contoursAreas.index(maxContourArea)

    contoursAreas[final_i] = 0
    maxContourArea2 = max(contoursAreas)
    second_i = contoursAreas.index(maxContourArea2)


    if maxContourArea2 > min_area:
        return contours[final_i], contours[second_i] 
    elif maxContourArea > min_area:
        return contours[final_i], None
    else:
        print("No orange or purple contours detected")
        return None, None

def start():
    # Have the car begin at a stop
    rc.drive.stop()
    rc.drive.set_max_speed(0.75)
    # Print start message
    print(">> The Bridge")


def update():
# Find the largest contour
    color_priority = [PURPLE, ORANGE]
    color_priority = [ORANGE, PURPLE]
    image = rc.camera.get_color_image()

    # Get the AR markers BEFORE we crop it
    markers = rc_utils.get_ar_markers(image)
   
    # Crop the image
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    contours = rc_utils.find_contours(image, color_priority[0][0], color_priority[0][1])

    contours_secondary = rc_utils.find_contours(image, color_priority[1][0], color_priority[1][1])

    largest_contour_1, largest_contour_2 = get_two_largest_contours(contours)
    contour_centers = []


    # Draw it on the image
    for contour in [largest_contour_1, largest_contour_2]:
        if type(contour) == np.ndarray:
            contour_centers.append(rc_utils.get_contour_center(contour))
            rc_utils.draw_contour(image, contour, (0,0,0))

    for center in contour_centers:
        rc_utils.draw_circle(image, center, (0,0,0), 6)

    rc.display.show_color_image(image)

    if len(contour_centers) == 0:
    	contour_centers_average_x = rc.camera.get_width() / 2
    elif len(contour_centers) == 1:
    	contour_centers_average_x = rc.camera.get_width() / 2
    elif len(contour_centers) == 2:
    	contour_centers_average_x = (contour_centers[0][1] + contour_centers[1][1]) / 2

    # temp manual controls
    speed = 0

    speed -= rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed += rc.controller.get_trigger(rc.controller.Trigger.RIGHT)

    angle = rc_utils.remap_range(contour_centers_average_x, 0, rc.camera.get_width(), -1, 1) * 1.5
    print(rc_utils.get_largest_contour(contours_secondary))
    if type(rc_utils.get_largest_contour(contours_secondary)) == np.ndarray:
    	if len(contours_secondary) > 0 and rc_utils.get_contour_area(rc_utils.get_largest_contour(contours_secondary)) > 200:
    		speed *= 0.2

    angle = rc_utils.clamp(angle, -1, 1)

    rc.drive.set_speed_angle(speed, angle)
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
