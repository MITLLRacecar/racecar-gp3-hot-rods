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

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
#PURPLE = ((90, 120, 120), (120, 255, 255)) 
PURPLE = ((130, 140, 0), (140, 255, 255))
ORANGE = ((10, 50, 150), (30, 255, 255))
CROP_FLOOR = ((rc.camera.get_height() - 170, 0), (rc.camera.get_height(), rc.camera.get_width()))

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

    # Print start message
    print(">> Grand Prix Part 9: The Leap of Faith")


def update():
# Find the largest contour
    image = rc.camera.get_color_image()

    # Get the AR markers BEFORE we crop it
    markers = rc_utils.get_ar_markers(image)
   
    # Crop the image
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    orange_contours = np.array(rc_utils.find_contours(image, ORANGE[0], ORANGE[1]), dtype=object)
    purple_contours = np.array(rc_utils.find_contours(image, PURPLE[0], PURPLE[1]), dtype=object)

    print(orange_contours.shape, purple_contours.shape)

    try:
        contours = np.concatenate((orange_contours, purple_contours))
    except:
        print("FAILED")
        print("ORANGE\n\n", orange_contours.shape, orange_contours, "PURPLE\n\n", purple_contours.shape, purple_contours)

    largest_contour_1, largest_contour_2 = get_two_largest_contours(contours)
    contour_centers = []


    # Draw it on the image
    for contour in [largest_contour_1, largest_contour_2]:
        #print(image.shape)
        if type(contour) == np.ndarray:
            contour_centers.append(rc_utils.get_contour_center(contour))
            rc_utils.draw_contour(image, contour, (0,0,0))

    for center in contour_centers:
        rc_utils.draw_circle(image, center, (0,0,0), 6)

    rc.display.show_color_image(image)

    
    
    
    
    # temp manual controls
    manual_speed = 0
    manual_angle = 0

    manual_speed -= rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    manual_speed += rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    manual_angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    rc.drive.set_speed_angle(manual_speed, manual_angle)
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
