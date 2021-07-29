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

#rc = racecar_core.create_racecar()
BLUE = ((90, 120, 120), (120, 255, 255))  # The HSV range for the color blue
CROP_FLOOR = None

# Add any global variables here

########################################################################################
# Functions
########################################################################################

def find_contours(mask):
    """
    Returns a list of contours around all objects in a mask.
    """
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]

def get_mask(
    image,
    hsv_lower,
    hsv_upper
):
    """
    Returns a mask containing all of the areas of image which were between hsv_lower and hsv_upper.
    
    Args:
        image: The image (stored in BGR) from which to create a mask.
        hsv_lower: The lower bound of HSV values to include in the mask.
        hsv_upper: The upper bound of HSV values to include in the mask.
    """
    # Convert hsv_lower and hsv_upper to numpy arrays so they can be used by OpenCV
    hsv_lower = np.array(hsv_lower)
    hsv_upper = np.array(hsv_upper)
    # TODO: Use the cv.cvtColor function to switch our BGR colors to HSV colors
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    # TODO: Use the cv.inRange function to highlight areas in the correct range
    mask = cv.inRange(image, hsv_lower, hsv_upper, cv.THRESH_BINARY)

    return mask

def get_two_largest_contours(contours, min_area: int = 30):
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
        print("No blue contours detected")
        return None, None

def get_contours(color_image):
    contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

def start(robot: racecar_core.Racecar):
    global rc, CROP_FLOOR
    rc = robot

    CROP_FLOOR = ((rc.camera.get_height() - 250, 0), (rc.camera.get_height(), rc.camera.get_width()))

    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Grand Prix Part 9: The Leap of Faith")


def update():
# Find the largest contour
    image = rc.camera.get_color_image()
    CROP_FLOOR = ((rc.camera.get_height() - 250, 0), (rc.camera.get_height(), rc.camera.get_width()))
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
    BLUE = ((90, 120, 120), (120, 255, 255))
    mask = get_mask(image, BLUE[0], BLUE[1])

    contours = find_contours(mask)
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

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
