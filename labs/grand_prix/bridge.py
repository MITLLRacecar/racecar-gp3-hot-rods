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

PURPLE = ((120, 140, 0), (150, 255, 255), "purple")
ORANGE = ((10, 150, 150), (30, 255, 255), "orange")
ORANGE_AR = ((10, 100, 60), (30, 255, 255), "orange") # different threshold for AR since it is in the shade
CROP_FLOOR = ((480 - 250, 0), (480, 640))
color_priority = []

# Add any global variables here

## TODO : ONE SIDE FOLLOWING

########################################################################################
# Functions
########################################################################################

def get_two_largest_contours(contours, min_area: int = 100):
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

def start(robot: racecar_core.Racecar):
    global rc
    global color_priority
    rc = robot

    # Detect AR code to determine color
    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    if len(markers) > 0:
        marker = markers[0]
        marker.detect_colors(image, [PURPLE, ORANGE_AR])
        marker_color = marker.get_color()
        print(marker_color)

    if marker_color == "orange":
        color_priority = [PURPLE, ORANGE] # color priority for orange ar code
    elif marker_color == "purple":
        color_priority = [ORANGE, PURPLE]

    # Drive parameters
    rc.drive.stop()
    rc.drive.set_max_speed(0.4)

    # Print start message
    print(">> The Bridge")


def update():
    global color_priority

    # Find the largest contour
    image = rc.camera.get_color_image()
   
    # Crop the image
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    color_index = -1
    for color in color_priority: 
        color_index += 1
        contours = rc_utils.find_contours(image, color[0], color[1])
        if len(contours) != 0:
            if len(contours) != 0 and len(sorted(contours, key=len, reverse=True)[0]) > 100:
                break
        
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

    if color_index == 1: # normal color
        if len(contour_centers) == 0:
            contour_centers_average_x = rc.camera.get_width() / 2
        elif len(contour_centers) == 1:
            contour_centers_average_x = rc.camera.get_width() / 2
        elif len(contour_centers) == 2:
            contour_centers_average_x = (contour_centers[0][1] + contour_centers[1][1]) / 2
    
    elif color_index == 0:
        print("TURN COLOR")
        if len(contour_centers) == 0:
            contour_centers_average_x = rc.camera.get_width() / 2
        elif len(contour_centers) == 1:
            contour_centers_average_x = rc.camera.get_width() - contour_centers[0][1]
        elif len(contour_centers) == 2:
            contour_centers_average_x = contour_centers[1][1]


    # temp manual controls
    """
    speed = 0
    speed -= rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed += rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    """
    speed = 1

    angle = rc_utils.remap_range(contour_centers_average_x, 0, rc.camera.get_width(), -1, 1)
    if color_index == 1:
        angle *= 2
    elif color_index == 0:
        if abs(angle) == angle:
            angle = 1
        else:
            angle = -1
        speed *= 0.1
        print("BIG TURN")


    angle = rc_utils.clamp(angle, -1, 1)

    rc.drive.set_speed_angle(speed, angle)
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#    rc.set_start_update(start, update, None)
#    rc.go()
