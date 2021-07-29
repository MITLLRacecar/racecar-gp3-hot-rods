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
from typing import List, Optional
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from camera import NDArray

########################################################################################
# Global variables
########################################################################################

#rc = racecar_core.create_racecar()

ORANGE = ((10, 125, 100), (20, 195, 255))
LIGHT_GRAY = ((95, 20, 140), (115, 40, 160))
DARK_GRAY = ((95, 80, 70), (115, 90, 100))
GRAY = ((95, 20, 70), (115, 90, 160))

# Max distance to detect a slab
MAX_SLAB_DISTANCE = 60
MAX_SPEED = 0.5

########################################################################################
# Functions
########################################################################################


def start(robot: racecar_core.Racecar):
    global rc, waypoint, TOP_LEFT, BOTTOM_RIGHT
    rc = robot

    waypoint = (rc.camera.get_height() // 2, rc.camera.get_width() // 2 )
    TOP_LEFT = (rc.camera.get_height() * 13 // 20, 0)
    BOTTOM_RIGHT = (rc.camera.get_height(), rc.camera.get_width())  

    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Slab Slalom")

def update():
    global color_image, depth_image
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    cropToOrange()

    rc.drive.set_speed_angle(MAX_SPEED, angleController())
    
def cropToOrange():
    global color_image, waypoint
    cropped_image = rc_utils.crop(color_image, TOP_LEFT, BOTTOM_RIGHT)
    contours: List = rc_utils.find_contours(cropped_image, ORANGE[0], ORANGE[1])

    # Filter out slabs that are too far

    # print(len(contours))
    # print(contours[0][0])

    slabs: List = []

    for contour in contours :
        contour_center = rc_utils.get_contour_center(contour)
        if contour_center is not None and depth_image[contour_center[0]][contour_center[1]] < MAX_SLAB_DISTANCE :
            slabs.append(contour)

    # for i in range (len(contours) - 1) :
    #     print(i)
    #     contour_center = rc_utils.get_contour_center(contours[0][i])
    #     if contour_center is not None and depth_image[contour_center[0]][contour_center[1]] > MAX_SLAB_DISTANCE :
    #         contours.pop(i)

    slab = rc_utils.get_largest_contour(slabs)

    if not isinstance(slab, type(None)) : 
        # Get center of orange slab
        br = cv.boundingRect(slab)
        center = (int(br[1] + br[3]/2), int(br[0] + br[2]/2))
        rc_utils.draw_circle(cropped_image, center, (0,0,0))
        rc_utils.draw_contour(cropped_image, slab, (0,0,0))

        print(depth_image[center[0]][center[1]])

        slab_image = rc_utils.crop(cropped_image, (br[1], 0), (br[1] + br[3], rc.camera.get_width() - 1))

        light_gray_contour = rc_utils.get_largest_contour(rc_utils.find_contours(slab_image, GRAY[0], GRAY[1]))
        light_gray_center = rc_utils.get_contour_center(light_gray_contour)
        dark_gray_contour = rc_utils.get_largest_contour(rc_utils.find_contours(slab_image, DARK_GRAY[0], DARK_GRAY[1]))
        dark_gray_center = rc_utils.get_contour_center(dark_gray_contour)

        if light_gray_center is not None : 
            rc_utils.draw_circle(cropped_image, light_gray_center, (255,255,255))
            waypoint = light_gray_center
    else : 
        waypoint = (rc.camera.get_height() // 2, rc.camera.get_width() // 2 )
        
    
    rc.display.show_color_image(cropped_image)

def angleController():
    kP = 1.0
    angle = 0
    error = waypoint[1] - rc.camera.get_width() / 2
    angle =  kP * error / (rc.camera.get_width() / 2)
    return rc_utils.clamp(angle, -1, 1)   

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
