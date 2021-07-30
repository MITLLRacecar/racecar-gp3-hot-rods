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

LIDAR_WINDOW = (85, 95)
WALL_DISTANCE = 40

# Max distance to detect a slab
MAX_SLAB_DISTANCE = 150
MAX_SPEED = 0.5

lidar_scan = None
orange_detected = False

########################################################################################
# Functions
########################################################################################


def start(robot: racecar_core.Racecar):
    global rc, waypoint, TOP_LEFT, BOTTOM_RIGHT
    rc = robot

    waypoint = (rc.camera.get_height() // 2, rc.camera.get_width() // 2 )
    TOP_LEFT = (rc.camera.get_height() * 11 // 20, 0)
    BOTTOM_RIGHT = (rc.camera.get_height() * 18 // 20, rc.camera.get_width())  

    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Slab Slalom")

def update():
    global lidar_scan
    lidar_scan = rc.lidar.get_samples()

    cropToOrange()

    if nextToWall() and not orange_detected : 
        avoidWall()
    else : 
        rc.drive.set_speed_angle(MAX_SPEED, angleController())

    print("Lidar Left: " + str(rc_utils.get_lidar_closest_point(lidar_scan, (-LIDAR_WINDOW[0], -LIDAR_WINDOW[1]))))

    print(nextToWall())

def nextToWall()-> bool :
    global lidar_scan
    _, right_point = rc_utils.get_lidar_closest_point(lidar_scan, LIDAR_WINDOW)
    _, left_point = rc_utils.get_lidar_closest_point(lidar_scan, (-LIDAR_WINDOW[0], -LIDAR_WINDOW[1]))
    
    if left_point < WALL_DISTANCE or right_point < WALL_DISTANCE : return True
    return False


def avoidWall() :
    global lidar_scan
    _, right_point = rc_utils.get_lidar_closest_point(lidar_scan, LIDAR_WINDOW)
    _, left_point = rc_utils.get_lidar_closest_point(lidar_scan, (-LIDAR_WINDOW[0], -LIDAR_WINDOW[1]))

    kP = 0.8
    scale = 1 / 10
    
    if left_point < WALL_DISTANCE :
        error = WALL_DISTANCE - left_point
    elif right_point < WALL_DISTANCE :
        error = right_point - WALL_DISTANCE

    print(left_point, right_point)

    # print(error * scale * kP)

    rc.drive.set_speed_angle(MAX_SPEED * 0.5, rc_utils.clamp(error * kP * scale, -1, 1))

def cropToOrange():
    global waypoint, orange_detected
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    cropped_image = rc_utils.crop(color_image, TOP_LEFT, BOTTOM_RIGHT)
    cropped_image = color_image
    contours: List = rc_utils.find_contours(cropped_image, ORANGE[0], ORANGE[1])

    # Filter out slabs that are too far

    # print(len(contours))
    # print(contours[0][0])

    slabs: List = []

    for contour in contours :
        contour_center = rc_utils.get_contour_center(contour)
        if contour_center is not None and depth_image[contour_center[0]][contour_center[1]] < MAX_SLAB_DISTANCE :
            rc_utils.draw_circle(cropped_image, contour_center)
            print("Contour: " + str(contour_center))
            print(depth_image[contour_center[0]][contour_center[1]])
            slabs.append(contour)

    slab = rc_utils.get_largest_contour(slabs)

    if not isinstance(slab, type(None)) : 
        orange_detected = True
        # Get center of orange slab
        x, y, width, height = cv.boundingRect(slab)
        center = (int(y + height/2), int(x + width/2))
        rc_utils.draw_circle(cropped_image, center, (0,0,0))
        rc_utils.draw_contour(cropped_image, slab, (0,0,0))

        print("Depth: " + str(depth_image[center[0]][center[1]]))

        slab_image = rc_utils.crop(cropped_image, (y, 0), (y + height, rc.camera.get_width() - 1))

        light_gray_contour = rc_utils.get_largest_contour(rc_utils.find_contours(slab_image, GRAY[0], GRAY[1]))
        light_gray_center = rc_utils.get_contour_center(light_gray_contour)
        dark_gray_contour = rc_utils.get_largest_contour(rc_utils.find_contours(slab_image, DARK_GRAY[0], DARK_GRAY[1]))
        dark_gray_center = rc_utils.get_contour_center(dark_gray_contour)
        
        if light_gray_center is not None : 
            # Offset to plot on normal image size
            waypoint = (light_gray_center[0] + y, light_gray_center[1])
            rc_utils.draw_circle(cropped_image, waypoint, (255,255,255))
            
            # rc.display.show_color_image(slab_image)
    else : 
        orange_detected = False
        waypoint = (rc.camera.get_height() // 2, rc.camera.get_width() // 2)
        
    rc.display.show_color_image(cropped_image)

def angleController():
    kP = 0.8
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
