"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Grand Prix
"""

########################################################################################
# Imports
########################################################################################

from enum import Enum
import sys
from typing import List, Optional
import cv2 as cv
import numpy as np
from numpy.lib.function_base import append

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

#rc = racecar_core.create_racecar()

class Turn(Enum) :
    Left = 1,
    Right = 2

ORANGE = ((10, 80, 100), (20, 195, 255))
LIGHT_GRAY = ((95, 20, 140), (115, 40, 160))
DARK_GRAY = ((95, 70, 80), (115, 85, 100))
GRAY = ((95, 20, 70), (115, 90, 160))

LIDAR_WINDOW = (70, 80)
WALL_DISTANCE = 50

# Min distance to detect a slab
SLAB_DETECTION_DISTANCE = 110
MIN_GRAY_CONTOUR_AREA = 4500
MIN_ORANGE_CONTOUR_AREA = 10000
MAX_SPEED = 0.45
TURN_ANGLE = 1.0

lidar_scan = None
orange_detected = False
gray_detected = False
isLeft = False
previousTurn = None
previousTurnLock = False
timer = 0

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
    rc.drive.set_max_speed(0.25)
    rc.drive.stop()

    # Print start message
    print(">> Slab Slalom")

def update():
    global lidar_scan, previousTurnLock, timer
    lidar_scan = rc.lidar.get_samples()

    cropToOrange()
    isStuck()

    timer += rc.get_delta_time()

    if previousTurn == Turn.Left and not gray_detected :
        if closeToWall() or (timer > 0.6 and orange_detected) : turn(TURN_ANGLE)
        else : turn (0)
        previousTurnLock = False
    elif previousTurn == Turn.Right and not gray_detected :
        if closeToWall() or (timer > 0.6 and orange_detected): turn(-TURN_ANGLE)
        elif inCenter() : turn(0)
        else : turn(0)
        previousTurnLock = False
    else : 
        rc.drive.set_speed_angle(MAX_SPEED, angleController())
        setPreviousTurn()
        timer = 0
        # print("SLAB FOLLOW")

    # print("isCloseToWall: " + str(closeToWall()))
    # print("Orange detected: " + str(orange_detected))

def inCenter() :
    _, right_point = rc_utils.get_lidar_closest_point(lidar_scan, (89.5, 90.5))
    _, left_point = rc_utils.get_lidar_closest_point(lidar_scan, (-89.5, -90.5))
    TOLERANCE = 5
    if abs(right_point - left_point) <= TOLERANCE :
        print("CENTER!!!")
        return True
         
    return False

def isStuck()-> bool :
    pass
    # if rc.physics.get_linear_acceleration()[2] < 0.02 and rc.physics.get_linear_acceleration()[2] > -0.02 : 
    #     timer = 0
    #     while (timer < 1) :
    #         timer += rc.get_delta_time()
    #         rc.drive.set_speed_angle(-1,0)
    #     return True
    # return False

def setPreviousTurn() :
    global previousTurn, previousTurnLock

    if non_weighted_waypoint is not None and not previousTurnLock : 
        TURN_REGION = (6 / 10, 4 / 10)
        if non_weighted_waypoint[1] > rc.camera.get_width() * TURN_REGION[0] :
            previousTurnLock = True
            previousTurn = Turn.Right
            print("Current turn: " + str(previousTurn))
        elif non_weighted_waypoint[1] < rc.camera.get_width() * TURN_REGION[1] :
            previousTurnLock = True
            previousTurn = Turn.Left
            print("Current turn: " + str(previousTurn))

# def straighten() :

def closeToWall()-> bool:
    global lidar_scan, isLeft, previousTurnLock
    _, right_point = rc_utils.get_lidar_closest_point(lidar_scan, LIDAR_WINDOW)
    _, left_point = rc_utils.get_lidar_closest_point(lidar_scan, (-LIDAR_WINDOW[0], -LIDAR_WINDOW[1]))

    isLeft = False

    # print("points: " + str(left_point) + " " + str(right_point))
    
    if left_point < right_point and left_point < WALL_DISTANCE: 
        isLeft = True
        return True
    elif right_point < left_point and right_point < WALL_DISTANCE:
        return True
    else :
        return False

def cropToOrange():
    global waypoint, orange_detected, gray_detected, non_weighted_waypoint
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
        # print(depth_image[contour_center[0]][contour_center[1]])
        if contour_center is not None and depth_image[contour_center[0]][contour_center[1]] < SLAB_DETECTION_DISTANCE :
            rc_utils.draw_circle(cropped_image, contour_center)
            # print("Contour: " + str(contour_center))
            slabs.append(contour)

    slab = rc_utils.get_largest_contour(slabs, MIN_ORANGE_CONTOUR_AREA)

    if not isinstance(slab, type(None)) :
        print(rc_utils.get_contour_area(slab)) 
        orange_detected = True
        # Get center of orange slab
        x, y, width, height = cv.boundingRect(slab)
        center = (int(y + height/2), int(x + width/2))
        rc_utils.draw_circle(cropped_image, center, (0,0,0))
        rc_utils.draw_contour(cropped_image, slab, (0,0,0))

        # print("Depth: " + str(depth_image[center[0]][center[1]]))

        extra_top = 0
        extra_bottom = 0
        slab_image = rc_utils.crop(cropped_image, (y - extra_top, 0), (y + height + extra_bottom, rc.camera.get_width() - 1))

        light_gray_contour = rc_utils.get_largest_contour(rc_utils.find_contours(slab_image, LIGHT_GRAY[0], LIGHT_GRAY[1]), MIN_GRAY_CONTOUR_AREA)
        dark_gray_contour = rc_utils.get_largest_contour(rc_utils.find_contours(slab_image, DARK_GRAY[0], DARK_GRAY[1]), MIN_GRAY_CONTOUR_AREA)

        # if light_gray_contour is not None : print("l_area: " + str(rc_utils.get_contour_area(light_gray_contour)))
        # if dark_gray_contour is not None : print("d_area: " + str(rc_utils.get_contour_area(dark_gray_contour)))
        gray_contours: List = []
        gray_centers = None
        gray_center: List = [0,0]
        if light_gray_contour is not None : gray_contours.append(light_gray_contour)
        if dark_gray_contour is not None: gray_contours.append(dark_gray_contour)

        # gray_centers = [rc_utils.get_contour_center(x) for x in gray_contours]
        
        # # Get averaged center between both gray countrs
        # for x in gray_centers :
        #     gray_center[0] += x[0] / len(gray_centers)
        #     gray_center[1] += x[1] / len(gray_centers)

        # print(gray_center)

        # gray_center: tuple = (int(gray_center[0]), int(gray_center[1]))

        gray_contour = rc_utils.get_largest_contour(gray_contours)
        
        if gray_contour is not None : 
            gray_detected = True
            # Offset to plot on normal image size
            gray_center = list(rc_utils.get_contour_center(gray_contour))
            non_weighted_waypoint = (int(gray_center[0] + y), int(gray_center[1]))
            
            WEIGHTING_REGION = (5 / 10, 5 / 10)
            WEIGHT = 5.0
            # print(gray_center)
            # print("min")
            # print(rc.camera.get_width() * WEIGHTING_REGION[0])
            manual_x_offset = 0
            if gray_center[1] > rc.camera.get_width() * WEIGHTING_REGION[0] :
                # gray_center[1] = rc_utils.clamp(WEIGHT * gray_center[1] + manual_x_offset, 0, rc.camera.get_width() - 1)
                # print("RIGHT WEIGHT")
                gray_center[1] = rc.camera.get_width() - 1
            elif gray_center[1] < rc.camera.get_width() * WEIGHTING_REGION[1] :
                # gray_center[1] = rc_utils.clamp(1 / WEIGHT * gray_center[1] - manual_x_offset, 0, rc.camera.get_width() - 1)
                # print("LEFT WEIGHT")
                gray_center[1] = 0

            waypoint = (int(gray_center[0] + y), int(gray_center[1]))

            # print(waypoint)
            rc_utils.draw_circle(cropped_image, waypoint, (255,255,255))
            
            # rc.display.show_color_image(slab_image)
    else : 
        orange_detected = False
        gray_detected = False
        waypoint = (rc.camera.get_height() // 2, rc.camera.get_width() // 2)
        non_weighted_waypoint = None
        
    rc.display.show_color_image(cropped_image)

def turn(angle) :
    # print("TURN")
    rc.drive.set_speed_angle(MAX_SPEED * 1.0, angle)

def angleController():
    kP = 0.8
    error = waypoint[1] - rc.camera.get_width() / 2
    angle =  kP * error / (rc.camera.get_width() / 2)
    return rc_utils.clamp(angle, -1, 1)   

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
