"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
"""

########################################################################################
# Imports
########################################################################################

import sys
from typing import Tuple
import cv2 as cv
import numpy as np
from numpy.testing._private.utils import jiffies

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

class State(IntEnum) :
    approaching = 0 # Go around red cone
    passing = 1 # Go around blue cone
    stopping = 2 # Finish line
    searching = 3 # Manual control until we re-aquire

class Cone(IntEnum) :
    red = 0
    blue = 1

robotState = State.approaching
coneVisible = None
coneApproaching = None

# The HSV ranges (min, max)
RED = ((165, 50, 50), (179, 255, 255))
BLUE = ((95, 150, 150), (120, 255, 255))
ALLCOLOR = ((0,50,50), (130,255,255))
WHITE = ((90, 30, 250), (110, 45, 255))
# FINALS

MAX_SPEED = 1.0
MIN_CONTOUR_AREA = 400

depthImage = None
colorImage = None
waypointCenter = (0,0)
coneCenter = None
speed = 0
angle = 0
counter = 0
coneCounter = 0
finishLine = False
distanceToCone = 0

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

def findCone():
    global colorImage, depthImage, waypointCenter, coneCenter, coneVisible
    
    if colorImage is None and depthImage is None:
        coneCenter = None
    else:
        blueContours = rc_utils.find_contours(colorImage, BLUE[0], BLUE[1])
        redContours = rc_utils.find_contours(colorImage, RED[0], RED[1])  
        blueLargestContour = rc_utils.get_largest_contour(blueContours, MIN_CONTOUR_AREA)      
        redLargestContour = rc_utils.get_largest_contour(redContours, MIN_CONTOUR_AREA)

        if blueLargestContour is None : blueArea = 0
        else : blueArea = rc_utils.get_contour_area(blueLargestContour)
        if redLargestContour is None : redArea = 0
        else: redArea = rc_utils.get_contour_area(redLargestContour)

        if blueArea > redArea:
            closestCone = blueLargestContour
            print("Contour area: " + str(blueArea))
            coneVisible = Cone.blue
        else :
            closestCone = redLargestContour
            print("Contour area: " + str(redArea))
            coneVisible = Cone.red

        coneCenter = rc_utils.get_contour_center(closestCone) # None when redArea, blueArea are 0
        
def calculateWaypoint():
    global colorImage, depthImage, waypointCenter, coneCenter, coneVisible, counter, finishLine, distanceToCone
    if coneCenter is not None : 
        distanceToCone = depthImage[coneCenter[0]][coneCenter[1]]
        tan73 = 3.25 if not finishLine else 3.45 # prev: 3.0
        k = 5000
        x = (k /distanceToCone) * tan73

        pointDirection = 1
        if coneVisible == Cone.blue : pointDirection = -1

        # Clamp to prevent viewport overflow and integer casting overflow
        waypointCenter = (rc_utils.clamp(coneCenter[0], 0, rc.camera.get_height() - 1), rc_utils.clamp(int(rc_utils.clamp(coneCenter[1] + (pointDirection * x), 0, sys.maxsize)), 0, rc.camera.get_width() - 1))
        print("Found " + str(coneVisible)[5:] + " waypoint at: " + str(waypointCenter))
    else : print("Could not find waypoint")

def findFinishLine():
    global finishLine, coneCounter, colorImage, MAX_SPEED
    MIN_CONTOUR_AREA = 1000
    MAX_CONTOUR_AREA = 5000
    HARD_SPEED_MULTIPLIER = 0.3

    if colorImage is None:
        finishLine = False
    else: 
        finishLineContour = rc_utils.get_largest_contour(rc_utils.find_contours(colorImage, WHITE[0], WHITE[1]), MIN_CONTOUR_AREA)
        finishLineCenter = rc_utils.get_contour_center(finishLineContour)

        if finishLineContour is not None and rc_utils.get_contour_area(finishLineContour) < MAX_CONTOUR_AREA:
            # Only detect finish line once
            if not finishLine and coneCounter > 8 :
                finishLine = True
                coneCounter = 0
                print(rc_utils.get_contour_area(finishLineContour))
                print("Found finish line")
                rc_utils.draw_circle(colorImage, finishLineCenter, (0,0,255))
                MAX_SPEED = MAX_SPEED * HARD_SPEED_MULTIPLIER
            elif finishLine and coneCounter > 2 :
                finishLine = False
                MAX_SPEED = MAX_SPEED / HARD_SPEED_MULTIPLIER
        
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Slalom between red and blue cones.  The car should pass to the right of
    # each red cone and the left of each blue cone. 
    global colorImage, depthImage, speed, angle, coneCenter, waypointCenter, robotState
    colorImage = rc.camera.get_color_image()
    depthImage = rc.camera.get_depth_image()
    
    findCone()
    calculateWaypoint()
    findFinishLine()

    if coneCounter == 0 :
        rc.drive.set_max_speed(0.8)
    else : rc.drive.set_max_speed(0.8)

    if robotState == robotState.approaching :
        approachCone()
    if robotState == robotState.passing :
        passCone()
    elif robotState == robotState.stopping :
        stop()
        # TODO: When finish line detected, pass and stop
    elif robotState == robotState.searching:
        search()
        # TODO: Implement autonomous searching

    rc.drive.set_speed_angle(speed, angle)
    
    print("Robot Status: " + str(robotState)[6:])

    if coneCenter is not None : rc_utils.draw_circle(colorImage, coneCenter)
    if waypointCenter is not None : rc_utils.draw_circle(colorImage, waypointCenter)
    rc.display.show_color_image(colorImage) 

def approachCone():
    global speed, angle, counter, coneApproaching, coneVisible, robotState, coneCounter, depthImage, coneCenter
    angle = angleController()
    speed = MAX_SPEED

    if counter == 0 :
        coneApproaching = coneVisible
        counter += rc.get_delta_time()
        
    # If next cone detected, or we lose all cones in viewport
    #TODO if distanceTOCone is a certain value, robotState = State.passing
    if coneApproaching != coneVisible or coneCenter is None :
        robotState = State.passing
        angle = 0
        coneCounter += 1
        counter = 0

def passCone():
    global speed, angle, counter, coneVisible, robotState, coneApproaching, finishLine, distanceToCone, depthImage, coneCenter
    turningSpeed = -0.55 if not finishLine else 0.4 # normal speed : 1.0
    coastingTime = 0.15 if not finishLine else 2.4 # normal speed : 0.35
    maxTurningTime = 2.0 if not finishLine else 5.0 
    turnAngle = 1.0 if not finishLine else 1.0 # normal speed : 0.825
    turnInAngle = 0.15 if not finishLine else 0 # normal speed : 0.175

    counter += rc.get_delta_time()
    # iterations = coastingTime / rc.get_delta_time()
    # increment = 0.25 * turnAngle / iterations

    if counter < coastingTime :
        speed = MAX_SPEED
        # if coneApproaching : angle += increment
        # else : angle -= increment
        angle = turnInAngle if coneApproaching == Cone.blue else -turnInAngle
    elif (counter < maxTurningTime + coastingTime and coneApproaching == coneVisible) or coneCenter is None:
        speed = MAX_SPEED * turningSpeed
        angle = turnAngle if coneApproaching == Cone.blue else -turnAngle
    else :
        if coneApproaching != coneVisible : # and counter < afterTurnTime + coastingTime + maxTurningTime
            robotState = State.approaching
            counter = 0 

def search():
    global speed, angle
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

def stop(): 
    global speed, angle
    speed = 0
    angle = 0 

# def speedController():
#     global depthImage
#     global closest_pixel
#     kP = 0.55
#     braking_distance = 100
#     stopping_distance = 20
#     speed = 0

#     if depthImage is not None : 
#         distance = rc_utils.get_pixel_average_distance(depthImage, closest_pixel, 3)
#         print("Distance: " + str(distance))

#         error = 1/braking_distance * (distance - stopping_distance)
#         speed = error * kP
    
#     return rc_utils.clamp(speed, -0.6, 0.6)

def angleController():
    global waypointCenter
    kP = 2.1 if not finishLine else 3
    angle = 0
    error = waypointCenter[1] - rc.camera.get_width() / 2
    angle =  kP * error / (rc.camera.get_width() / 2)
    return rc_utils.clamp(angle, -1, 1)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
