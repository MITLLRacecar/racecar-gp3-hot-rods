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
from numpy.lib.polynomial import roots
from numpy.testing._private.utils import jiffies

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################
class State(IntEnum) :
    approaching = 0 # Go around red cone
    passing = 1 # Go around blue cone
    finishing = 2 # Finish line
    searching = 3 # Manual control until we re-aquire

class Cone(IntEnum) :
    red = 0
    blue = 1

robotState = State.searching
coneVisible = None
coneApproaching = None
canPassCone = False

rc: racecar_core.Racecar = None

# The HSV ranges (min, max)
RED = ((165, 160, 180), (179, 200, 255))
BLUE = ((95, 150, 150), (120, 255, 255))
ALLCOLOR = ((0,50,50), (130,255,255))
WHITE = ((90, 30, 250), (110, 45, 255))
# FINALS

MAX_SPEED = 0.5
MIN_CONTOUR_AREA = 550
MIN_PASSING_DISTANCE = 110
PASSING_ANGLE_RANGE = (60, 75)

depthImage = None
colorImage = None
lidarScan = None
waypointCenter = None # (rc.camera.get_height() / 2, rc.camera.get_width() / 2)
coneCenter = None
speed = 0
angle = 0
counter = 0
coneCounter = 0
finishLine = False
distanceToCone = 0
lidarConePos = (0,0)

########################################################################################
# Functions
########################################################################################

def start(robot: racecar_core.Racecar):
    """
    This function is run once every time the start button is pressed
    """
    global rc, waypointCenter, robotState, coneCounter, counter, coneCenter, coneVisible, coneApproaching, canPassCone, coneCenter
    rc = robot

    waypointCenter = (rc.camera.get_height() / 2, rc.camera.get_width() / 2)

    # Reset incase grand_prix calls us again
    rc.drive.set_max_speed(0.25)
    robotState = State.searching
    coneCounter = 0
    counter = 0
    coneCenter = None
    coneVisible = None
    coneApproaching = None
    canPassCone = False
    coneCenter = None
    
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Cone Slaloming")

def findCone():
    global colorImage, depthImage, waypointCenter, coneCenter, coneVisible, robotState, lidarScan, lidarConePos
    
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

        closestCone = None

        if robotState != State.approaching :
            if blueArea > redArea:
                coneVisible = Cone.blue
            elif redArea > blueArea :
                coneVisible = Cone.red
        
        if coneVisible == Cone.blue : closestCone = blueLargestContour
        elif coneVisible == Cone.red : closestCone = redLargestContour

        # if closestCone is not None : print(rc_utils.get_contour_area(closestCone))

        # print("Cone Visible: " + str(coneVisible))
        # print("Cone Approaching: " + str(coneApproaching))
        
        if coneVisible == None : return
        coneCenter = rc_utils.get_contour_center(closestCone)

        if coneApproaching == Cone.red : window = (-PASSING_ANGLE_RANGE[1], 0)
        else : window = (0, PASSING_ANGLE_RANGE[1])
            
        lidarConePos = rc_utils.get_lidar_closest_point(lidarScan, window)
        # print(rc_utils.get_lidar_average_distance(lidarScan,(lidarConePos[0] - 1) % 360, (lidarConePos[0] + 1) % 360))
        # rc.display.show_lidar(lidarScan, 128, 1000, [lidarConePos])
        
def calculateWaypoint():
    global colorImage, depthImage, waypointCenter, coneCenter, coneVisible, counter, finishLine, distanceToCone
    if coneCenter is not None : 
        distanceToCone = depthImage[coneCenter[0]][coneCenter[1]]
        tan73 = 3.25 if not finishLine else 3.25 # prev: 3.0
        k = 5000
        x = (k /distanceToCone) * tan73

        pointDirection = 1
        if coneVisible == Cone.blue : pointDirection = -1

        # Clamp to prevent viewport overflow and integer casting overflow
        waypointCenter = (rc_utils.clamp(coneCenter[0], 0, rc.camera.get_height() - 1), rc_utils.clamp(int(rc_utils.clamp(coneCenter[1] + (pointDirection * x), 0, sys.maxsize)), 0, rc.camera.get_width() - 1))
        # print("Found " + str(coneVisible)[5:] + " waypoint at: " + str(waypointCenter))
    else :
        # print("Could not find waypoint")
        waypointCenter = None
        
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Slalom between red and blue cones.  The car should pass to the right of
    # each red cone and the left of each blue cone.
    global colorImage, depthImage, speed, angle, coneCenter, waypointCenter, lidarScan, robotState
    colorImage = rc.camera.get_color_image()
    depthImage = rc.camera.get_depth_image()
    lidarScan = rc.lidar.get_samples()

    if robotState == robotState.approaching :
        approachCone()
    if robotState == robotState.passing :
        passCone()
    if robotState == robotState.finishing :
        finish()
    if robotState == robotState.searching:
        search()

    rc.drive.set_speed_angle(speed, angle)

    findCone()
    calculateWaypoint()

    detectMarkers()
    # print(isRobotPastCone(1) and canPassCone)

    if coneCounter == 9 and robotState == State.passing : robotState = State.finishing
    
    print("Robot Status: " + str(robotState)[6:])
    # print(lidarConePos)
    # print(canPassCone)
    # if coneCenter is not None : print(depthImage[coneCenter[0]][coneCenter[1]])

    if coneCenter is not None : rc_utils.draw_circle(colorImage, coneCenter)
    if waypointCenter is not None : rc_utils.draw_circle(colorImage, waypointCenter)
    # rc.display.show_color_image(colorImage) 

def detectMarkers() :
    global robotState, coneCounter
    markers = rc_utils.get_ar_markers(colorImage)
    if  len(markers) > 0: 
        rc_utils.draw_ar_markers(colorImage, markers)
        id = markers[0].get_id()
        # print("Marker detected: " + str(id))
        if id == 4 and coneCounter != 0 :
            coneCounter = 0
            robotState = State.searching

def approachCone():
    global speed, angle, coneVisible, robotState, coneCounter, coneCenter, counter, canPassCone
    if waypointCenter is not None : angle = angleController()
    speed = speedController()

    counter = 0
    if coneCenter is not None and depthImage[coneCenter[0]][coneCenter[1]] < 75 : 
        canPassCone = True
    if canPassCone and isRobotPastCone(1) :
        robotState = State.passing
        canPassCone = False
        coneCounter += 1
        print("Cones Passed: " + str(coneCounter))

def passCone():
    global speed, angle, coneVisible, robotState, finishLine, distanceToCone, depthImage, coneCenter, coneApproaching, counter, canPassCone
    turnAngle =  1.0 if not finishLine else 0.65

    counter = 0

    if coneApproaching == coneVisible : 
        speed = speedController() * 0.9
        angle = turnAngle if coneApproaching == Cone.blue else -turnAngle
    elif not isRobotPastCone(1) and coneApproaching != coneVisible :
        robotState = State.approaching
        coneApproaching = coneVisible

def isRobotPastCone(iterations) -> bool :
    global counter
    if lidarConePos[1] < MIN_PASSING_DISTANCE and (lidarConePos[0] >= PASSING_ANGLE_RANGE[0] and lidarConePos[0] <= PASSING_ANGLE_RANGE[1] or lidarConePos[0] <= 360 - PASSING_ANGLE_RANGE[0] and lidarConePos[0] >= 360 - PASSING_ANGLE_RANGE[1]) :
        counter += 1
    if counter >= iterations : return True
    return False

def search():
    global speed, angle, robotState, coneApproaching
    speed = MAX_SPEED
    angle = 0

    if coneVisible != None : 
        robotState = State.approaching
        coneApproaching = coneVisible

def finish(): 
    global speed, angle
    speed = 0.8
    angle = 0.3

def speedController():
    kP = 1.5
    constantSpeed = 0.05
    targetAcceleration = 0.5
    scale = 2.5
    error = targetAcceleration - rc.physics.get_linear_acceleration()[2] / scale

    speed = rc_utils.clamp(kP * error + constantSpeed, constantSpeed, 1)

    return speed

def angleController():
    global waypointCenter
    kP = 1.0 if not finishLine else 3
    angle = 0
    error = waypointCenter[1] - rc.camera.get_width() / 2
    angle =  kP * error / (rc.camera.get_width() / 2)
    return rc_utils.clamp(angle, -1, 1)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
