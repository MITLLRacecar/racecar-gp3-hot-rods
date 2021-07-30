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
import cone_slalom as coneSlalom
import line_follow as lineFollow
import wall_follow as wallFollow
import bridge as bridge
import columns as columns
import elevator as elevator
import trains as trains
import slab_slalom as slabSlalom
import ramp_jump as rampJump
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

MARKER_DETECTION_DISTANCE = 100
MIN_STATE_TIMER = 6
# Enables manual robot control override
DEBUG = False
WALL_FOLLOWED_BEFORE = False

# Marker IDs for each segment
class Segment (IntEnum) :
    LineFollow = -1, 
    WallFollow = 0,
    Bridge = 1,
    Columns = 199,
    Elevator = 3,
    ConeSlalom = 4,
    Trains = 5,
    SlabSlalom = 6,
    RampJump = 8

# Changes initial segment
currentSegment: Segment = Segment.LineFollow

timer = 0

# Maps IDs to scripts
SegmentMappings = {
    Segment.LineFollow: lineFollow,
    Segment.WallFollow: wallFollow,
    Segment.Bridge: bridge,
    Segment.Columns : lineFollow,
    Segment.Elevator : elevator,
    Segment.ConeSlalom : lineFollow,
    Segment.Trains : lineFollow,
    Segment.SlabSlalom : lineFollow,
    Segment.RampJump : rampJump
}

########################################################################################
# Functions
########################################################################################

def start():
    global timer
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Final Challenge - Grand Prix")

    timer = 0

    # Start selected segment
    SegmentMappings[currentSegment].start(rc)

def update():
    detectARMarkers()
    if currentSegment not in [Segment.LineFollow, Segment.ConeSlalom, Segment.SlabSlalom] : detectLineFollow()

    # Update selected segment
    SegmentMappings[currentSegment].update()

    if DEBUG : rc.drive.set_speed_angle(
    rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - 
    rc.controller.get_trigger(rc.controller.Trigger.LEFT),
    rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0])

def detectARMarkers() :
    global currentSegment, timer, colorImage, WALL_FOLLOWED_BEFORE

    colorImage = rc.camera.get_color_image()
    depthImage = rc.camera.get_depth_image()
    colorImage = rc_utils.crop(colorImage, (0, 230), (480, 430))
    depthImage = rc_utils.crop(depthImage, (0, 230), (480, 430))
    # rc.display.show_color_image(colorImage)

    markers = rc_utils.get_ar_markers(colorImage)

    if rc.controller.was_pressed((rc.controller.Button.A)) : print("Current Segment: " + str(currentSegment))

    if len(markers) > 0: 
        marker = markers[0]
        id = marker.get_id()

        # Get distance to marker's center
        top_left = marker.get_corners()[0]
        bottom_right = marker.get_corners()[2]
        center = tuple(map(sum, zip(0.5 * top_left, 0.5 * bottom_right))) 
        distance = depthImage[int(center[0])][int(center[1])]

        if id == Segment.Elevator:
            distanceOffset = 350
        elif id == Segment.WallFollow:
            #distanceOffset = -30
            distanceOffset = 0

        else:
            distanceOffset = 0

        # Update current segment
        if currentSegment != id and id in Segment._value2member_map_ and distance < MARKER_DETECTION_DISTANCE + distanceOffset:
            prevSegment = currentSegment

            currentSegment = id

            print(WALL_FOLLOWED_BEFORE)
            if id == Segment.WallFollow:
                if WALL_FOLLOWED_BEFORE == False:
                    currentSegment = id
                    WALL_FOLLOWED_BEFORE = True
                elif WALL_FOLLOWED_BEFORE == True:
                    currentSegment = Segment.LineFollow
                    print("WALL FOLLOW SLOWER")
                    lineFollow.MAX_SPEED = 0.55

            rc.drive.stop()

            # Start selected segment
            timer = 0
            SegmentMappings[currentSegment].start(rc)

            if SegmentMappings[id] == lineFollow:
                if WALL_FOLLOWED_BEFORE == True:
                    print("WALL FOLLOW SLOWER")
                    lineFollow.MAX_SPEED = 0.5
                if id == Segment.ConeSlalom: # Cones
                    print("CONES SLOWER")
                    lineFollow.MAX_SPEED = 0.3
                if id == Segment.SlabSlalom: # Slabs
                    print("SLAB SLOWER")
                    lineFollow.MAX_SPEED = 0.5
                if id == Segment.Columns: # Wall Follow
                    print("WALL FOLLOW SLOWER")
                if id == Segment.Elevator: # Elevator
                    print("ELEVATOR")
                    lineFollow.MAX_SPEED = 0.45
                if id == Segment.RampJump: # RampJump
                    print("RAMPJUMP")
                    lineFollow.MAX_SPEED = 0.5
                    

def detectLineFollow() :
    global timer, currentSegment
    timer += rc.get_delta_time()

    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
    GREEN = ((60, 50, 50), (80, 255, 255))
    MIN_CONTOUR_AREA = 1000
    cropped_image = rc_utils.crop(colorImage, CROP_FLOOR[0], CROP_FLOOR[1])

    contours = rc_utils.find_contours(cropped_image, GREEN[0], GREEN[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    # if contour is not None : print("Contour area: " + str(rc_utils.get_contour_area(contour)))
    # print(timer)

    if timer >= MIN_STATE_TIMER and contour is not None:
        currentSegment = Segment.LineFollow 
        print("Following line")
        SegmentMappings[currentSegment].start(rc)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
 