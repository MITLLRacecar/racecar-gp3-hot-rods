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
# import wall_follow_dg as wallFollow
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
MIN_STATE_TIMER = 2

# Enables manual robot control override
DEBUG = False

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

# Maps IDs to scripts
SegmentMappings = {
    Segment.LineFollow: lineFollow,
    Segment.WallFollow: wallFollow,
    Segment.Bridge: bridge,
    Segment.Columns : columns,
    Segment.Elevator : elevator,
    Segment.ConeSlalom : coneSlalom,
    Segment.Trains : trains,
    Segment.SlabSlalom : slabSlalom,
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
    tick()

    # Update selected segment
    SegmentMappings[currentSegment].update()

    if DEBUG : rc.drive.set_speed_angle(
    rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - 
    rc.controller.get_trigger(rc.controller.Trigger.LEFT),
    rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0])

def detectARMarkers() :
    global currentSegment, timer, colorImage

    colorImage = rc.camera.get_color_image()
    depthImage = rc.camera.get_depth_image()
    markers = rc_utils.get_ar_markers(colorImage)

    if len(markers) > 0: 
        marker = markers[0]
        id = marker.get_id()

        # Get distance to marker's center
        top_left = marker.get_corners()[0]
        bottom_right = marker.get_corners()[2]
        center = tuple(map(sum, zip(0.5 * top_left, 0.5 * bottom_right))) 
        distance = depthImage[int(center[0])][int(center[1])]
        
        # Update current segment
        if currentSegment != id and id in Segment._value2member_map_ and distance < MARKER_DETECTION_DISTANCE:
            currentSegment = id
            rc.drive.stop()
            
            if rc.controller.was_pressed((rc.controller.Button.A)) : 
                print("Current Segment: " + str(currentSegment))

            # Start selected segment
            SegmentMappings[currentSegment].start(rc)
            timer = 0

def tick() :
    global timer, currentSegment
    timer += rc.get_delta_time()

    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
    GREEN = ((60, 50, 50), (80, 255, 255))
    MIN_CONTOUR_AREA = 40
    cropped_image = rc_utils.crop(colorImage, CROP_FLOOR[0], CROP_FLOOR[1])

    contours = rc_utils.find_contours(cropped_image, GREEN[0], GREEN[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if timer >= MIN_STATE_TIMER and contour is not None:
        currentSegment = Segment.LineFollow 
        SegmentMappings[currentSegment].start(rc)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
 