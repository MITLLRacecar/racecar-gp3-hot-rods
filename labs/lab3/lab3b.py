"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3B - Depth Camera Cone Parking
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

from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((0, 50, 50), (30, 255, 255))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
depth_image = None
cone_distance = None

class State(IntEnum):
    search = 0
    approach = 1
    stop = 2

robot_state = State.search
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
    print(">> Lab 3B - Depth Camera Cone Parking")
    print("Robot Status: Searching")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global robot_state
    global depth_image
    global cone_distance
    
    # TODO: Park the car 30 cm away from the closest orange cone.
    # Use both color and depth information to handle cones of multiple sizes.
    # You may wish to copy some of your code from lab2b.py

    depth_image = rc.camera.get_depth_image()
    update_contour()

    if robot_state == State.search:
        search()

        if contour_area != 0:
            print("Found cone!")
            robot_state = State.approach
            
    elif robot_state == State.approach:
        approach()
    
        if cone_distance is not None and abs(30 - cone_distance) < 0.2 and abs(rc.physics.get_angular_velocity()[2]) < 0.02 : 
            print("Robot Status: Stopped")
            robot_state = State.stop

        if contour_area == 0:
            print("Robot Status: Searching")
            robot_state = State.search

    elif robot_state == State.stop:
        stop()
    
    rc.drive.set_speed_angle(speed, angle)
    
def search():
    global speed
    global angle
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

def approach():
    global speed
    global angle
    speed = speed_controller()
    angle = angle_controller()

def stop(): 
    global speed
    global angle
    speed = 0
    angle = 0
    

def speed_controller() :
    global depth_image
    global contour_center
    global cone_distance
    kP = 0.3

    if contour_center is not None : 
        cone_distance = rc_utils.get_pixel_average_distance(depth_image, contour_center, 1)
        print("Distance: " + str(cone_distance))

        error = 1/120 * (cone_distance - 30)
        speed = error * kP
    else : 
        cone_distance = None
    return rc_utils.clamp(speed, -0.6, 0.6)
    
def angle_controller():
    kP = 1
    angle = 0
    if contour_center is not None: 
        error = contour_center[1] - rc.camera.get_width() / 2
        angle = kP * 2 * error/rc.camera.get_width()
    return rc_utils.clamp(angle, -1, 1)

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global depth_image

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        # contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        smallest_distance = 100000
        smallest_contour = None
        if contours is not None:
            for i in contours:
                if i is not None : 
                    contour_center = rc_utils.get_contour_center(i)
                    if contour_center is not None :
                        pixel = rc_utils.get_pixel_average_distance(depth_image, contour_center, 1)
                
                        if smallest_distance > pixel :  
                            smallest_distance = pixel
                            smallest_contour = i
            
            if smallest_contour is not None:
                contour = smallest_contour
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
