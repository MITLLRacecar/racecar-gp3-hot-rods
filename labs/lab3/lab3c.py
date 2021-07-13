"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
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

# Add any global variables here
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
depth_image = None
closest_pixel = None

class State(IntEnum):
    moving = 0
    stopped = 1
    
########################################################################################
# Functions
########################################################################################

robot_state = State.moving

def start():
    """
    This function is run once every time the start button is pressed
    """
    
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")
    print("Robot Status: Moving")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Park the car 20 cm away from the closest wall with the car directly facing
    # the wall
    global depth_image
    global closest_pixel
    global robot_state

    depth_image = rc.camera.get_depth_image()
    depth_image = rc_utils.crop(depth_image, (0,0), (rc.camera.get_height() // 2, rc.camera.get_width()))
    closest_pixel = rc_utils.get_closest_pixel(depth_image, 3)
    
    speed = 0
    angle = 0
    if robot_state == State.moving : 
        speed = speed_controller()
        angle = angle_controller()

        if abs(20 - rc_utils.get_pixel_average_distance(depth_image, closest_pixel, 1)) < 0.2 and abs(rc.physics.get_angular_velocity()[2]) < 0.02 : 
            robot_state = State.stopped
            print("Robot Status: Stopped")
    else:
        speed = 0
        angle = 0
        
    rc.drive.set_speed_angle(speed, angle)
    
    if closest_pixel is not None : 
        rc_utils.draw_circle(depth_image, closest_pixel, (255, 0, 0))
    
    rc.display.show_depth_image(depth_image)

def speed_controller() :
    global depth_image
    global closest_pixel
    kP = 0.55
    speed = 0
    braking_distance = 100
    stopping_distance = 20

    if depth_image is not None : 
        distance = rc_utils.get_pixel_average_distance(depth_image, closest_pixel, 3)
        print("Distance: " + str(distance))

        error = 1/braking_distance * (distance - stopping_distance)
        speed = error * kP
    
    return rc_utils.clamp(speed, -0.6, 0.6)
    
def angle_controller():
    global depth_image
    global closest_pixel
    kP = 1.4
    angle = 0
   
    if depth_image is not None : 
        error = closest_pixel[1] - rc.camera.get_width() / 2
        angle = kP * error / (rc.camera.get_width() / 2)
    
    return rc_utils.clamp(angle, -1, 1)
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
