"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""

# REMAP ANGLE

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from typing import Any, Tuple, List, Optional
from nptyping import NDArray
sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

isBackingUp = False

########################################################################################
# Functions
########################################################################################
def start():
	# Have the car begin at a stop
	rc.drive.set_max_speed(0.5)
	rc.drive.stop()

	# Print start message
	print(">> Passing Through Trains")

def update():
	"""
	After start() is run, this function is run every frame until the back button
	is pressed
	"""
	global isBackingUp

	further_y_distance = 40

    # Use the triggers to control the car's speed
	rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
	lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
	speed = rt - lt

    # TODO (warmup): Prevent forward movement if the car is about to hit something.
    # Allow the user to override safety stop by holding the right bumper.
	cropped_camera_height = (rc.camera.get_height() // 4) * 2
	top_left_inclusive = (cropped_camera_height, rc.camera.get_width() // 4)
	bottom_right_exclusive = (rc.camera.get_height(), (rc.camera.get_width() // 4) * 3)
	depth_image= rc.camera.get_depth_image()
	depth_image = rc_utils.crop(depth_image, top_left_inclusive, bottom_right_exclusive)
	depth_image = (depth_image - 0.01) % 10000

	# Retrieve the distance of the central pixel
	y, x = rc_utils.get_closest_pixel(depth_image)
	distance = depth_image[y, x]

	# Check if in distance to stop
	ang_vel = rc.physics.get_linear_acceleration()
	print(ang_vel)
	scan = rc.lidar.get_samples()
	lidar_distance = scan[0]
	if lidar_distance < 150 and not rc.controller.is_down(rc.controller.Button.RB):
		isBackingUp = True
		speed = -0.5


	# Use the left joystick to control the angle of the front wheels
	angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

	rc.drive.set_speed_angle(speed, angle)

	# Print the current speed and angle when the A button is held down
	if rc.controller.is_down(rc.controller.Button.A):
		print("Speed:", speed, "Angle:", angle)

	# Print the depth image center distance when the B button is held down
	if rc.controller.is_down(rc.controller.Button.B):
		print("Center distance:", distance)

	# Display the current depth image
	rc.display.show_depth_image(depth_image, points=[(y, x)])

	# TODO (stretch goal): Prevent forward movement if the car is about to drive off a
	# ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
