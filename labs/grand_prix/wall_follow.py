"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
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
from enum import Enum

########################################################################################
# Global variables
########################################################################################

# rc = racecar_core.create_racecar()

# Add any global variables here
windows = np.array(())
start_degrees = -90
total_degrees = 180
total_windows = 10
max_speed = 0.5
timer = 0

########################################################################################
# Functions
########################################################################################

def start(robot: racecar_core.Racecar):
    global rc
    """
    This function is run once every time the start button is pressed
    """
    rc = robot
    # Have the car begin at a stop
    rc.drive.stop()
    global max_speed
    rc.drive.set_max_speed(max_speed)

    # Create scanning windows
    global windows
    global total_degrees
    global total_windows
    global start_degrees
    global timer
    timer = 0
    window_size = round(total_degrees / total_windows)
    for i in range(total_windows):
        windows = np.append(windows, i * window_size)
        windows = np.append(windows, (i+1) * window_size - 1)

    windows = windows + start_degrees
    windows = windows.reshape(-1, 2)

    # Print start message
    print(">> Wall Following")

def update():
    # Follow the wall to the right of the car without hitting anything.

    global windows # getting all the angle windows that the car will look at
    global total_degrees
    global total_windows
    global start_degrees
    global timer
    speed = 1

    # First grab lidar data
    scan = rc.lidar.get_samples()
    scan = (scan - 0.001) % 10000

    # Prevent the car from wall following if it has empty space to either side
    for window in ((268, 4), (88, 4)):
        dist = rc_utils.get_lidar_average_distance(scan, window[0], window[1])
        print(dist)
        if dist > 200:
            print("NOT WALL FOLLOWING")
            rc.drive.set_speed_angle(0.5, 0)
            return

    # Get the (!!!average) closest distance for each window using lidar scan data
    windows_distances = np.array(())
    for window in windows:
        _, window_distance = rc_utils.get_lidar_closest_point(scan, window)
        windows_distances = np.append(windows_distances, window_distance)
    
    windows_distances = windows_distances.reshape(-1, 1)

    # Turns to the angle
    angle_index = np.argmax(windows_distances)
    angle_degrees = np.mean(windows[angle_index])
    angle = rc_utils.remap_range(angle_degrees, start_degrees, start_degrees + total_degrees - 1, -1, 1) * 2
    angle = rc_utils.clamp(angle, -1, 1)

    # Manual speed control
    """
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    """

    # If the distance in front of you is very close, SLOW DOWN
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, (-2, 2))
    if forward_dist < 250 * max_speed and abs(speed) == speed:
        multiplier = rc_utils.remap_range(forward_dist, 15 * max_speed, 250 * max_speed, 0 * max_speed, 0.7 * max_speed)
        speed = multiplier * speed

    timer += rc.get_delta_time()
    if timer < 0.2:
        angle = 0
        speed = 0.15

    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
