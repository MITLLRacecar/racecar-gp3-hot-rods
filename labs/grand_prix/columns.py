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
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

#rc = racecar_core.create_racecar()

# Add any global variables here
TARGET_SPEED = 3
speed = 1
angle = 0
FRONT_WINDOW = (-5, 5)
MIN_DISTANCE = 100
ORANGE = ((0, 50, 50), (20, 255, 255))
closest_distance = 0
contour_center = (0,0)
timer = 0

windows_left = np.array(())
start_degrees_left = -90
windows_right = np.array(())
start_degrees_right = 0
total_degrees = 90
total_windows = 10
max_speed = 0.8
timer = 0

class State(IntEnum):
    locate = 0
    turn_right = 1
    turn_left = 2

cur_state: State = State.locate

########################################################################################
# Functions
########################################################################################

def update_contours():
    #takes depth, image, creates mask from colored image, masks depth image accordingly
    global contour_center
    global closest_distance
    
    depth_image = rc.camera.get_depth_image()
    image = rc.camera.get_color_image()
    contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
    contour = rc_utils.get_largest_contour(contours, 30)
    if contour is None:
        return False
    contour_center = rc_utils.get_contour_center(contour)
    rc_utils.draw_circle(image, contour_center)
    rc_utils.draw_contour(image, contour)
    rc.display.show_color_image(image)
    closest_distance = rc_utils.get_pixel_average_distance(depth_image, contour_center)
    
    #checks if 
    if closest_distance > MIN_DISTANCE or closest_distance == 0:
        print(closest_distance)
        return False
    else:
        return True

def start(robot: racecar_core.Racecar):
    global rc
    rc = robot
    # Have the car begin at a stop
    rc.drive.stop()
    global windows_left
    global total_degrees
    global total_windows
    global start_degrees_left
    global windows_right
    global start_degrees_right
    global max_speed

    rc.drive.set_max_speed(max_speed)

    # left side
    window_size_left = round(total_degrees / total_windows)
    for i in range(total_windows):
        windows_left = np.append(windows_left, i * window_size_left)
        windows_left = np.append(windows_left, (i+1) * window_size_left - 1)

    windows_left = windows_left + start_degrees_left
    windows_left = windows_left.reshape(-1, 2)
    print(windows_left)

    # right side
    window_size_right = round(total_degrees / total_windows)
    for i in range(total_windows):
        windows_right = np.append(windows_right, i * window_size_right)
        windows_right = np.append(windows_right, (i+1) * window_size_right - 1)

    windows_right = windows_right + start_degrees_right
    windows_right = windows_right.reshape(-1, 2)
    print(windows_right)

    # Print start message
    print(">> Column Turns")


def update():
    global speed
    global angle
    global cur_state
    global timer

    global windows_left
    global total_degrees
    global total_windows
    global start_degrees_left
    global windows_right
    global start_degrees_right

    color_image = rc.camera.get_color_image()
    scan = rc.lidar.get_samples()
    depth_image = rc.camera.get_depth_image()
    markers = rc_utils.get_ar_markers(color_image)
    rc.drive.set_max_speed(.25)
    speed = 1
    rc.drive.set_speed_angle(speed, angle)

    if cur_state == State.locate:
        print("state locate")
        # get marker and orientation
        for marker in markers:
            marker_orientation = marker.get_orientation()
            orientation_value = marker_orientation.value
            corners = marker.get_corners()
            row_center = (int(abs(corners[1][0] + corners[2][0]) / 2), int(abs(corners[0][1] + corners[1][1]) / 2))
            center_distance = rc_utils.get_pixel_average_distance(depth_image, row_center)
            
            # if center_distance < 200:
            #     print(center_distance)
            if orientation_value == 1:
                cur_state = State.turn_left
            elif orientation_value == 3:
                cur_state = State.turn_right
    
    if cur_state == State.turn_left:
        print("state left")

        # Get the (!!!average) closest distance for each window using lidar scan data
        windows_distances_left = np.array(())
        for window in windows_left:
            _, window_distance_left = rc_utils.get_lidar_closest_point(scan, window)
            windows_distances_left = np.append(windows_distances_left, window_distance_left)
        
        windows_distances_left = windows_distances_left.reshape(-1, 1)

        # Turns to the angle
        angle_index = np.argmax(windows_distances_left)
        angle_degrees = np.mean(windows_left[angle_index])
        angle = rc_utils.remap_range(angle_degrees, start_degrees_left, start_degrees_left + total_degrees - 1, -1, -.25)
        angle = rc_utils.clamp(angle, -1, 0)

        _, forward_dist = rc_utils.get_lidar_closest_point(scan, (-2, 2))
        if forward_dist < 250 * max_speed and abs(speed) == speed:
            multiplier = rc_utils.remap_range(forward_dist, 15 * max_speed, 250 * max_speed, 0 * max_speed, 0.7 * max_speed)
            speed = multiplier * speed
            
        rc.drive.set_speed_angle(speed, angle)

        for marker in markers:
            marker_orientation = marker.get_orientation()
            orientation_value = marker_orientation.value
        
            if orientation_value == 3:
                cur_state = State.turn_right
    
    if cur_state == State.turn_right:
        print("state right")

        # Get the (!!!average) closest distance for each window using lidar scan data
        windows_distances_right = np.array(())
        for window in windows_right:
            _, window_distance_right = rc_utils.get_lidar_closest_point(scan, window)
            windows_distances_right = np.append(windows_distances_right, window_distance_right)
        
        windows_distances_right = windows_distances_right.reshape(-1, 1)

        # Turns to the angle
        angle_index = np.argmax(windows_distances_right)
        angle_degrees = np.mean(windows_right[angle_index])
        angle = rc_utils.remap_range(angle_degrees, start_degrees_right, start_degrees_right + total_degrees - 1, .25, 1)
        angle = rc_utils.clamp(angle, 0, 1)

        _, forward_dist = rc_utils.get_lidar_closest_point(scan, (-2, 2))
        if forward_dist < 250 * max_speed and abs(speed) == speed:
            multiplier = rc_utils.remap_range(forward_dist, 15 * max_speed, 250 * max_speed, 0 * max_speed, 0.7 * max_speed)
            speed = multiplier * speed
        
        rc.drive.set_speed_angle(speed, angle)

        for marker in markers:
            marker_orientation = marker.get_orientation()
            orientation_value = marker_orientation.value

            if orientation_value == 1:
                cur_state = State.turn_left

    # id = 199, left = 1, right = 3
    pass

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
