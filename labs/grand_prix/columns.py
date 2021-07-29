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
speed = 0
angle = 0
FRONT_WINDOW = (-5, 5)
MIN_DISTANCE = 100
ORANGE = ((0, 50, 50), (20, 255, 255))
closest_distance = 0
contour_center = (0,0)
marker_id = 200
orientation_value = 0

class State(IntEnum):
    locate = 0
    turn_right = 1
    turn_left = 2
    line_follow = 3

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

    # Print start message
    print(">> Final Challenge - Grand Prix")


def update():
  global speed
  global angle
  global cur_state
  global marker_id
  global orientation_value

  color_image = rc.camera.get_color_image()
  scan = rc.lidar.get_samples()
  markers = rc_utils.get_ar_markers(color_image)
  # speed = speedController()
  rc.drive.set_speed_angle(speed, angle)

  if cur_state == State.locate:
    print("state locate")
    # get marker and orientation
    for marker in markers:
      marker_orientation = marker.get_orientation()
      orientation_value = marker_orientation.value
      marker_id = marker.get_id()
    
    if marker_id == 199:
      print("hey")
      pillar = update_contours()
      if orientation_value == 1:
        if not pillar:
          angle = -0.9
          speed = 1
        elif pillar:
          speed = 1
          angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, .2)
          
          if closest_distance < 100:
            timer = 0
            cur_state = State.turn_left
      elif orientation_value == 3:
        if not pillar:
          angle = 0.9
          speed = 1
        elif pillar:
          speed = 1
          angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -0.2, 1)
          
          if closest_distance < 100:
            timer = 0
            cur_state = State.turn_right
    else:
      angle = 0
      rc.drive.set_speed_angle(speed, angle)
  
  if cur_state == State.turn_left:
    print("state left")
    pillar = update_contours()
    speed = 1
    
    if not pillar:
      timer += rc.get_delta_time()
      angle = 0
      
      if timer > .3:
        timer = 0
        cur_state = State.approach_blue
    else:
      if pillar:
        angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, -.4)
  
  if cur_state == State.turn_right:
    print("state right")
    pillar = update_contours()
    speed = 1
    
    if not pillar:
      timer += rc.get_delta_time()
      angle = 0
      
      if timer > .3:
        timer = 0
        cur_state = State.approach_blue
    else:
      if pillar:
        angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), .4, 1)

  
  print("id: ", marker_id)
  print("orientation: ", orientation_value)

  # id = 199, left = 1, right = 3
  pass

# def speedController():
#   kP = 1
#   error = TARGET_SPEED - rc.physics.get_angular_velocity()[2]
#   speed = rc_utils.clamp(error * kP, -2, 2)
#   print(speed)
#   return speed


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
