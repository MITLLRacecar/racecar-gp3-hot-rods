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
RED = ((170, 50, 50), (10, 255, 255), "red")
ORANGE = ((0, 50, 50), (20, 255, 255), "orange")
BLUE = ((100, 150, 150), (120, 255, 255), "blue")
GREEN = ((40, 50, 50), (80, 255, 255))
colors = [RED, ORANGE, BLUE]
item = "red"
speed = 0
angle = 0
FRONT_WINDOW = (-5, 5)
contour = None
closest_distance = 0
contour_area = 0
contour_center = (0,0)
CROP_FLOOR = None
MIN_CONTOUR_AREA = 30

class State(IntEnum):
    locate = 0

cur_state: State = State.locate

########################################################################################
# Functions
########################################################################################

def update_contour():
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            pass

        else:
          contour_center = None
          contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)

def start(robot: racecar_core.Racecar):
    global rc, CROP_FLOOR
    rc = robot

    # Have the car begin at a stop
    rc.drive.stop()

    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

    # Print start message
    print(">> Elevator")


def update():
  global speed
  global angle
  global cur_state

  update_contour()

  color_image = rc.camera.get_color_image()
  scan = rc.lidar.get_samples()
  depth_image = rc.camera.get_depth_image()
  markers = rc_utils.get_ar_markers(color_image)

  _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
  rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
  lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)

  if cur_state == State.locate:
    for marker in markers:
      marker.detect_colors(color_image, colors)
      item = marker.get_color()
      corners = marker.get_corners()
      row_center = (int(abs(corners[1][0] + corners[2][0]) / 2), int(abs(corners[0][1] + corners[1][1]) / 2))
      center_distance = rc_utils.get_pixel_average_distance(depth_image, row_center)

      if center_distance < 200:
        if item == "blue":
          speed = 1
          rc.drive.set_speed_angle(speed, angle)
          if forward_dist < 20:
            rc.drive.stop()
        else:
          speed = 0
      else:
        # for now use right, left triggers for line following
        speed = 1
      rc.drive.set_speed_angle(speed, angle)
  pass

# id = 3, ori = 0

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
