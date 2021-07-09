"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (120, 255, 255))  # The HSV range for the color blue
RED = ((0, 50, 50), (20, 255, 255))
GREEN = ((60, 50, 50), (80, 255, 200))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_centers = [0,0,0]  # The (pixel row, pixel column) of contour
contour_areas = [0,0,0]  # The area of contour
contour_center = None
contout_area = 0

########################################################################################
# Functions
########################################################################################


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_centers
    global contour_areas

    image = rc.camera.get_color_image()

    if image is None:
        contour_centers = None
        contour_areas = 0
    else:

        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # Find all of the contours
        contours = [rc_utils.find_contours(image, RED[0], RED[1]), 
         rc_utils.find_contours(image, GREEN[0], GREEN[1]),
         rc_utils.find_contours(image, BLUE[0], BLUE[1])]

        # Select the largest contours
        # TODO priority order should be sorted into one contour beforehand, not afterwards
        contour = [rc_utils.get_largest_contour(contours[0], MIN_CONTOUR_AREA),
         rc_utils.get_largest_contour(contours[1], MIN_CONTOUR_AREA),
         rc_utils.get_largest_contour(contours[2], MIN_CONTOUR_AREA)]

        i = 0
        for x in contour :
            if x is not None:
                # Calculate contour information
                contour_centers[i] = rc_utils.get_contour_center(x)
                contour_areas[i] = rc_utils.get_contour_area(x)

              # Draw contour onto the image
                rc_utils.draw_contour(image, x)
                rc_utils.draw_circle(image, contour_centers[i])

            else :
                contour_centers[i] = None
                contour_areas[i] = 0

            i += 1

        # Display the image to the screen
        rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    A button = print current speed and angle\n"
        "    B button = print contour center and area"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle

    # Search for contours in the current color image
    update_contour()
    #follow contours with priority for red, green, then blue
    follow_contour()

    # Use the triggers to control the car's speed
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    # TODO fix contour_centers telemetry only at index 0
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_centers[0] is None:
            print("No contour found")
        else:
            print("Center:", contour_centers[0], "Area:", contour_areas[0])

def follow_contour():
    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    global angle
    for x in contour_centers :
        if x is not None: 
            # Bang-bang control (very choppy) replaced with a P controller
            kP = 1.0 # or 0.9
            maxAngle = 1
            minAngle = -1
            # scale angle bounds to that of camera
            scale = 1 / (rc.camera.get_width() / 2)

            error = (x[1] - (rc.camera.get_width() / 2)) * scale
            angle = kP * error

             # prevent overflow
            if angle > maxAngle : angle = maxAngle
            elif angle < minAngle: angle = minAngle
            return

def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        # TODO implement other colors here where contour_centers is present
        if contour_centers[0] is None:
            print("-" * 32 + " : area = " + str(contour_areas[0]))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_centers[0][1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_areas[0]))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
