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

########################################################################################
# Global variables
########################################################################################

#rc = racecar_core.create_racecar()
BLUE = ((90, 120, 120), (120, 255, 255))  # The HSV range for the color blue
ORANGE = ((0, 100, 150), (40, 255, 255))  # THE HSV threshhold for the color orange
CROP_FLOOR = (
    (480 - 220, 0),
    (480, 640),
)

# Add any global variables here

########################################################################################
# Functions
########################################################################################


def get_two_largest_contours(contours, min_area: int = 30):
    """
    Finds the largest contour with size greater than min_area.

    Args:
        contours: A list of contours found in an image.
        min_area: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no contour was larger than min_area.
    """
    if len(contours) == 0:
        # TODO: What should we return if the list of contours is empty?
        return None, None

    # TODO: Return the largest contour, but return None if no contour is larger than min_area
    contoursAreas = []
    for i in range(len(contours)):
        contoursAreas.append(cv.contourArea(contours[i]))

    maxContourArea = max(contoursAreas)
    final_i = contoursAreas.index(maxContourArea)

    contoursAreas[final_i] = 0
    maxContourArea2 = max(contoursAreas)
    second_i = contoursAreas.index(maxContourArea2)

    if maxContourArea2 > min_area:
        return contours[final_i], contours[second_i]
    elif maxContourArea > min_area:
        return contours[final_i], None
    else:
        print("No blue contours detected")
        return None, None


def start(robot: racecar_core.Racecar):
    global rc, CROP_FLOOR
    rc = robot

    CROP_FLOOR = ((rc.camera.get_height() - 250, 0), (rc.camera.get_height(), rc.camera.get_width()))

    # Have the car begin at a stop
    rc.drive.stop()
    rc.drive.set_max_speed(1)

    # Print start message
    print(">> The Leap of Faith")


def update():
    speed = 0.5
    angle = 0
    contour_centers_average_x = (
        rc.camera.get_width() / 2
    )  # setting a default value just in case

    # Find the largest contour
    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    scan = rc.lidar.get_samples()
    scan = (scan - 0.001) % 10000

    contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
    largest_contour_1, largest_contour_2 = get_two_largest_contours(contours)
    contour_centers = []

    orange_contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
    largest_orange_contour = rc_utils.get_largest_contour(orange_contours)
    if type(largest_orange_contour) == np.ndarray:
        largest_orange_contour_area = rc_utils.get_contour_area(largest_orange_contour)
    else:
        largest_orange_contour_area = 0

    # Draw it on the image
    for contour in [largest_contour_1, largest_contour_2]:
        # print(image.shape)
        if type(contour) == np.ndarray:
            contour_centers.append(rc_utils.get_contour_center(contour))
            rc_utils.draw_contour(image, contour, (0, 0, 0))

    for center in contour_centers:
        rc_utils.draw_circle(image, center, (0, 0, 0), 6)

    rc.display.show_color_image(image)

    # remapping angle using p controller
    if len(contour_centers) == 0:
        angle = 0
    elif len(contour_centers) == 2:
        # check if both contours are on one side
        if contour_centers[0][1] > rc.camera.get_width() * (0.66) and contour_centers[
            1
        ][1] > rc.camera.get_width() * (0.66):
            contour_centers = [contour_centers[0]]
            # otherwise just use a p controller
        else:
            contour_centers_average_x = (contour_centers[0][1] + contour_centers[1][1]) / 2
            angle = rc_utils.remap_range(contour_centers_average_x, 0, rc.camera.get_width(), -1, 1) * 1.2
    elif len(contour_centers) == 1:
        contour_centers_average_x = rc.camera.get_width() - contour_centers[0][1]
        # Check if there is white (a ramp) to the left of the contour center!
        if contour_centers[0][1] > rc.camera.get_width() / 2:
            print("CONTOUR RIGHT")
            side_pixel_check_channels = np.where(np.array(image[contour_centers[0][0], contour_centers[0][1] - 150]) > 110)[0]
        else:
            print("CONTOUR LEFT")
            side_pixel_check_channels = np.where(np.array(image[contour_centers[0][0], contour_centers[0][1] + 150]) > 110)[0]
        if len(side_pixel_check_channels) == 3:
            print("ON RAMP!")
            angle = (
                rc_utils.remap_range(
                    contour_centers_average_x, 0, rc.camera.get_width(), -1, 1
                )
                * 2
            )
        else:
            angle = (
                rc_utils.remap_range(
                    contour_centers_average_x, 0, rc.camera.get_width(), -1, 1
                )
                * -2
            )


    # slow down if in orange section
    if largest_orange_contour_area > 3000:
        angle = 0
        speed = 0.3

    # slow down if in ramp section
    if abs(contour_centers_average_x - rc.camera.get_width()) < 20:
        angle = 0

    if scan[0] > 9000:
        speed = 0.8
        angle = 0

    angle = rc_utils.clamp(angle, -1, 1)

    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

# if __name__ == "__main__":
#     rc.set_start_update(start, update, None)
#     rc.go()
