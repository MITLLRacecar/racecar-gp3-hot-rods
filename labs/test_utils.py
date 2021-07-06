"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

A simple program which can be used to manually test racecar_utils functionality.
"""

########################################################################################
# Imports
########################################################################################

import math
import sys

sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

BLUE = ((90, 100, 100), (120, 255, 255), "blue")
GREEN = ((40, 100, 100), (80, 255, 255), "green")
RED = ((170, 100, 100), (10, 255, 255), "red")
COLORS = [BLUE, GREEN, RED]

max_speed = 0
show_triggers = False
show_joysticks = False

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global max_speed
    global show_triggers
    global show_joysticks

    print("Start function called")
    rc.set_update_slow_time(0.5)
    rc.drive.stop()

    max_speed = 0.25
    show_triggers = False
    show_joysticks = False

    # Test numeric functions
    assert rc_utils.remap_range(5, 0, 10, 0, 50) == 25
    assert rc_utils.remap_range(5, 0, 20, 1000, 900) == 975
    assert rc_utils.remap_range(2, 0, 1, -10, 10) == 30
    assert rc_utils.remap_range(2, 0, 1, -10, 10, True) == 10

    assert rc_utils.clamp(3, 0, 10) == 3
    assert rc_utils.clamp(-2, 0, 10) == 0
    assert rc_utils.clamp(11, 0, 10) == 10

    # Print start message
    print(
        ">> Test Utils: A testing program for the racecar_utils library.\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    Right joystick = measure LIDAR distance in a specific direction when clicked\n"
        "    A button = Take a color image and crop it to the top left\n"
        "    B button = Take a color image and identify the largest red contour\n"
        "    X button = Take a depth image and print several statistics\n"
        "    Y button = Take a lidar scan and print several statistics\n"
        "    Right bumper = Take a color image and identify the AR markers\n"
        "    Left bumper = Throw an exception\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Display the color image cropped to the top left
    if rc.controller.was_pressed(rc.controller.Button.A):
        image = rc.camera.get_color_image()
        cropped = rc_utils.crop(
            image, (0, 0), (rc.camera.get_height() // 2, rc.camera.get_width() // 2)
        )
        rc.display.show_color_image(cropped)

    # Find and display the largest red contour in the color image
    if rc.controller.was_pressed(rc.controller.Button.B):
        image = rc.camera.get_color_image()
        contours = rc_utils.find_contours(image, RED[0], RED[1])
        largest_contour = rc_utils.get_largest_contour(contours)

        if largest_contour is not None:
            center = rc_utils.get_contour_center(largest_contour)
            area = rc_utils.get_contour_area(largest_contour)
            print(f"Largest red contour: center={center}, area={area:.2f}")
            rc_utils.draw_contour(image, largest_contour, rc_utils.ColorBGR.green.value)
            rc_utils.draw_circle(image, center, rc_utils.ColorBGR.yellow.value)
            rc.display.show_color_image(image)
        else:
            print("No red contours found")

    # Print depth image statistics and show the cropped upper half
    if rc.controller.was_pressed(rc.controller.Button.X):
        depth_image = rc.camera.get_depth_image()

        # Measure average distance at several points
        left_distance = rc_utils.get_pixel_average_distance(
            depth_image, (rc.camera.get_height() // 2, rc.camera.get_width() // 4),
        )
        center_distance = rc_utils.get_depth_image_center_distance(depth_image)
        center_distance_raw = rc_utils.get_depth_image_center_distance(depth_image, 1)
        right_distance = rc_utils.get_pixel_average_distance(
            depth_image, (rc.camera.get_height() // 2, 3 * rc.camera.get_width() // 4),
        )
        print(f"Depth image left distance: {left_distance:.2f} cm")
        print(f"Depth image center distance: {center_distance:.2f} cm")
        print(f"Depth image raw center distance: {center_distance_raw:.2f} cm")
        print(f"Depth image right distance: {right_distance:.2f} cm")

        # Measure pixels where the kernel falls off the edge of the photo
        upper_left_distance = rc_utils.get_pixel_average_distance(
            depth_image, (2, 1), 11
        )
        lower_right_distance = rc_utils.get_pixel_average_distance(
            depth_image, (rc.camera.get_height() - 2, rc.camera.get_width() - 5), 13
        )
        print(f"Depth image upper left distance: {upper_left_distance:.2f} cm")
        print(f"Depth image lower right distance: {lower_right_distance:.2f} cm")

        # Find closest point in bottom third
        cropped = rc_utils.crop(
            depth_image,
            (0, 0),
            (rc.camera.get_height() * 2 // 3, rc.camera.get_width()),
        )
        closest_point = rc_utils.get_closest_pixel(cropped)
        closest_distance = cropped[closest_point[0]][closest_point[1]]
        print(
            f"Depth image closest point (upper half): (row={closest_point[0]}, col={closest_point[1]}), distance={closest_distance:.2f} cm"
        )
        rc.display.show_depth_image(cropped, points=[closest_point])

    # Print lidar statistics and show visualization with closest point highlighted
    if rc.controller.was_pressed(rc.controller.Button.Y):
        lidar = rc.lidar.get_samples()
        front_distance = rc_utils.get_lidar_average_distance(lidar, 0)
        right_distance = rc_utils.get_lidar_average_distance(lidar, 90)
        back_distance = rc_utils.get_lidar_average_distance(lidar, 180)
        left_distance = rc_utils.get_lidar_average_distance(lidar, 270)
        print(f"Front LIDAR distance: {front_distance:.2f} cm")
        print(f"Right LIDAR distance: {right_distance:.2f} cm")
        print(f"Back LIDAR distance: {back_distance:.2f} cm")
        print(f"Left LIDAR distance: {left_distance:.2f} cm")

        closest_sample = rc_utils.get_lidar_closest_point(lidar)
        print(
            f"Closest LIDAR point: {closest_sample[0]:.2f} degrees, {closest_sample[1]:.2f} cm"
        )
        rc.display.show_lidar(lidar, highlighted_samples=[closest_sample])

    # Identify AR markers
    if rc.controller.was_pressed(rc.controller.Button.RB):
        image = rc.camera.get_color_image()
        markers = rc_utils.get_ar_markers(image, COLORS)
        for i in range(len(markers)):
            print(f"AR Marker {i}:")
            print(markers[i])
            print("")
        rc_utils.draw_ar_markers(image, markers)
        rc.display.show_color_image(image)

    if rc.controller.was_pressed(rc.controller.Button.LB):
        raise Exception("The left bumper was pressed")

    # Print lidar distance in the direction the right joystick is pointed, if clicked
    if rc.controller.is_down(rc.controller.Button.RJOY):
        rjoy_x, rjoy_y = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
        lidar = rc.lidar.get_samples()
        angle = (math.atan2(rjoy_x, rjoy_y) * 180 / math.pi) % 360
        distance = rc_utils.get_lidar_average_distance(lidar, angle)
        print(f"LIDAR distance at angle {angle:.2f} = {distance:.2f} cm")

    # Default drive-style controls
    left_trigger = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    right_trigger = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    left_joystick = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    rc.drive.set_speed_angle(right_trigger - left_trigger, left_joystick[0])


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
