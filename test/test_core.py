"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

A simple program which can be used to manually test racecar_core functionality.
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

max_speed = 0
update_slow_time = 0
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
    global update_slow_time
    global show_triggers
    global show_joysticks

    print("Start function called")
    max_speed = 0.25
    update_slow_time = 0.5
    show_triggers = False
    show_joysticks = False

    rc.set_update_slow_time(update_slow_time)
    rc.drive.set_max_speed(max_speed)
    rc.drive.stop()

    # Print start message
    print(
        ">> Test Core: A testing program for the racecar_core library.\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    Left bumper = decrease max speed\n"
        "    Right bumper = increase max speed\n"
        "    Left joystick click = print trigger values\n"
        "    Right joystick click = print joystick values\n"
        "    A button = Display color image\n"
        "    B button = Display depth image\n"
        "    X button = Display lidar data\n"
        "    Y button = Display IMU data\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global max_speed
    global update_slow_time
    global show_triggers
    global show_joysticks

    # Check if each button was_pressed or was_released
    for button in rc.controller.Button:
        if rc.controller.was_pressed(button):
            print(f"Button [{button.name}] was pressed")
        if rc.controller.was_released(button):
            print(f"Button [{button.name}] was released")

    # Click left and right joystick to toggle showing trigger and joystick values
    left_trigger = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    right_trigger = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    left_joystick = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    right_joystick = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)

    if rc.controller.was_pressed(rc.controller.Button.LJOY):
        show_triggers = not show_triggers

    if rc.controller.was_pressed(rc.controller.Button.RJOY):
        show_joysticks = not show_joysticks

    if show_triggers:
        print(f"Left trigger: [{left_trigger}]; Right trigger: [{right_trigger}]")

    if show_joysticks:
        print(f"Left joystick: [{left_joystick}]; Right joystick: [{right_joystick}]")

    # Use triggers and left joystick to control car (like default drive)
    rc.drive.set_speed_angle(right_trigger - left_trigger, left_joystick[0])

    # Change max speed and update_slow time when the bumper is pressed
    if rc.controller.was_pressed(rc.controller.Button.LB):
        max_speed = max(1 / 16, max_speed / 2)
        rc.drive.set_max_speed(max_speed)
        update_slow_time *= 2
        rc.set_update_slow_time(update_slow_time)
        print(f"max_speed set to [{max_speed}]")
        print(f"update_slow_time set to [{update_slow_time}] seconds")
    if rc.controller.was_pressed(rc.controller.Button.RB):
        max_speed = min(1, max_speed * 2)
        rc.drive.set_max_speed(max_speed)
        update_slow_time /= 2
        rc.set_update_slow_time(update_slow_time)
        print(f"max_speed set to [{max_speed}]")
        print(f"update_slow_time set to [{update_slow_time}] seconds")

    # Capture and display color images when the A button is down
    if rc.controller.is_down(rc.controller.Button.A):
        rc.display.show_color_image(rc.camera.get_color_image())

    # Capture and display depth images when the B button is down
    elif rc.controller.is_down(rc.controller.Button.B):
        depth_image = rc.camera.get_depth_image()
        rc.display.show_depth_image(depth_image)
        depth_center_distance = rc_utils.get_depth_image_center_distance(depth_image)
        print(f"Depth center distance: [{depth_center_distance:.2f}] cm")

    # Capture and display Lidar data when the X button is down
    elif rc.controller.is_down(rc.controller.Button.X):
        lidar = rc.lidar.get_samples()
        rc.display.show_lidar(lidar)
        lidar_forward_distance = rc_utils.get_lidar_average_distance(lidar, 0)
        print(f"LIDAR forward distance: [{lidar_forward_distance:.2f}] cm")

    # Show IMU data when the Y button is pressed
    if rc.controller.is_down(rc.controller.Button.Y):
        a = rc.physics.get_linear_acceleration()
        w = rc.physics.get_angular_velocity()
        print(
            f"Linear acceleration: ({a[0]:5.2f},{a[1]:5.2f},{a[2]:5.2f}); "
            + f"Angular velocity: ({w[0]:5.2f},{w[1]:5.2f},{w[2]:5.2f})"
        )


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Check if each button is_down
    for button in rc.controller.Button:
        if rc.controller.is_down(button):
            print(f"Button [{button.name}] is down")


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
