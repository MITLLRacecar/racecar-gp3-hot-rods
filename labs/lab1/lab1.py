"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here

########################################################################################
# Functions
########################################################################################

counter = 0
mode = 0

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()
    rc.drive.set_max_speed(1.0)

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # When the right trigger is pressed, the car should accelerate forward.
    # When the left trigger is pressed, the car should accelerate backward.
    # The front wheels of the car should steer based on the horizontal position of the left joystick.

    global counter
    global mode

    if (mode == 0):
        power = rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        turn = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
        rc.drive.set_speed_angle(power, turn)

    if rc.controller.was_pressed(rc.controller.Button.A) and mode == 0:
        counter = 0
        mode = 1
        print("Driving in a circle...")

    if rc.controller.was_pressed(rc.controller.Button.B) and mode == 0:
        counter = 0  
        mode = 2
        print("Driving in a square...")
    
    if rc.controller.was_pressed(rc.controller.Button.X) and mode == 0:
        counter = 0  
        mode = 3
        print("Driving in a figure eight...")

    if rc.controller.was_pressed(rc.controller.Button.Y) and mode == 0:
        counter = 0  
        mode = 4
        print("Driving in a triangle...")
    
    # circle
    if mode == 1 :
        turn = 6
        if (counter < turn) : rc.drive.set_speed_angle(1,1)
        else : 
            rc.drive.stop()
            mode = 0

    # square
    if mode == 2 :
        straight = 2
        turn = 1.32
        if (counter < straight) : rc.drive.set_speed_angle(1,0)
        elif (counter < straight + turn) : rc.drive.set_speed_angle(1, 1)
        elif (counter < 2 * straight + turn) : rc.drive.set_speed_angle(1, 0)
        elif (counter < 2 * straight + 2 * turn) : rc.drive.set_speed_angle(1, 1)
        elif (counter < 3 * straight + 2 * turn) : rc.drive.set_speed_angle(1, 0)
        elif (counter < 3 * straight + 3 * turn) : rc.drive.set_speed_angle(1, 1)
        elif (counter < 4 * straight + 3 * turn) : rc.drive.set_speed_angle(1, 0)
        elif (counter < 4 * straight + 4 * turn) : rc.drive.set_speed_angle(1, 1)
        else : 
            rc.drive.stop()
            mode = 0

    # figure eight
    if mode == 3 :
        straight = 2
        turn = 3.25
        if (counter < straight) : rc.drive.set_speed_angle(1,0)
        elif (counter < straight + turn) : rc.drive.set_speed_angle(1, 1)
        elif (counter < 2 * straight + turn) : rc.drive.set_speed_angle(1, 0)
        elif (counter < 2 * straight + 2 * turn) : rc.drive.set_speed_angle(1, -1)
        else : 
            rc.drive.stop()
            mode = 0
    
    # triangle
    if mode == 4 : 
        straight = 1
        turn = 1.865
        if (counter < straight) : rc.drive.set_speed_angle(1,0)
        elif (counter < straight + turn) : rc.drive.set_speed_angle(1, 1)
        elif (counter < 2 * straight + turn) : rc.drive.set_speed_angle(1, 0)
        elif (counter < 2 * straight + 2 * turn) : rc.drive.set_speed_angle(1, 1)
        elif (counter < 3 * straight + 2 * turn) : rc.drive.set_speed_angle(1, 0)
        elif (counter < 3 * straight + 3 * turn) : rc.drive.set_speed_angle(1, 1)
        else : 
            rc.drive.stop()
            mode = 0

    counter += rc.get_delta_time()

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
