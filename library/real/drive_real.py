"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Drive module of the racecar_core library
"""

from drive import Drive

import rclpy as ros2
import numbers
from ackermann_msgs.msg import AckermannDriveStamped

import sys

sys.path.insert(0, "../../library")
import racecar_utils as rc_utils


class DriveReal(Drive):
    # The ROS topic to which we publish drive messages
    __TOPIC = "/drive"

    # PWM constants
    __PWM_TURN_RIGHT = 3000
    __PWM_TURN_LEFT = 9000
    __PWM_SPEED_MIN = 3000
    __PWM_SPEED_MAX = 9000

    def __init__(self):
        # ROS node
        self.__node = ros2.create_node("drive_pub")
        # publish to the drive topic, which will publish a message
        # every time __update is called
        self.__publisher = self.__node.create_publisher(
            AckermannDriveStamped, self.__TOPIC, qos_profile=1
        )
        self.__message = AckermannDriveStamped()
        self.__max_speed = 0.25

    def set_speed_angle(self, speed: float, angle: float) -> None:
        assert (
            -1.0 <= speed <= 1.0
        ), f"speed [{speed}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= angle <= 1.0
        ), f"angle [{angle}] must be between -1.0 and 1.0 inclusive."

        self.__message.drive.speed = rc_utils.remap_range(
            speed * self.__max_speed,
            -1.0,
            1.0,
            self.__PWM_SPEED_MIN,
            self.__PWM_SPEED_MAX,
        )

        angle = -angle
        self.__message.drive.steering_angle = rc_utils.remap_range(
            angle, -1.0, 1.0, self.__PWM_TURN_LEFT, self.__PWM_TURN_RIGHT,
        )

    def set_max_speed(self, max_speed: float = 0.25) -> None:
        assert (
            0.0 <= max_speed <= 1.0
        ), f"max_speed [{max_speed}] must be between 0.0 and 1.0 inclusive."

        self.__max_speed_scale_factor = max_speed

    def __update(self):
        """
        Publishes the current drive message.
        """
        self.__publisher.publish(self.__message)
