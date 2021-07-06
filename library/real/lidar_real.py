"""
Copyright Harvey Mudd College
MIT License
Spring 2020

Contains the Lidar module of the racecar_core library
"""

from lidar import Lidar

# General
import numpy as np
from nptyping import NDArray

# ROS2
import rclpy as ros2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import threading


class LidarReal(Lidar):
    # The ROS topic from which we get Lidar data
    __SCAN_TOPIC = "/scan"

    def __init__(self):
        # ROS node
        self.node = ros2.create_node("scan_sub")

        # subscribe to the scan topic, which will call
        # __scan_callback every time the lidar sends data
        self.__scan_sub = self.node.create_subscription(
            LaserScan, self.__SCAN_TOPIC, self.__scan_callback, qos_profile_sensor_data
        )

        self.__samples = np.empty(0)
        self.__samples_new = np.empty(0)

    def __scan_callback(self, data):
        self.__samples_new = np.array(data.ranges)

    def __update(self):
        self.__samples = self.__samples_new

    def get_samples(self) -> NDArray[720, np.float32]:
        return self.__samples

    def get_samples_async(self) -> NDArray[720, np.float32]:
        return self.__samples_new
