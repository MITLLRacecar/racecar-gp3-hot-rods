"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

A simple program which tests the racecar_core async functionality (for use in Jupyter).
"""

import sys
import time

sys.path.insert(1, "../library")
import racecar_core

if __name__ == "__main__":
    rc = racecar_core.create_racecar()

    print("Requesting color image...")
    color_image = rc.camera.get_color_image_async()
    print("Color image received")
    rc.display.show_color_image(color_image)
    time.sleep(2)

    print("Requesting depth image...")
    depth_image = rc.camera.get_depth_image_async()
    print("Depth image received")
    rc.display.show_depth_image(depth_image)
    time.sleep(2)

    print("Requesting lidar scan...")
    lidar = rc.lidar.get_samples_async()
    print("LIDAR scan received")
    rc.display.show_lidar(lidar)
    time.sleep(2)

    print("Goodbye!")
