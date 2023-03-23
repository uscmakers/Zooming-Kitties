# MIT License
# Copyright (c) 2019-2022 JetsonHacks
# See LICENSE for OpenCV license and additional information

# https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_face_detection.html
# On the Jetson Nano, OpenCV comes preinstalled
# Data files are in /usr/sharc/OpenCV

import cv2

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1920x1080 @ 30fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen
# Notice that we drop frames if we fall outside the processing time in the appsink element

# MOTOR CONTROL

from dronekit import connect
import time
connection_string = '/dev/ttyACM0'

def gstreamer_pipeline(
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=True"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def face_detect():
    vehicle = connect(connection_string, wait_ready=True, baud=115200, timeout=60)
    print("Successfully connected to vehicle at " + connection_string + "!")
    vehicle.armed = True
    time.sleep(1)
    vehicle.channels.overrides['3'] = 2000
    print("sleeping 5 seconds")
    time.sleep(5)
    vehicle.channels.overrides['3'] = 1000
        
    vehicle.armed = False
    vehicle.close()
    print("Closed vehicle.")

if __name__ == "__main__":
    face_detect()
