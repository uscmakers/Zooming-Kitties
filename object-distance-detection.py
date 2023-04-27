### IMPORTS ###

import math
from dronekit import connect
import time
import argparse
import sys
import imutils
from imutils import paths
import numpy as np
import cv2
import apriltag.python.apriltag as apriltag

### CONFIGURATION ###

connect_to_vehicle = True # enables/disables code dependent on Pixhawk connection (set to True for integrated testing)
show_window = True # enables/disables camera feed window appearing on monitor (set to False for integrated testing)

### CONSTANTS ###

connection_string = '/dev/ttyACM0'

FOCAL_LENGTH = 1320 # pixels
CAPTURE_WIDTH = 1920 # pixels
CAPTURE_HEIGHT = 1080 # pixels
DISPLAY_WIDTH = 960 # pixels
DISPLAY_HEIGHT = 540 # pixels
TAG_WIDTH = 28.57500 # mm
TAG_HEIGHT = 28.57500 # mm
LOWER_THRESH_DIST = 60 # cm
UPPER_THRESH_DIST = 1200 # cm

def gstreamer_pipeline(
    capture_width=CAPTURE_WIDTH,
    capture_height=CAPTURE_HEIGHT,
    display_width=DISPLAY_WIDTH,
    display_height=DISPLAY_HEIGHT,
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

### HELPER FUNCS ###

def calculate_dist(p1, p2):
	return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def distance_to_camera(W, F, P):
	# Compute and return the distance from the marker to the camera
	# Triangle similarity equation: F = (P x D) / W
	return (W * F) / P

def angle_to_marker(window_width, c_x):
    return 2000-(1000/window_width)*c_x

def speed_from_dist(dist):
	speed = -1000/(UPPER_THRESH_DIST-LOWER_THRESH_DIST) * (dist-LOWER_THRESH_DIST) + 2000
	if speed > 2000: speed = 2000
	if speed < 1000: speed = 1000
	return speed

### MAIN ROUTINE ###

def main():
	print("Program started")
	if connect_to_vehicle:
		vehicle = connect(connection_string, wait_ready=True, baud=115200, timeout=60)
		print("Successfully connected to vehicle at " + connection_string + "!")
		vehicle.armed = True
		time.sleep(1)

	window_title = "Object Distance Detection"
	video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
	if video_capture.isOpened():
		try:
			if show_window:
				cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
			while True:
				# Grab the video frame (ret is false if no frames have been grabbed)
				ret, frame = video_capture.read()
				# if not ret:
					# continue
    			# Convert image to grayscale
				image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				at_detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
				# Get list of april tags
				tags = at_detector.detect(image)
       			
				if connect_to_vehicle:
					if len(tags) > 0:
						print("detected")
						tag = tags[0] # Arbitrarily pick first tag
						x = tag.center[0]
						y = tag.center[1]
						w = calculate_dist(tag.corners[0], tag.corners[1])
						h = calculate_dist(tag.corners[0], tag.corners[2])
      					# Use triangle similarity to get distance from camera to marker
						dist_cm = distance_to_camera(TAG_WIDTH, FOCAL_LENGTH, w)/10
						# Get servo value from x pos of marker in window
						servo_motor_val = int(angle_to_marker(DISPLAY_WIDTH, x))
						dc_motor_val = int(speed_from_dist(dist_cm))
						# Motor control
						print(x, y, dist_cm, servo_motor_val, dc_motor_val)
						vehicle.channels.overrides['1'] = servo_motor_val
						# vehicle.channels.overrides['3'] = dc_motor_val
					else:
						print("nothing")
						# Stop vehicle if no marker detected
						# vehicle.channels.overrides['3'] = 1000
    
				if show_window:
        			# Check to see if the user closed the window
					if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
						cv2.imshow(window_title, frame)
					else:
						break
				keyCode = cv2.waitKey(10) & 0xFF
				# Stop the program on the ESC key or 'q'
				if keyCode == 27 or keyCode == ord('q'):
					break
		finally:
			video_capture.release()
			cv2.destroyAllWindows()
	else:
		print("Unable to open camera")
      
	if connect_to_vehicle:  
		vehicle.channels.overrides['3'] = 2000
		vehicle.armed = False
		vehicle.close()
		print("Closed vehicle")

if __name__ == "__main__":
    main()
