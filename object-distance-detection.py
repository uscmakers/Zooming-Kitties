### IMPORTS ###

from dronekit import connect
import time
import argparse
import sys
import imutils
from imutils import paths
import numpy as np
import cv2
import apriltag.python.apriltag as apriltag

### CONSTANTS ###

connection_string = '/dev/ttyACM0'

KNOWN_WIDTH = 11
KNOWN_HEIGHT = 8.5
FOCAL_LENGTH = 1 # TODO: calibrate

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

### HELPER FUNCS ###

def calculate_dist(p1, p2):
	return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**(0.5)

def distance_to_camera(W, F, P):
	# Compute and return the distance from the marker to the camera
	# Triangle similarity equation: F = (P x D) / W
	return (W * F) / P

# TODO: Make angle helper function

### MAIN ROUTINE ###

def main():

	# vehicle = connect(connection_string, wait_ready=True, baud=115200, timeout=60)
	# print("Successfully connected to vehicle at " + connection_string + "!")
	# vehicle.armed = True
	# time.sleep(1)

	window_title = "Object Distance Detection"
	video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
	if video_capture.isOpened():
		try:
			cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
			# TODO: Get window size (in particular, width) to calculate angle
			while True:
				# Grab the video frame (ret is false if no frames have been grabbed)
				ret, frame = video_capture.read()
    			# Convert frame to grayscale
				image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				at_detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
				# Get list of april tags
				tags = at_detector.detect(image)
       			
				if len(tags) > 0:
					print("detected")
				else:
					print("nothing")
          		
				for tag in tags:
					x = tag.center[0]
					y = tag.center[1]
					w = calculate_dist(tag.corners[0][0], tag.corners[0][1])
					h = calculate_dist(tag.corners[0][0], tag.corners[1][0])
					print(x, y, w, h)
					
				# Use triangle similarity to get distance from camera to marker
				# inches = distance_to_camera(KNOWN_WIDTH, FOCAL_LENGTH, perceived_width)

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
        
	# vehicle.channels.overrides['3'] = 2000
	# vehicle.armed = False
	# vehicle.close()
	print("Closed vehicle")

if __name__ == "__main__":
    main()