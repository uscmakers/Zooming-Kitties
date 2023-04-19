### IMPORTS ###

from dronekit import connect
import time
import argparse
import sys
import imutils
from imutils import paths
import numpy as np
import cv2
from pupil_apriltags import Detector
import copy

### CONSTANTS ###

connection_string = '/dev/ttyACM0'

KNOWN_WIDTH = 11
KNOWN_HEIGHT = 8.5
FOCAL_LENGTH = 1 # TODO: calibrate

at_detector = Detector( 
	families="tag36h11",
	nthreads=1,
	quad_decimate=1.0,
	quad_sigma=0.0,
	refine_edges=1,
	decode_sharpening=0.25,
	debug=0
) # April Tag detector

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

def distance_to_camera(W, F, P):
	# Compute and return the distance from the marker to the camera
	# Triangle similarity equation: F = (P x D) / W
	return (W * F) / P

# TODO: Make angle helper function

# COPIED FROM [https://github.com/Kazuhito00/AprilTag-Detection-Python-Sample/blob/main/sample.py]
def draw_tags(image, tags):
    for tag in tags:
        center = tag.center
        corners = tag.corners

        center = (int(center[0]), int(center[1]))
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

        cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

        cv2.line(image, (corner_01[0], corner_01[1]),
                (corner_02[0], corner_02[1]), (255, 0, 0), 2)
        cv2.line(image, (corner_02[0], corner_02[1]),
                (corner_03[0], corner_03[1]), (255, 0, 0), 2)
        cv2.line(image, (corner_03[0], corner_03[1]),
                (corner_04[0], corner_04[1]), (0, 255, 0), 2)
        cv2.line(image, (corner_04[0], corner_04[1]),
                (corner_01[0], corner_01[1]), (0, 255, 0), 2)
    
    return image

### MAIN ROUTINE ###

def main():
	vehicle = connect(connection_string, wait_ready=True, baud=115200, timeout=60)
	print("Successfully connected to vehicle at " + connection_string + "!")
	vehicle.armed = True
	time.sleep(1)

	window_title = "Object Distance Detection"
	video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
	if video_capture.isOpened():
		try:
			cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
			# TODO: Get window size (in particular, width) to calculate angle
			while True:
				# Grab the video frame (ret is false if no frames have been grabbed)
				ret, frame = video_capture.read()
				dbg_image = copy.deepcopy(frame)
    			# Convert frame to grayscale
				image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
       			# Get list of april tags
				tags = at_detector.detect(image)
       			# Draw tags
				dbg_image = draw_tags(dbg_image, tags)
				if(len(tags) > 0):
					print("detected")
				else:
					print("nothing")

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
        
	vehicle.channels.overrides['3'] = 2000
	vehicle.armed = False
	vehicle.close()
	print("Closed vehicle")

if __name__ == "__main__":
    main()