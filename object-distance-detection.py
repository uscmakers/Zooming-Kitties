### IMPORTS ###

from dronekit import connect
import time
import argparse
import sys
import imutils
from imutils import paths
import numpy as np
import cv2

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

def distance_to_camera(W, F, P):
	# Compute and return the distance from the marker to the camera
	# Triangle similarity equation: F = (P x D) / W
	return (W * F) / P

# TODO: Make angle helper function

### MAIN ROUTINE ###

def main():
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
	aruco_params = cv2.aruco.DetectorParameters_create()

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
				# Get list of corners of aruco markers in frame
				(corners, ids, rejected) = cv2.aruco.detectMarkers(image=frame, dictionary=aruco_dict, parameters=aruco_params)
				# Check if at least one marker was found
				if len(corners) > 0:
					ids = ids.flatten()
					# TUTORIAL COPIED CODE
					for (markerCorner, markerID) in zip(corners, ids):
						corners = markerCorner.reshape((4, 2))
						(topLeft, topRight, bottomRight, bottomLeft) = corners
						topRight = (int(topRight[0]), int(topRight[1]))
						bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
						bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
						topLeft = (int(topLeft[0]), int(topLeft[1]))
						# draw the bounding box of the ArUCo detection
						cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
						cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
						cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
						cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
						# compute and draw the center (x, y)-coordinates of the
						# ArUco marker
						cX = int((topLeft[0] + bottomRight[0]) / 2.0)
						cY = int((topLeft[1] + bottomRight[1]) / 2.0)
						cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
						# draw the ArUco marker ID on the frame
						cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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