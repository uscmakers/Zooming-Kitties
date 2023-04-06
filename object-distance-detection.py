### IMPORTS ###

from dronekit import connect
import time
from imutils import paths
import numpy as np
import imutils
import cv2

### CONSTANTS ###

connection_string = '/dev/ttyACM0'

KNOWN_WIDTH = 11
KNOWN_HEIGHT = 8.5
FOCAL_LENGTH = 1

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

def find_marker(image):
	# Convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)
	# Find the contours in the edged image and keep the one with maximum area
	# We assume that the object with the largest contour is our marker
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	c = max(cnts, key = cv2.contourArea)
	# Compute the bounding box of the of the marker
	# Returns ( top-left corner(x,y), (width, height), angle of rotation )
	return cv2.minAreaRect(c)

def distance_to_camera(W, F, P):
	# Compute and return the distance from the marker to the camera
	# Triangle similarity equation: F = (P x D) / W
	return (W * F) / P

# TODO: Make angle helper function

### MAIN ROUTINE ###

def main():
	vehicle = connect(connection_string, wait_ready=True, baud=115200, timeout=60)
	print("Successfully connected to vehicle at " + connection_string + "!")
	vehicle.armed = True
	time.sleep(1)

	window_title = "Object Distance Detection"
	face_cascade = cv2.CascadeClassifier(
		"/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
	)
	video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
	if video_capture.isOpened():
		try:
			cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
			# TODO: Get window size (in particular, width) to calculate angle
			while True:
				# Grab the video frame (ret is false if no frames have been grabbed)
				ret, frame = video_capture.read()
				# Get the bounded rectangle of the marker
				marker = find_marker(frame)
				# [[x,y],[w,h],[θ]]
				perceived_width = marker[1][0]
				# Use triangle similarity to get distance from camera to marker
				inches = distance_to_camera(KNOWN_WIDTH, FOCAL_LENGTH, perceived_width)
				print(inches)

				# Display bounded rectangle
				cv2.rectangle(frame, (marker[0][0], marker[0][1]), (marker[0][0] + marker[1][0], marker[0][1] + marker[1][1]), (255, 0, 0), 2)

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