# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import apriltag
import imutils
import time

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# initialize the list of tracked points
pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=0).start()

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:

	# grab the current frame
	frame = vs.read()

	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break

	# resize the frame, blur it, and convert it to greyscale
	frame = imutils.resize(frame, width=600)
	gscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect apriltag family:
	# define the AprilTags detector options and then detect the AprilTags
	# in the input frame
	options = apriltag.DetectorOptions(families="tag36h11")
	detector = apriltag.Detector(options)
	results = detector.detect(gscale)
	center = None
	
	# only proceed if at least one apriltag was found
	for r in results:

		# draw the center (x, y)-coordinates of the AprilTag
		center = (int(r.center[0]), int(r.center[1]))
		cv2.circle(frame, center, 5, (0, 255, 0), -1)

		# extract the bounding box (x, y)-coordinates for the AprilTag
		# and convert each of the (x, y)-coordinate pairs to integers
		(ptA, ptB, ptC, ptD) = r.corners
		ptB = (int(ptB[0]), int(ptB[1]))
		ptC = (int(ptC[0]), int(ptC[1]))
		ptD = (int(ptD[0]), int(ptC[1]))
		ptA = (int(ptA[0]), int(ptB[1]))

		# draw the bounding box of the AprilTag detection
		cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
		cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
		cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
		cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

		# draw the tag family on the image
		tagFamily = r.tag_family.decode('utf-8')
		cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

		# update the points queue
		pts.appendleft(center)

	# display point in space and general location
	if center is None:
		pass
	else:
		direction = ""
		moveOrStay = ""
		# horizontal location
		if 0 <= center[0] < 100:
			direction = "far left"
		elif 100 <= center[0] < 200:
			direction = "left"
		elif 200 <= center[0] < 400:
			direction = "center"
		elif 400 <= center[0] < 500:
			direction = "right"
		elif 500 <= center[0] <= 600:
			direction = "far right"

		# vertical location
		if 0 <= center[1] < 150:
			moveOrStay = "stay"
		else:
			pass

		print(direction, moveOrStay)

	# loop over the set of tracked points
	for i in range(1, len(pts)):

		# if either of the tracked points are None, ignore them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()