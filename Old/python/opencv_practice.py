import cv2 as cv
import imutils
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np

P_xc = 640
P_yc = 480
alpha = .543
beta = .426
ball_radius = 1.25
cone_radius = 4

pi = 3.1416

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))

pink_low = (0, 32, 132)
pink_high = (255, 123, 255)

orange_low = (0, 60, 183)
orange_high = (119, 200, 255)

blue_low = (0, 0, 20)
blue_high = (160, 60, 80)

plate_low = (80, 150, 0)
plate_high = (220, 255, 150)

silver_low = (125, 125, 125)
silver_high = (200, 200, 200)

yellow_low = (0, 150, 150)
yellow_high = (100, 255, 255)

red_low = (0, 0, 180)
red_high = (50, 50, 255)



time.sleep(2)

start = time.time()
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):


    image = frame.array
    # red = image.item(100,100,2)
    #print([image.item(320,240,0),image.item(320,240,1),image.item(320,240,2)])

    #blurred = cv.GaussianBlur(image, (11, 11), 0)
    mask = cv.inRange(image, orange_low, orange_high)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)

    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    if len(cnts) > 0:

        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)
        end = time.time() - start
        if radius > 2:
            theta = np.arctan((2*(P_xc/2 - x))/(P_xc*np.tan((pi/2) - alpha)))
            phi = np.arctan((2*abs(P_yc/2 - y))/(P_yc*np.tan((pi/2) - beta)))
            d = (cone_radius*(P_xc/2 - x)*np.cos(phi))/(radius*np.sin((theta)))
            print("Distance: ", d, " Theta: ", theta*180/pi)
            cv.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        else:
            print('None')
    else:
        end = time.time() - start
        print('None')

    cv.imshow("Mask", mask)
    cv.imshow("Image", image)
    key = cv.waitKey(1) & 0xFF
    start = time.time()
    rawCapture.truncate(0)

cv.destroyAllWindows()
