"""
Title: NewController.py
Author: Kristopher Ward
Group: OUTREACH DEMOS
"""
"""
##############################################################
###   LIBRARIES   ###   LIBRARIES   ###   LIBRARIES   ###   LI
##############################################################
"""
import threading
import cv2
import time
import numpy as np
import serial
import struct

"""
##############################################################
###   GLOBALS/LOCKS   ###   GLOBALS/LOCKS   ###   GLOBALS/LOCK
##############################################################
"""
# filtering data
green_low_hsv = np.array([40, 100, 0])
green_high_hsv = np.array([95, 255, 255])
# flag to indicate a quit
quit_flag = False
# new data flag
new_data = False
# motion status of car
car_is_moving = False

# global variables that determine what will happen to values received
x = -1.
y = -1.
rad = 0.

# steer and throttle variables
STEER = 0
THROTTLE = 1
MAX_DISTANCE = 24
THROTTLE_VAL = 112
THROTTLE_STOP = 100

"""
##############################################################
###   SERIAL SETUP   ###   SERIAL SETUP   ###   SERIAL SETUP  
##############################################################
"""
PORT = "/dev/ttyUSB0"
BAUD = 9600
ArduinoCOM = serial.Serial(port=PORT,
                           baudrate=BAUD)

"""
##############################################################
###   CAMERA SETUP   ###   CAMERA SETUP   ###   CAMERA SETUP  
##############################################################
"""
# initialize camera
camera = cv2.VideoCapture(0)
# let camera warm up
time.sleep(1)

"""
##############################################################
###   FUNCTIONS   ###   FUNCTIONS   ###   FUNCTIONS   ###   FU
##############################################################
"""

def pack(value):
    return struct.pack('>B', value)

"""
##############################################################
###   IMAGE PROCESSING   ###   IMAGE PROCESSING   ###   IMAGE 
##############################################################
"""


def image_processing():
    # forever loop
    while True:
        # grab globals
        global new_data, quit_flag, x, y, rad
        # if new data flag is still high, start loop over
        if new_data:
            continue
        # get new data
        _, frame = camera.read()
        # get hsv frame
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # get mask from hsv frame
        frame_msk = cv2.inRange(frame_hsv, green_low_hsv, green_high_hsv)
        frame_msk = cv2.erode(frame_msk, None, iterations=2)
        frame_msk = cv2.dilate(frame_msk, None, iterations=2)

        # find contours
        contours, _ = cv2.findContours(frame_msk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # the circle can be recognized so we act on it
        if len(contours) > 0:
            # grab biggest contour
            c = max(contours, key=cv2.contourArea)
            ((x_float, y_float), rad_float) = cv2.minEnclosingCircle(c)
            x = int(x_float)
            y = int(y_float)
            rad = int(rad_float)
            print("circle detected\n")
        # we set specific values to force a stop
        else:
            # save values for x y and rad as nothing
            x = -1
            y = -1
            rad = 0

        # set new data flag high
        new_data = True

        # show mask
        cv2.imshow("frame", frame_msk)
        
        # check to see if we are quiting
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            camera.release()
            quit_flag = True
            break




"""
##############################################################
###   MOTION CONTROL   ###   MOTION CONTROL   ###   MOTION CON
##############################################################
"""


def motion_control():
    # camera frame size (used for converting pixel location to physical distance)
    P_xc = 640
    P_yc = 480
    alpha = .543
    beta = .426

    #object radii in inches
    object_radius = 6
    pi = 3.1416

    # create a forever loop
    while True:
        # grab global
        global new_data, car_is_moving
        # check to see if there is new data to act on
        if not new_data:
            continue
        # if so, lower flag
        if new_data:
##            # check to stop
##            if x == -1 or y == -1 or rad == 0:
##                # only stop if car is moving
##                if car_is_moving:
##                    # stop car
##                    ArduinoCOM.write(pack(THROTTLE))
##                    ArduinoCOM.write(pack(THROTTLE_STOP))
##                    # lower flag
##                    car_is_moving = False
##                else:
##                    pass
##            else:
##                print("circle detected")
##                # theta is the horizontal angle of the object from the vertical plane coming out of the car's nose
##                theta = -np.arctan((2*(P_xc/2 - x))/(P_xc*np.tan((pi/2) - alpha)))
##                # phi is the vertical angle of the object from the horizontal plane coming out of the car's nose
##                phi = -np.arctan((2*abs(P_yc/2 - y))/(P_yc*np.tan((pi/2) - beta)))
##                # d is the distance of the object from the car in inches
##                object_distance = -(object_radius*(P_xc/2 - rad)*np.cos(phi))/(rad*np.sin(theta))
##
##                # find the required steering value and communicate for communication with the arduino
##                # 100 corresponds to setting the servo all the way to the right and -100 corresponds
##                # to setting the servo all the way to the left
##                if abs(theta*180/pi) <= 30:
##                    steer_out = (10/3)*(theta*180/pi)
##                elif theta > 0:
##                    steer_out = 100
##                else:
##                    steer_out = -100
##
##                # write steer out to arduino and throttle
##                ArduinoCOM.write(pack(STEER))
##                ArduinoCOM.write(pack(int(steer_out) + 100))
##
##                # write throttle to arduino
##                if object_distance > MAX_DISTANCE:
##                    # check to see if car is already moving
##                    if car_is_moving:
##                        pass
##                    # if not, we move
##                    else:
##                        # set throttle
##                        ArduinoCOM.write(pack(THROTTLE))
##                        ArduinoCOM.write(pack(THROTTLE_VAL))
##                        # set movement status
##                        car_is_moving = True
##                else:
##                    # stop
##                    ArduinoCOM.write(pack(THROTTLE))
##                    ArduinoCOM.write(pack(THROTTLE_STOP))
##                    # update moving status
##                    car_is_moving = False
            # lower new data flag
            new_data = False





"""
##############################################################
###   MAIN   ###   MAIN   ###   MAIN   ###   MAIN   ###   MAIN
##############################################################
"""


def main():
    # start image processing in separate thread
    image_processing_thread = threading.Thread(target=image_processing)
    image_processing_thread.daemon = True
    image_processing_thread.start()
    # start control in separate thread
    motion_control_thread = threading.Thread(target=motion_control)
    motion_control_thread.daemon = True
    motion_control_thread.start()
    # wait until quit
    while not quit_flag:
        pass


if __name__ == "__main__":
    main()
    