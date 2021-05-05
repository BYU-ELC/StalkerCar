import serial
import time
import struct
##from pid import PID
from multiprocessing import Process, Value
import cv2 as cv
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import imutils

#function prepares values to be sent to arduino
def pack(value):
    return struct.pack('>B', value)

#computer vision process
def perception(r_px, x_px, y_px):

    #set up the camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(640,480))

    #define color ranges
    pink_low = (88, 0, 106)
    pink_high = (179, 53, 204)

    pink_low_hsv = (147, 96, 159)
    pink_high_hsv = (158, 216, 255)

    yellow_low = (21, 172, 70)
    yellow_high = (44, 255, 255)

    orange_low = (0, 60, 183)
    orange_high = (119, 200, 255)

    orange_low_hsv = (0, 152, 127)
    orange_high_hsv = (17, 255, 255)

    green_low_hsv = (40, 100, 70)
    green_high_hsv = (80, 255, 255)

    black_low_hsv = (0, 100, 0)
    black_high_hsv = (255, 255, 70)

    #set color range
    object_color_low = green_low_hsv
    object_color_high = green_high_hsv

    time.sleep(2)

    #loop grabs current camera frame and processes it
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        #grab the frame and convert to HSV colorspace
        image = frame.array
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        #blur if necessary
        #image = cv.GaussianBlur(image, (11, 11), 0)

        #look for specified color in frame
        mask = cv.inRange(hsv, object_color_low, object_color_high)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=3)

        #find the contours in the mask
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        #if there is at least one contour, we have found the object
        if len(cnts) > 0:
            #grab the biggest contour and find its size and position
            c = max(cnts, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)

            #if the contour size is large enough, then follow it
            if radius > 5:
                #Uncomment this if you are looking at the image feed and want to see the found object
                cv.circle(image, (int(x), int(y)), int(radius), (255, 0, 0), 2)

                #pass values to the control process
                r_px.value = int(radius)
                x_px.value = int(x)
                y_px.value = int(y)
            #if there aren't any contours large enough, send nothing
            else:
                r_px.value = 0
                x_px.value = -1
                y_px.value = -1

        #if there aren't any contours, send nothing
        else:
            r_px.value = 0
            x_px.value = -1
            y_px.value = -1

        #uncomment these if you want to see the output
        cv.imshow("Mask", mask)
        cv.imshow("Image", image)
        key = cv.waitKey(1) & 0xFF

        rawCapture.truncate(0)

#this is a set up loop for PID controllers that isn't being used.
#Might be a good place to start for PID implementation
##def loop_setup():
##    steer_kp = 0.01
##    steer_ki = 0
##    steer_kd = 0.01
##    steer_sp = 0
##    throttle_kp = 1
##    throttle_ki = 0
##    throttle_kd = 1
##    throttle_sp = 24
##    dt = 0.1
##    steer_control = PID(steer_kp, steer_ki, steer_kd, steer_sp, dt)
##    throttle_control = PID(throttle_kp, throttle_ki, throttle_kd, throttle_sp, dt)
##    steer_control.setBounds(-100, 100)
##    throttle_control.setBounds(0, 100)
##    cont = []
##    cont.append(steer_control)
##    cont.append(throttle_control)
##    return cont

#controller process
def control_loop(arduino, r_px, x_px, y_px):
    #camera frame size (used for converting pixel location to physical distance)
    P_xc = 640
    P_yc = 480
    alpha = 0.543
    beta = 0.426

    #object radii in inches
    ball_radius = 1.25
    yellow_ball_r = 4
    green_ball_r = 1.875 
    cone_radius = 4
    circle_radius = 1.5
    object_radius = circle_radius
    pi = 3.1416

    #cont is a bank of pid controllers that isn't being used
##    cont = loop_setup()
    #dt = cont[0].sampleTime

    #sample time
    dt = 0.1

    #manual_override = False
    #wait 5 seconds for things to start up before starting control process
    time.sleep(5)
    run_time = 0
    time_passed = 0
    start_time = time.time()
    last_time = start_time

    #loop forever
    while True:

        #if sample time has passed, run loop
        if time_passed >= dt:
            last_time = time.time()
            time_passed = 0
            #calculate distance/position
            #if the object is not seen, don't do anything
            if x_px.value == -1:
                object_distance = 0
                object_position = 0
                theta = 0
                phi = 0
                steer_out = 0
                throttle_out = 0
            #otherwise, convert pixel location to angle and distance
            else:
                #theta is the horizontal angle of the object from the vertical plane coming straight out of the car's nose
                theta = -np.arctan2((2*(P_xc/2 - x_px.value)), (P_xc*np.tan((pi/2) - alpha)))
                
                #phi is the vertical angle of the object from the horizontal plane coming straight out of the car's nose
                phi = -np.arctan2((2*abs(P_yc/2 - y_px.value)), (P_yc*np.tan((pi/2) - beta)))
                #d is the distance of the object from the car in inches
                d = -(object_radius*(P_xc/2 - x_px.value)*np.cos(phi))/(r_px.value*np.sin(theta))

                object_distance = d
                #relative object pixel location is not used
                object_position = P_xc/2 - x_px.value

                #find the required steering value and communicate for communication with the arduino
                #100 corresponds to setting the servo all the way to the right and -100 corresponds
                #to setting the servo all the way to the left
                if abs(theta*180/pi) <= 30:
                    steer_out = (10/3)*(theta*180/pi)
                elif theta > 0:
                    steer_out = 100
                else:
                    steer_out = -100

                #steer_out = cont[0].compute(object_position)
                #throttle_out = cont[1].compute(object_distance)

            #write the values to the arduino. First write 0 to specify servo, then
            #servo value, then 1 to specify throttle, then throttle value
            arduino.write(pack(0))
            arduino.write(pack(int(steer_out) + 100))
            #if the object is more than two feet away, write 12% to the throttle.
            #otherwise, write 0%. 100 corresponds to 0% and 200 corresponds to 100%.
            #  edit: from my tests it looks like this motor controller only allows for full forward/off/full
            #  reverse with no graduation in between. I might have done something wrong though
            if object_distance > 24:
                arduino.write(pack(1))
                # arduino.write(pack(112))
                arduino.write(pack(100))
            else:
                arduino.write(pack(1))
                arduino.write(pack(100))
            #arduino.write(pack(int(throttle_out) + 100))
            # print("Distance: ", object_distance, " Angle: ", theta*180/pi, " Vertical Angle: ", phi*180/pi)
            print("Distance: %.2f Theta: %.2f Phi: %.2f Radius: %.2f" % (object_distance, theta*180/pi, phi*180/pi, r_px.value))
            run_time = time.time() - start_time

        time_passed = time.time() - last_time



def main():
    #set up serial communication
    arduino = serial.Serial("/dev/ttyUSB0", 9600)
    #wait for link to establish and reset buffers
    time.sleep(2)
    arduino.reset_input_buffer()
    arduino.reset_output_buffer()

    #these objects allowing communication between different threads
    r_px = Value('i', False)
    x_px = Value('i', False)
    y_px = Value('i', False)

    #control process
    p1 = Process(target=control_loop, args=(arduino, r_px, x_px, y_px,))
    #perception process
    p2 = Process(target=perception, args=(r_px, x_px, y_px,))

    #start the processes
    p1.start()
    p2.start()
    p1.join()
    p2.join()

if __name__ == "__main__":
    main()
