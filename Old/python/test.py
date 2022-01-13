import cv2
import time

camera = cv2.VideoCapture(0)

time.sleep(1)

_, frame = camera.read()

cv2.imshow("frame", frame)

time.sleep(5)

cv2.destroyAllWindows()

camera.release()

##while True:
##    # get new data
##    _, frame = camera.read()
##    # get hsv frame
##    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
##    # get mask from hsv frame
##    frame_msk = cv2.inRange(frame_hsv, green_low_hsv, green_high_hsv)
##    frame_msk = cv2.erode(frame_msk, None, iterations=2)
##    frame_msk = cv2.dilate(frame_msk, None, iterations=2)
##
##    # find contours
##    contours, _ = cv2.findContours(frame_msk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
##
##    # the circle can be recognized so we act on it
##    if len(contours) > 0:
##        # grab biggest contour
##        c = max(contours, key=cv2.contourArea)
##        ((x_float, y_float), rad_float) = cv2.minEnclosingCircle(c)
##        x = int(x_float)
##        y = int(y_float)
##        rad = int(rad_float)
##        print(x)
##        print(y)
##        print(rad)
##        print("circle detected\n")
##
##    # show mask
##    cv2.imshow("frame", frame_msk)
##    
##    # check to see if we are quiting
##    if cv2.waitKey(1) & 0xFF == ord('q'):
##        cv2.destroyAllWindows()
##        camera.release()
##        break
