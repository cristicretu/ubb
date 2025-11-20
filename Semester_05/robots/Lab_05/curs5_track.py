#!/usr/bin/env python3 
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image

ROS_NODE_NAME = "image_processing_node"

def filterColor(img):
    # cv2.inRange(src, lowerb, upperb[, dst]) -> dst
    # the robot has some predefined ranges for some colors
    # accessed with the help of rosparam 
    # currently calibrated for red
    img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(img, np.array([0, 153, 88]), np.array([255, 255, 255]))
    # dilate + erode the mask to remove/reduce the amount of noise
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    return mask

def getMaxContour(mask):
    # cv.findContours(image, mode, method[, contours[, hierarchy[, offset]]]) -> contours, hierarchy
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    max_contour = None
    for c in contours:
        c_area = math.fabs(cv2.contourArea(c))
        if c_area > max_area:
            max_area = c_area
            max_contour = c
    return max_contour, max_area

def drawRectangularBound(img, c):
    x, y, w, h = cv2.boundingRect(c)
    img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), thickness = 2)
    return img, (x + w//2, y + h//2) 
    
def drawCircularBound(img, c):
    center, radius = cv2.minEnclosingCircle(c)
    img = cv2.circle(img, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), thickness = 2)
    return img, (int(center[0]), int(center[1]))

def img_process(img):
    #avoid leaving enabled logs in the image processing method since it will drastically increase execution time and will reduce the framerate
    #rospy.loginfo("image width: %s height: %s" % (img.width, img.height))
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    mask = filterColor(cv2_img)
    max_contour, max_area = getMaxContour(mask)
    # if you want to ignore detected objects smaller than a certain area, you can use something like this
    if max_area > 100:
        cv2_img, center = drawCircularBound(cv2_img, max_contour)
        cv2_img = cv2.circle(cv2_img, center, 5, (0, 0, 255), thickness = 5)

    cv2.imshow("Frame", cv2_img)
    cv2.waitKey(1)

def cleanup():
    rospy.loginfo("Shutting down...")
    cv2.destroyWindow("Frame")

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
