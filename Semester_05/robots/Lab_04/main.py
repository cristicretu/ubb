import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor.msg import Led, RGB
import math
import numpy as np

ROS_NODE_NAME = "color_detection_node"
led_pub = None
my_pub = None
bridge = CvBridge()

def filterByColor(img):
    img_to_cv = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(img, np.array([0, 153, 88]), np.array([255, 255, 255]))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    return mask

def maxCountour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    max_contour = None
    for c in contours:
        c_area = math.fabs(cv2.contourArea(c))
        if c_area > max_area:
            max_area = c_area
            max_contour = c
    return max_contour, max_area
    

def drawCircularBound(img, c):
    center, radius = cv2.minEnclosingCircle(c)
    cx, cy = int(center[0]), int(center[1])
    
    hexagon_points = []
    for i in range(6):
        angle = math.pi / 3 * i
        x = int(cx + radius * math.cos(angle))
        y = int(cy + radius * math.sin(angle))
        hexagon_points.append([x, y])
    
    hexagon_points = np.array(hexagon_points, np.int32)
    hexagon_points = hexagon_points.reshape((-1, 1, 2))
    img = cv2.polylines(img, [hexagon_points], True, (255, 0, 0), thickness=2)
    
    return img, (cx, cy)

def processImage(img):
    img_to_cv = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    mask = filterByColor(img_to_cv)
    mx_contour, mx_area = maxCountour(mask)

    cv2_img, center = drawCircularBound(img_to_cv, mx_contour)
    cv2_img = cv2.circle(cv2_img, center, 5, (0, 0, 255), thickness = 5)

    return cv2_img  

def image_callback(msg):
    cv_image = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
    result = processImage(cv_image)
    cv2.imshow("Camera", result)
    cv2.waitKey(1)


def cleanup():
    rospy.loginfo("Shutting down...")
    cv2.destroyWindow("Frame")

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    
    cv2.namedWindow("Camera")
    cv2.namedWindow("Mask")
    
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    