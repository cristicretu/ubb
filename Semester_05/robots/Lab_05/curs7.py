#!/usr/bin/env python3
import rospy
import cv2
import math
import numpy as np
import time
from threading import Thread, Lock
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from puppy_control.msg import Pose, Gait, Velocity
from puppy_control.srv import SetRunActionName

ROS_NODE_NAME = "move_on_detect_node"

lock = Lock()
move_th = None

max_contour = None
contour_center = None
radius = 0

pose_pub = None
gait_pub = None
vel_pub = None

shut = False
pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=0, stance_x=0, stance_y=0, run_time=500)
gait_msg = Gait(overlap_time=0.3, swing_time=0.5, clearance_time=0.0, z_clearance=3)
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

def img_process(img):
    global pose_pub, pose_msg
    global contour_center, radius, max_contour
    global lock, shut
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # find all contours
    with lock:
        if not shut:
            #max_contour = get max contour and store in this variable
            if max_contour is not None:
                # calculate the center of the contour and estimate the size of the contour               
                # draw the bounding circle or box around the contour
                pass
        cv2.imshow("Frame", cv2_img)
        cv2.waitKey(1)

def cleanup():
    global shut, lock
    with lock:
        shut = True
    rospy.loginfo("Shutting down...")
    cv2.destroyWindow("Frame")
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()

def move():
    global pose_pub, vel_pub
    global contour_center, radius
    global lock, shut
    while True:
        time.sleep(0.2)
        with lock:
            if shut:
                break
            if max_contour is not None:
                # if there is a contour, decide how to move and change pitch
                pass
            else:
                # if no contour is detected, do something else
                pass

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)
    
    gait_pub.publish(gait_msg)
    
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    # create a daemon that will run the "move" function in the background
    # the move function should contain all the logic for moving the robot towards the detected object and for tracking it
    move_th = Thread(target=move, daemon=True)
    move_th.start()
    rospy.spin()
