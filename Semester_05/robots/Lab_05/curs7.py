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
pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=-0.65, stance_x=0, stance_y=0, run_time=200)
gait_msg = Gait(overlap_time=0.1, swing_time=0.2, clearance_time=0.3, z_clearance=5)
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

# Control variables
last_command_time = 0
command_duration = 0.5  # Execute each command for 0.5 seconds before recalculating
current_x_speed = 0
current_yaw_rate = 0
current_pitch = 0

# State tracking for better logging
last_state = None

def img_process(img):
    global pose_pub, pose_msg
    global contour_center, radius, max_contour
    global lock, shut
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # Filter for RED color in LAB color space
    img_lab = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(img_lab, np.array([0, 153, 88]), np.array([255, 255, 255]))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    with lock:
        if not shut:
            # Get max contour
            max_area = 0
            temp_contour = None
            for c in contours:
                c_area = math.fabs(cv2.contourArea(c))
                if c_area > max_area:
                    max_area = c_area
                    temp_contour = c
            
            if temp_contour is not None and max_area > 100:
                max_contour = temp_contour
                # Calculate center and radius
                center, rad = cv2.minEnclosingCircle(max_contour)
                contour_center = (int(center[0]), int(center[1]))
                radius = int(rad)
                
                # Draw bounding circle
                cv2.circle(cv2_img, contour_center, radius, (0, 255, 0), 2)
                cv2.circle(cv2_img, contour_center, 5, (0, 0, 255), -1)
                
                # Draw crosshair at center
                cv2.line(cv2_img, (contour_center[0] - 10, contour_center[1]), 
                        (contour_center[0] + 10, contour_center[1]), (0, 255, 0), 2)
                cv2.line(cv2_img, (contour_center[0], contour_center[1] - 10), 
                        (contour_center[0], contour_center[1] + 10), (0, 255, 0), 2)
                
                cv2.putText(cv2_img, f"RED detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(cv2_img, f"Radius: {radius}px", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(cv2_img, f"Center: ({contour_center[0]}, {contour_center[1]})", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                max_contour = None
                contour_center = None
                radius = 0
                cv2.putText(cv2_img, "Searching for RED...", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
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
    global pose_pub, vel_pub, pose_msg
    global contour_center, radius, max_contour
    global lock, shut
    global last_command_time, command_duration, current_x_speed, current_yaw_rate

    img_width = 640
    img_height = 480
    center_x = img_width // 2
    center_y = img_height // 2

    while True:
        time.sleep(0.1)
        current_time = time.time()

        # Only recalculate command if enough time has passed
        if current_time - last_command_time >= command_duration:
            with lock:
                if shut:
                    break

                if max_contour is not None and contour_center is not None:
                    offset_x = contour_center[0] - center_x

                    # Calculate yaw rate (turning)
                    if abs(offset_x) < 30:  # Dead zone - consider centered
                        current_yaw_rate = 0
                    else:
                        # Turn left or right based on offset
                        current_yaw_rate = math.radians(-offset_x / img_width * 25)

                    # Calculate forward speed based on radius (distance)
                    if radius < 50:
                        current_x_speed = 10  # Far away - move fast
                    elif radius < 100:
                        current_x_speed = 6  # Medium distance
                    elif radius < 150:
                        current_x_speed = 3  # Getting close - slow down
                    else:
                        current_x_speed = 0  # Close enough - stop

                    rospy.loginfo(f"Command: speed={current_x_speed}, yaw={math.degrees(current_yaw_rate):.1f}°, radius={radius}")

                else:
                    # No target detected - rotate in place to search
                    current_x_speed = 0
                    current_yaw_rate = math.radians(20)
                    rospy.loginfo("Searching for red object...")

                last_command_time = current_time

        # Publish the current command continuously
        vel_pub.publish(x=current_x_speed, y=0, yaw_rate=current_yaw_rate)

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)
    
    time.sleep(0.2)
    gait_pub.publish(gait_msg)
    time.sleep(0.2)
    pose_pub.publish(pose_msg)
    time.sleep(0.2)
    
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    # create a daemon that will run the "move" function in the background
    # the move function should contain all the logic for moving the robot towards the detected object and for tracking it
    move_th = Thread(target=move, daemon=True)
    move_th.start()
    rospy.spin()
