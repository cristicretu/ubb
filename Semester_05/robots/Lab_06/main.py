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

ROS_NODE_NAME = "cam_control_node"

lock = Lock()
move_th = None

tl = (240, 180)
br = (400, 300)

max_contour = None
contour_center = None
radius = 0
detected_color = None

pose_pub = None
gait_pub = None
vel_pub = None
actionGroupSrv = None

shut = False

pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=0, stance_x=0, stance_y=0, run_time=500)
gait_msg = Gait(overlap_time=0.3, swing_time=0.5, clearance_time=0.0, z_clearance=3)
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

COLOR_RANGES = {
    'blue': {
        'min': np.array([0, 0, 0]),
        'max': np.array([255, 255, 102])
    },
    'green': {
        'min': np.array([0, 0, 77]),
        'max': np.array([255, 107, 255])
    }
}

def filterColor(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    
    for color_name, ranges in COLOR_RANGES.items():
        mask = cv2.inRange(lab, ranges['min'], ranges['max'])
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if math.fabs(cv2.contourArea(c)) > 200:
                return mask, color_name
    
    return None, None

def getMaxContour(mask, thresh=50):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    max_contour = None
    for c in contours:
        c_area = math.fabs(cv2.contourArea(c))
        if c_area > thresh and c_area > max_area:
            max_area = c_area
            max_contour = c
    return max_contour

def drawCircularBound(img, c):
    center, radius = cv2.minEnclosingCircle(c)
    img = cv2.circle(img, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), thickness=2)
    return img, (int(center[0]), int(center[1])), int(radius)

def img_process(img):
    global pose_pub, pose_msg
    global contour_center, radius, max_contour, detected_color
    global lock, shut
    
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    mask, color = filterColor(cv2_img)
    
    with lock:
        if not shut:
            if mask is not None:
                max_contour = getMaxContour(mask)
                detected_color = color
            else:
                max_contour = None
                detected_color = None
            
            if max_contour is not None:
                cv2_img, contour_center, radius = drawCircularBound(cv2_img, max_contour)
                cv2_img = cv2.circle(cv2_img, contour_center, 5, (0, 0, 255), thickness=5)
                cv2.putText(cv2_img, f"Color: {detected_color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2_img = cv2.rectangle(cv2_img, tl, br, (255, 255, 255))
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
    global pose_pub, vel_pub, actionGroupSrv
    global contour_center, radius, detected_color
    global lock, shut
    
    should_kick = False
    kick_color = None
    
    while True:
        send_pose = False
        send_yaw = False
        send_vel = False
        prev_yaw = vel_msg.yaw_rate
        prev_vel = vel_msg.x
        prev_pitch = pose_msg.pitch
        time.sleep(0.2)
        
        with lock:
            if shut:
                break
            if max_contour is not None:
                if radius < 50:
                    if contour_center[1] < tl[1]:
                        if pose_msg.pitch < math.radians(20):
                            pose_msg.pitch += math.radians(2)
                            send_pose = True
                    elif contour_center[1] > br[1]:
                        if pose_msg.pitch > math.radians(-20):
                            pose_msg.pitch -= math.radians(2)
                            send_pose = True
                else:
                    pose_msg.pitch = math.radians(-20)
                    send_pose = prev_pitch != pose_msg.pitch
                
                if radius < 100:
                    if contour_center[0] < tl[0]:
                        vel_msg.yaw_rate = math.radians(5)
                        send_yaw = True
                    elif contour_center[0] > br[0]:
                        vel_msg.yaw_rate = math.radians(-5)
                        send_yaw = True
                    else:
                        vel_msg.yaw_rate = math.radians(0)
                        send_yaw = prev_yaw != vel_msg.yaw_rate
                
                if radius < 70:
                    vel_msg.x = 10
                    send_vel = True
                elif 70 <= radius < 110:
                    vel_msg.x = 5
                    send_vel = True
                else:
                    vel_msg.x = 0
                    send_vel = prev_vel != vel_msg.x
                    should_kick = True
                    kick_color = detected_color
            else:
                vel_msg.yaw_rate = math.radians(-12)
                vel_msg.x = 0
                send_yaw = True
                send_vel = prev_vel != vel_msg.x
        
        if should_kick and kick_color in ('blue', 'green'):
            should_kick = False
            
            pose_msg.pitch = math.radians(0)
            pose_pub.publish(pose_msg)
            time.sleep(0.5)
            
            if kick_color == 'blue':
                vel_msg.yaw_rate = math.radians(10)
            else:
                vel_msg.yaw_rate = math.radians(-10)
            
            vel_msg.x = 5
            vel_pub.publish(vel_msg)
            time.sleep(2.2)
            
            vel_msg.x = 0
            vel_msg.yaw_rate = math.radians(0)
            vel_pub.publish(vel_msg)
            time.sleep(0.5)

            if kick_color == 'blue':
                actionGroupSrv("kick_ball_right.d6ac", True)
            else:
                actionGroupSrv("kick_ball_left.d6ac", True)
            
            kick_color = None
            time.sleep(3)
        else:
            should_kick = False
            if send_pose and pose_pub is not None:
                pose_pub.publish(pose_msg)
            if (not send_pose and (send_vel or send_yaw)) and vel_pub is not None:
                vel_pub.publish(vel_msg)
                time.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)
    actionGroupSrv = rospy.ServiceProxy("/puppy_control/runActionGroup", SetRunActionName)
    gait_pub.publish(gait_msg)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    move_th = Thread(target=move, daemon=True)
    move_th.start()
    rospy.spin()
