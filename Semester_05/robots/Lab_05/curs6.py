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

ROS_NODE_NAME = "color_tracking_node"

lock = Lock()
shutdown_flag = False

max_contour = None
contour_center = None
object_radius = 0

pose_pub = None
gait_pub = None
vel_pub = None

current_mode = "SEARCHING"
last_pitch_update_time = 0
current_pitch = 0
last_logged_state = None

IMG_WIDTH = 640
IMG_HEIGHT = 480
CENTER_X = IMG_WIDTH // 2
CENTER_Y = IMG_HEIGHT // 2

APPROACH_COMPLETE_RADIUS = 60
OPTIMAL_RADIUS_MIN = 80

MIN_DETECTION_FRAMES = 3
detection_count = 0

HORIZONTAL_DEADZONE = 40
VERTICAL_DEADZONE = 50

PITCH_UPDATE_INTERVAL = 0.5
PITCH_CHANGE_THRESHOLD = math.radians(5)
MAX_PITCH = math.radians(25)

pose_msg = Pose(
    roll=math.radians(0),
    pitch=math.radians(0),
    yaw=0,
    height=-10,
    x_shift=-0.65,
    stance_x=0,
    stance_y=0,
    run_time=200
)

gait_msg = Gait(overlap_time=0.1, swing_time=0.2, clearance_time=0.3, z_clearance=5)


def img_process(img):
    global max_contour, contour_center, object_radius, shutdown_flag, lock

    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    img_lab = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(img_lab, np.array([0, 153, 88]), np.array([255, 255, 255]))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    with lock:
        if not shutdown_flag:
            max_area = 0
            temp_contour = None

            for c in contours:
                c_area = math.fabs(cv2.contourArea(c))
                if c_area > max_area:
                    max_area = c_area
                    temp_contour = c

            if temp_contour is not None and max_area > 100:
                max_contour = temp_contour
                center, rad = cv2.minEnclosingCircle(max_contour)
                contour_center = (int(center[0]), int(center[1]))
                object_radius = int(rad)

                cv2.circle(cv2_img, contour_center, object_radius, (0, 255, 0), 2)
                cv2.circle(cv2_img, contour_center, 5, (0, 0, 255), -1)
                cv2.putText(cv2_img, f"R:{object_radius}px ({contour_center[0]},{contour_center[1]})",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                max_contour = None
                contour_center = None
                object_radius = 0
                cv2.putText(cv2_img, "Searching...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("Camera", cv2_img)
    cv2.waitKey(1)


def update_pitch_safely(new_pitch):
    global pose_pub, pose_msg, current_pitch

    pose_msg.pitch = new_pitch
    pose_pub.publish(pose_msg)

    current_pitch = new_pitch


def control_loop():
    global shutdown_flag, lock, vel_pub, pose_pub
    global max_contour, contour_center, object_radius
    global current_mode, last_pitch_update_time, current_pitch, last_logged_state
    global detection_count

    smoothed_x_speed = 0
    smoothed_yaw_rate = 0

    while not rospy.is_shutdown():
        time.sleep(0.08)

        with lock:
            if shutdown_flag:
                break
            has_target = max_contour is not None and contour_center is not None
            radius = object_radius
            center = contour_center

        target_x_speed = 0
        target_yaw_rate = 0

        if not has_target:
            detection_count = max(0, detection_count - 1)
            if detection_count == 0:
                if current_mode != "SEARCHING":
                    current_mode = "SEARCHING"
                target_x_speed = 0
                target_yaw_rate = 0
        else:
            detection_count = min(MIN_DETECTION_FRAMES, detection_count + 1)

            if detection_count >= MIN_DETECTION_FRAMES:
                offset_x = center[0] - CENTER_X
                offset_y = center[1] - CENTER_Y
                horizontal_centered = abs(offset_x) < HORIZONTAL_DEADZONE
                vertical_centered = abs(offset_y) < VERTICAL_DEADZONE

                if horizontal_centered:
                    target_yaw_rate = 0
                else:
                    target_yaw_rate = math.radians(-offset_x / IMG_WIDTH * 25)

                current_time = time.time()
                if current_time - last_pitch_update_time >= PITCH_UPDATE_INTERVAL:
                    if not vertical_centered:
                        target_pitch = math.radians(-offset_y / IMG_HEIGHT * 30)
                        target_pitch = max(min(target_pitch, MAX_PITCH), -MAX_PITCH)

                        if abs(target_pitch - current_pitch) > PITCH_CHANGE_THRESHOLD:
                            update_pitch_safely(target_pitch)
                            last_pitch_update_time = current_time

                if radius < OPTIMAL_RADIUS_MIN:
                    if current_mode != "APPROACH":
                        current_mode = "APPROACH"

                    distance_error = OPTIMAL_RADIUS_MIN - radius
                    if distance_error > 50:
                        target_x_speed = 6
                    elif distance_error > 25:
                        target_x_speed = 4
                    else:
                        target_x_speed = 2

                else:
                    if current_mode != "OPTIMAL":
                        current_mode = "OPTIMAL"
                    target_x_speed = 0
                    smoothed_x_speed = 0

        if target_x_speed == 0 and smoothed_x_speed != 0:
            smoothed_x_speed = 0
        else:
            smoothed_x_speed = smoothed_x_speed * 0.5 + target_x_speed * 0.5

        smoothed_yaw_rate = smoothed_yaw_rate * 0.5 + target_yaw_rate * 0.5

        vel_pub.publish(Velocity(x=smoothed_x_speed, y=0, yaw_rate=smoothed_yaw_rate))


def cleanup():
    global shutdown_flag, lock, vel_pub

    with lock:
        shutdown_flag = True

    if vel_pub is not None:
        vel_pub.publish(Velocity(x=0, y=0, yaw_rate=0))
        time.sleep(0.5)

    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    time.sleep(0.5)

    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)

    time.sleep(0.2)
    gait_pub.publish(gait_msg)
    time.sleep(0.2)
    pose_pub.publish(pose_msg)
    time.sleep(0.2)

    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)

    control_thread = Thread(target=control_loop, daemon=True)
    control_thread.start()

    rospy.spin()
