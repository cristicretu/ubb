#!/usr/bin/env python3 
import rospy
import cv2
import math
import numpy as np
import time
from std_srvs.srv import Empty
from puppy_control.msg import Pose, Gait, Velocity

ROS_NODE_NAME = "move_node"

# roll - rotation along the horizontal axis (length-wise) - the robot will tilt left/right
# pitch - rotation along the horizontal axis (width-wise) - the robot will tilt forwards/backwards
# yaw - not applicable for this robot - rotation along the vertical axis - the robot will rotate left/right
#   roll and pitch (-30 to +30)
# height (-5 to -15) - distance from the ground, -5 is closest to the ground, -15 is furthest from the ground
# x_shift - shifts the center of mass forwards or backwards - used to maintain balance during movement
pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=0, stance_x=0, stance_y=0, run_time=200)

# You may test each one of these and see for yourself how the robot moves
# Trot
pose_msg.x_shift = -0.6
gait_msg = Gait(overlap_time=0.2, swing_time=0.3, clearance_time=0.0, z_clearance=5)

# Amble
#pose_msg.x_shift = -0.9
#gait_msg = Gait(overlap_time=0.1, swing_time=0.2, clearance_time=0.1, z_clearance=5)

# Walk
#pose_msg.x_shift = -0.65
#gait_msg = Gait(overlap_time=0.1, swing_time=0.2, clearance_time=0.3, z_clearance=5)

##############

# Full stop
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

def cleanup():
    rospy.loginfo("Shutting down...")
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)

    # setup gait
    # this defines how the robot will move its legs during horizontal motion
    # publishing the gait should be done only when we want to change it, in this case, we publish it once and that is enough
    # (try the gait examples above)
    time.sleep(0.2)
    gait_pub.publish(gait_msg)
    
    # setup the pose once and then wait for 200 ms for the servos to actually perform the change 
    pose_pub.publish(pose_msg)
    time.sleep(0.2)
   
    i = 0
    # for a total of 10 seconds (10 iterations of the while, each with a sleep of 1 second), we will send a message indicating that the robot should move forward 
    while not rospy.is_shutdown():
        i = i + 1
        if i < 10:
            # if x is set to a negative number, the robot will move backwards
            vel_msg.x = -10
            # if we set yaw_rate to a non zero number, the robot will steer
            # positive values for left, negative values for right
            vel_msg.yaw_rate = math.radians(0)
        else:
            vel_msg.x = 0
            vel_msg.yaw_rate = 0
        vel_pub.publish(vel_msg)
        time.sleep(1)
        if i >= 10: 
            break
