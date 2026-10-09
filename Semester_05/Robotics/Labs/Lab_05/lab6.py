#!/usr/bin/env python3
# coding=utf8

import sys
import math
import time
import rospy
from std_srvs.srv import SetBool
from puppy_control.msg import Velocity, Pose, Gait


ROS_NODE_NAME = 'puppy_demo'

PuppyMove = {'x':6, 'y':0, 'yaw_rate':0}
# x:直行控制，  前进方向为正方向，单位cm/s
# y:侧移控制，左侧方向为正方向，单位cm/s，目前无此功能
# yaw_rate：转弯控制，逆时针方向为正方向，单位rad/s

PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':0.5, 'stance_x':0, 'stance_y':0}
# PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}

gait = 'Amble'
# overlap_time:4脚全部着地的时间，单位秒
# swing_time：单脚离地时间，单位秒
# clearance_time：前后交叉脚相位间隔时间，单位秒
# z_clearance：走路时，脚尖要抬高的距离，单位cm

if gait == 'Trot':
    GaitConfig = {'overlap_time':0.2, 'swing_time':0.3, 'clearance_time':0.0, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.6

elif gait == 'Amble':
    GaitConfig = {'overlap_time':0.1, 'swing_time':0.2, 'clearance_time':0.1, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.9

elif gait == 'Walk':
    GaitConfig = {'overlap_time':0.1, 'swing_time':0.2, 'clearance_time':0.3, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.65

elif gait == 'Gallop_Rotary':
    GaitConfig = {'overlap_time':0.05, 'swing_time':0.25, 'clearance_time':0.15, 'z_clearance':6}
    PuppyPose['x_shift'] = -0.7

def cleanup():
    PuppyVelocityPub.publish(x=20, y=0, yaw_rate=0)
    print('is_shutdown')

def dance_routine():
    PuppyVelocityPub.publish(x=8, y=0, yaw_rate=0)
    rospy.sleep(1.5)
    PuppyVelocityPub.publish(x=-8, y=0, yaw_rate=0)
    rospy.sleep(1.5)

    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'],
                        x_shift=PuppyPose['x_shift'], height=PuppyPose['height'],
                        roll=PuppyPose['roll'], pitch=math.radians(20), yaw=PuppyPose['yaw'], run_time=300)
    rospy.sleep(0.5)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'],
                        x_shift=PuppyPose['x_shift'], height=PuppyPose['height'],
                        roll=PuppyPose['roll'], pitch=math.radians(-20), yaw=PuppyPose['yaw'], run_time=300)
    rospy.sleep(0.5)

    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'],
                       x_shift=PuppyPose['x_shift'], height=PuppyPose['height'],
                       roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    rospy.sleep(0.2)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)

    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    rospy.sleep(0.2)

    rospy.sleep(1.0)

    dance_routine()

    while True:
        try:
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit(0)
        except :
            sys.exit(0)