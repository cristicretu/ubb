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
# stance_x：4条腿在x轴上额外分开的距离，单位cm
# stance_y：4条腿在y轴上额外分开的距离，单位cm
# x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm
# height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm
# pitch： 狗身体的俯仰角，单位弧度


gait = 'Trot'
# overlap_time:4脚全部着地的时间，单位秒
# swing_time：单脚离地时间，单位秒
# clearance_time：前后交叉脚相位间隔时间，单位秒
# z_clearance：走路时，脚尖要抬高的距离，单位cm

if gait == 'Trot':
    GaitConfig = {'overlap_time':0.2, 'swing_time':0.3, 'clearance_time':0.0, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.6
    # Trot步态 clearance_time = 0

elif gait == 'Amble':
    GaitConfig = {'overlap_time':0.1, 'swing_time':0.2, 'clearance_time':0.1, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.9
    # Amble步态 0 ＜ clearance_time ＜ swing_time

elif gait == 'Walk':
    GaitConfig = {'overlap_time':0.1, 'swing_time':0.2, 'clearance_time':0.3, 'z_clearance':5}
    PuppyPose['x_shift'] = -0.65
    # Walk步态   swing_time ≤ clearance_time

def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('is_shutdown')

def dance_routine():
    print("Starting jump routine!")
    
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    rospy.sleep(0.5)
    
    print("Jump 1! Crouch...")
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=-18, 
                       roll=PuppyPose['roll'], pitch=math.radians(-10), yaw=PuppyPose['yaw'], run_time=150)
    rospy.sleep(0.2)
    print("JUMP!")
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=-5, 
                       roll=PuppyPose['roll'], pitch=math.radians(10), yaw=PuppyPose['yaw'], run_time=100)
    rospy.sleep(0.15)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=PuppyPose['height'], 
                       roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=200)
    
    print("Waiting 2 seconds...")
    rospy.sleep(2.0)
    
    print("Jump 2! Crouch...")
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=-18, 
                       roll=PuppyPose['roll'], pitch=math.radians(-10), yaw=PuppyPose['yaw'], run_time=150)
    rospy.sleep(0.2)
    print("JUMP!")
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=-5, 
                       roll=PuppyPose['roll'], pitch=math.radians(10), yaw=PuppyPose['yaw'], run_time=100)
    rospy.sleep(0.15)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=PuppyPose['height'], 
                       roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=200)
    
    print("Waiting 2 seconds...")
    rospy.sleep(2.0)
    
    print("Jump 3! Crouch...")
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=-18, 
                       roll=PuppyPose['roll'], pitch=math.radians(-10), yaw=PuppyPose['yaw'], run_time=150)
    rospy.sleep(0.2)
    print("JUMP!")
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=-5, 
                       roll=PuppyPose['roll'], pitch=math.radians(10), yaw=PuppyPose['yaw'], run_time=100)
    rospy.sleep(0.15)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], 
                       x_shift=PuppyPose['x_shift'], height=PuppyPose['height'], 
                       roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=200)
    
    print("Jump routine complete!")

if __name__ == '__main__':

    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)
    # 原地踏步服务

    rospy.sleep(0.2)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)

    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    rospy.sleep(0.2)

    set_mark_time_srv(False)
    
    rospy.sleep(1.0)
    
    dance_routine()
    

    while True:
        try:
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit(0)
        except :
            sys.exit(0)
