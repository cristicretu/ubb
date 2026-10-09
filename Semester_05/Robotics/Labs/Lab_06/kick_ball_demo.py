#!/usr/bin/python3 #1
# coding=utf8 #2
# 第7章 ROS机器狗创意课程\3.AI自主追踪踢球(7.ROS Robot Creative Lesson\3.AI Auto Shooting) #3
import sys #4
import cv2 #5
import time #6
import math #7
import threading #8
import numpy as np #9
from enum import Enum #10

from common import Misc #12

import rospy #14
from std_srvs.srv import * #15
from sensor_msgs.msg import Image #16
from ros_robot_controller.msg import RGBState, RGBsState #17
from object_tracking.srv import * #18
from puppy_control.msg import Velocity, Pose, Gait #19
from puppy_control.srv import SetRunActionName #20

ROS_NODE_NAME = 'kick_ball_demo' #22
is_shutdown = False #23

color_range_list = rospy.get_param('/lab_config_manager/color_range_list') #25

PuppyMove = {'x':0, 'y':0, 'yaw_rate':0} #27


if sys.version_info.major == 2: #30
    print('Please run this program with python3!') #31
    sys.exit(0) #32

lock = threading.Lock() #34
debug = False #35
__isRunning = True #36
haved_detect = False #37

class PuppyStatus(Enum): #39
    LOOKING_FOR = 0 #寻找 先低头寻找，没有就跳转到LOOKING_FOR_LEFT向左一步寻找，再没有的话就跳转到LOOKING_FOR_RIGHT，一直向右寻找(search, start by looking down,  If nothing is found, transition to LOOKING_FOR_LEFT to search one step to the left. If still nothing is found, transition to LOOKING_FOR_RIGHT and continue searching to the right) #40
    LOOKING_FOR_LEFT = 1 #41
    LOOKING_FOR_RIGHT = 2  #42
    FOUND_TARGET = 3 # 已经发现目标(target acquired) #43
    CLOSE_TO_TARGET = 4 # 靠近目标(approach target) #44
    CLOSE_TO_TARGET_FINE_TUNE = 5 # 细调(fine-tuning) #45
    KICK_BALL = 6 # 踢球(shooting) #46
    STOP = 10 #47
    END = 20             #48

puppyStatus = PuppyStatus.LOOKING_FOR #50
puppyStatusLast = PuppyStatus.END #51


expect_center = {'X':640/2,'Y':480/2} # #54
expect_center_kick_ball_left = {'X':150,'Y':480-150} # 左脚踢球，希望小球到达此坐标开始踢(left foot kicking the ball, aiming for it to reach this coordinate before starting) #55
expect_center_kick_ball_right = {'X':640-150,'Y':480-150}# 右脚踢球，希望小球到达此坐标开始踢(right foot kicking the ball, aiming for it to reach this coordinate before starting) #56
target_info = None # 小球中心点坐标(coordinates of the center point of the ball) #57

range_rgb = { #59
    'red': (0, 0, 255), #60
    'blue': (255, 0, 0), #61
    'green': (0, 255, 0), #62
    'black': (0, 0, 0), #63
    'white': (255, 255, 255), #64
} #65

color_list = [] #67
detect_color = 'None' #68
action_finish = True #69
draw_color = range_rgb["black"] #70
__target_color = ('green',) #71


# 找出面积最大的轮廓(find out the contour with the maximal area) #74
# 参数为要比较的轮廓的列表(the parameter is the list of contour to be compared) #75
def getAreaMaxContour(contours): #76
    contour_area_temp = 0 #77
    contour_area_max = 0 #78
    area_max_contour = None #79

    for c in contours:  # 历遍所有轮廓(iterate through all contours) #81
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate the contour area) #82
        if contour_area_temp > contour_area_max: #83
            contour_area_max = contour_area_temp #84
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 300, the contour with the largest area is considered valid to filter out interference) #85
                area_max_contour = c #86

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour) #88


def move(): #91
    global detect_color #92
    global puppyStatus, puppyStatusLast, haved_detect, action_finish, target_info, PuppyPose #93
    time.sleep(2) #94

    while True: #96
        time.sleep(0.01) #97
        while(puppyStatus == PuppyStatus.LOOKING_FOR) : #98
            if haved_detect: #99
                puppyStatus = PuppyStatus.FOUND_TARGET #100
                break #101
            with lock: #102
                PuppyPose = PP['LookDown_10deg'].copy() #103
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'] #104
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw']) #105
                time.sleep(0.2) #106
            time.sleep(0.8) #107
            puppyStatus = PuppyStatus.LOOKING_FOR_LEFT #108
            break #109
        while(puppyStatus == PuppyStatus.LOOKING_FOR_LEFT) : #110
            if haved_detect: #111
                puppyStatus = PuppyStatus.FOUND_TARGET #112
                break #113
            with lock: #114
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(10)) #115
                time.sleep(3) #116
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0)) #117
                time.sleep(0.3) #118
            time.sleep(0.8) #119
            puppyStatus = PuppyStatus.LOOKING_FOR_RIGHT #120
            break #121
        while(puppyStatus == PuppyStatus.LOOKING_FOR_RIGHT) : #122
            if haved_detect: #123
                puppyStatus = PuppyStatus.FOUND_TARGET #124
                break #125

            PuppyPose = PP['LookDown_10deg'].copy() #127
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'] #128
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw']) #129
            time.sleep(0.2) #130
            PuppyVelocityPub.publish(x=2, y=0, yaw_rate = math.radians(-12)) #131
            break #132
        while(puppyStatus == PuppyStatus.FOUND_TARGET) : #133
            # with lock: #134
            if target_info['centerY'] > 380: #135
                puppyStatus = PuppyStatus.CLOSE_TO_TARGET #136
                PuppyPose = PP['LookDown_20deg'].copy() #137
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'] #138
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw']) #139
                time.sleep(0.2) #140
                break #141
            if expect_center['X'] - target_info['centerX'] < -80: #142
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(-12)) #143
                time.sleep(0.2) #144
            elif expect_center['X'] - target_info['centerX'] > 80: #145
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(12)) #146
                time.sleep(0.2) #147
            else: #148
                PuppyVelocityPub.publish(x=10, y=0, yaw_rate = math.radians(0)) #149
                time.sleep(0.2) #150
            break #151
        while(puppyStatus == PuppyStatus.CLOSE_TO_TARGET) : #152
            # with lock: #153
            if target_info['centerY'] > 380: #154
                PuppyMove['x'] = 0 #155
                PuppyMove['yaw_rate'] = math.radians(0) #156
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0)) #157
                puppyStatus = PuppyStatus.CLOSE_TO_TARGET_FINE_TUNE #158
                print('expect_center[X] , target_info[centerX]',expect_center['X'] , target_info['centerX']) #159
                if expect_center['X'] > target_info['centerX']: #160
                    which_foot_kick_ball = 'left' #161
                else: #162
                    which_foot_kick_ball = 'right' #163
                print(which_foot_kick_ball) #164
                break #165
            if expect_center['X'] - target_info['centerX'] < -50: #166
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(-10)) #167
                time.sleep(0.2) #168
            elif expect_center['X'] - target_info['centerX'] > 50: #169
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(10)) #170
                time.sleep(0.2) #171
            else: #172
                PuppyVelocityPub.publish(x=8, y=0, yaw_rate = math.radians(0)) #173
                time.sleep(0.2) #174
            # print(target_info) #175
            break #176
        
        while(puppyStatus == PuppyStatus.CLOSE_TO_TARGET_FINE_TUNE) : #178

            if target_info['centerY'] < expect_center_kick_ball_left['Y']: #180
                PuppyVelocityPub.publish(x=4, y=0, yaw_rate = math.radians(0)) #181
                time.sleep(0.1) #182
            elif which_foot_kick_ball == 'left' and target_info['centerX'] > expect_center_kick_ball_left['X']: #183
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(-8)) #184
                time.sleep(0.1) #185
            elif which_foot_kick_ball == 'right' and target_info['centerX'] < expect_center_kick_ball_right['X']: #186
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(8)) #187
                time.sleep(0.1) #188
            else:# 最后一次微调(the final fine-tuning) #189
                if which_foot_kick_ball == 'left': #190
                    PuppyVelocityPub.publish(x=5, y=0, yaw_rate = math.radians(-10)) #191
                else: #192
                    PuppyVelocityPub.publish(x=5, y=0, yaw_rate = math.radians(10)) #193
                time.sleep(1.8) #194
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0)) #195
                puppyStatus = PuppyStatus.KICK_BALL #196
            # PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0)) #197
            # time.sleep(0.1)#停下来需要稳定的时间(the time required to come to a stable stop) #198
            time.sleep(0.1) #199
            break #200
        while(puppyStatus == PuppyStatus.KICK_BALL) : #201
            with lock: #202
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0)) #203
                time.sleep(0.2) #204
                if which_foot_kick_ball == 'left': #205
                    runActionGroup_srv('kick_ball_left.d6ac',True) #206
                else: #207
                    runActionGroup_srv('kick_ball_right.d6ac',True) #208
                puppyStatus = PuppyStatus.LOOKING_FOR #209
                haved_detect = False #210

        if puppyStatus == PuppyStatus.STOP: #212
            PuppyMove['x'] = 0 #213
            PuppyMove['yaw_rate'] = math.radians(0) #214
            PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0)) #215

        
        if puppyStatusLast != puppyStatus: #218
            print('puppyStatus',puppyStatus) #219
        puppyStatusLast = puppyStatus #220

        if is_shutdown:break #222

# 运行子线程(run sub-thread) #224
th = threading.Thread(target=move) #225
th.setDaemon(True) #226
# th.start() #227


size = (320, 240) #230
def run(img): #231
    global draw_color #232
    global color_list #233
    global detect_color #234
    global action_finish #235
    global haved_detect #236
    global target_info  #237
    # img_copy = img.copy() #238
    img_h, img_w = img.shape[:2] #239

    if not __isRunning: #241
        return img #242

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST) #244
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)       #245
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space) #246

    max_area = 0 #248
    color_area_max = None     #249
    areaMaxContour_max = 0 #250
    
    if True:#action_finish #252
        for i in color_range_list: #253
            if i in __target_color: #254
                frame_mask = cv2.inRange(frame_lab, #255
                                             (color_range_list[i]['min'][0], #256
                                              color_range_list[i]['min'][1], #257
                                              color_range_list[i]['min'][2]), #258
                                             (color_range_list[i]['max'][0], #259
                                              color_range_list[i]['max'][1], #260
                                              color_range_list[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask) #261
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion) #262
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation) #263
                if debug: #264
                    cv2.imshow(i, dilated) #265
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out the contour) #266
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the maximal contour) #267
                if areaMaxContour is not None: #268
                    if area_max > max_area:#找最大面积(find the maximal area) #269
                        max_area = area_max #270
                        color_area_max = i #271
                        areaMaxContour_max = areaMaxContour #272
                       
        if max_area > 200:  # 200   #274

            rect = cv2.minAreaRect(areaMaxContour_max)#最小外接矩形(the minimal bounding rectangle) #276
            
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle) #278
            centerX = int(Misc.map(rect[0][0], 0, size[0], 0, img_w)) #279
            centerY = int(Misc.map(rect[0][1], 0, size[1], 0, img_h)) #280
            sideX = int(Misc.map(rect[1][0], 0, size[0], 0, img_w)) #281
            sideY = int(Misc.map(rect[1][1], 0, size[1], 0, img_h)) #282
            angle = rect[2] #283
            for i in range(4): #284
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h)) #285
            for i in range(4):                 #286
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w)) #287
            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形(draw a rectangle formed by connecting the four points) #288


            if color_area_max == 'red':  #红色最大(red is the maximal area) #291
                color = 1 #292
            elif color_area_max == 'green':  #绿色最大(green is the maximal area) #293
                color = 2 #294
            elif color_area_max == 'blue':  #蓝色最大(blue is the maximal area) #295
                color = 3 #296
            else: #297
                color = 0 #298
            color_list.append(color) #299

            if len(color_list) == 3:  #多次判断(multiple judgement) #301
                # 取平均值(take the average value) #302
                color = int(round(np.mean(np.array(color_list)))) #303
                color_list = [] #304
                if color == 1: #305
                    detect_color = 'red' #306
                    draw_color = range_rgb["red"] #307
                elif color == 2: #308
                    detect_color = 'green' #309
                    draw_color = range_rgb["green"] #310
                elif color == 3: #311
                    detect_color = 'blue' #312
                    draw_color = range_rgb["blue"] #313
                else: #314
                    detect_color = 'None' #315
                    draw_color = range_rgb["black"]                #316
        else: #317
            detect_color = 'None' #318
            draw_color = range_rgb["black"] #319
        if detect_color == 'red': #320
            haved_detect = True #321
            if sideX > sideY: #322
                target_info = {'centerX':centerX, 'centerY':centerY, 'sideX':sideX, 'sideY':sideY, 'scale':sideX/sideY, 'angle':angle} #323
            else: #324
                target_info = {'centerX':centerX, 'centerY':centerY, 'sideX':sideX, 'sideY':sideY, 'scale':sideY/sideX, 'angle':angle} #325
        else: #326
            haved_detect = False #327
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2) #328
    
    return img #330


def image_callback(ros_image): #333
    # global lock #334
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, #336
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customize image information to image) #337
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) #338
    
    frame = cv2_img.copy() #340
    frame_result = frame #341
    with lock: #342
        if __isRunning: #343
            frame_result = run(frame) #344
            cv2.imshow('Frame', frame_result) #345
            key = cv2.waitKey(1) #346

def cleanup(): #348
    global is_shutdown #349
    is_shutdown = True #350
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0) #351
    print('is_shutdown') #352
if __name__ == '__main__': #353
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG) #354
    rospy.on_shutdown(cleanup) #355

    PP = rospy.get_param('/puppy_control/PuppyPose') #357
    PuppyPose = PP['LookDown_10deg'].copy() #358
    
    GaitConfig = {'overlap_time':0.15, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':3} #360

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback) #362

    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher #364


    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1) #367
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1) #368
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1) #369

    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName) #371

    rospy.sleep(0.3) #373
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'] #374
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500) #375
    
    rospy.sleep(0.2) #377
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time'] #378
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance']) #379
    rospy.sleep(0.2) #380
    
    debug = False #382
    if debug == False: #383
        th.start() #384

    try: #386
        rospy.spin() #387
    except KeyboardInterrupt: #388
        print("Shutting down") #389
    finally: #390
        cv2.destroyAllWindows() #391

