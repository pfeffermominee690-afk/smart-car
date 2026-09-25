#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
import argparse
import sys
from sensor_msgs.msg import LaserScan
from laser_test.msg import laser_control as LaserControl
import math
import os
#from skimage import morphology
import time
import warnings

TEMPLATE_DIR = os.path.dirname(os.path.abspath(__file__))

global now_signal
now_signal=0
global wanpai
# wanpai=[1,0,2,2,4]
wanpai=[0,2,0,1,1,4]
global last_time
last_time=0
global mode2_time_inte
mode2_time_inte=0
global mid_angle
mid_angle=3.5
global  mode1_1
mode1_1=0
global time_mode1
time_mode1=0
global line_k
line_k=0
global mid_x
mid_x=0
global mid_y
mid_y=0
global mode1_m
mode1_m=0
global e_time
e_time=0
global stop_turn
stop_turn=False
global time_go_setup
time_go_setup=0
global time_turn_setup
time_turn_setup=0
global x0 
x0 = 1
global peak_thresh
peak_thresh = 50 
global n 
n = 0
global laser_cmd
laser_cmd = False
global msg 
msg = AckermannDriveStamped()
global flag1 
flag1 = 0
global flag2 
flag2 = 0
global flag3 
flag3 = 0
global mode
mode=4
global _traffic_sign
_traffic_sign=[]
global traffic_sign
traffic_sign=0
global c0,c1
global imid_x
imid_x=0
global imid_y
imid_y=0
global rearfront
rearfront=0
intrinsicMat = np.array([[489.3828, 0.8764, 297.5558],
                            [0, 489.8446, 230.0774],
                            [0, 0, 1]])
distortionCoe = np.array([-0.4119,0.1709,0,0.0011, 0.018])


#src_pts = np.float32([[128,378],[1,435],[639,435],[488,378]]) #[220,306],[1,435],[639,435],[451,306]   [132,358],[1,435],[639,435],[501,353]   [160,334],[38,378],[628,378],[502,334] 
src_pts = np.float32([[108,378],[1,435],[639,435],[508,378]])
dst_pts = np.float32([[150,0],[150,480],[490,480],[490,0]])   #dst_pts = np.float32([[70,0],[70,480],[570,480],[570,0]])
showMe = 0


#net = jetson.inference.imageNet("resnet-18",labels="labels.txt")

def light_detection(image):      
    time_mbpp=time.time()
    imgmid_x=0
    imgmid_y=0
   # image=cv2.resize(image,(480,390))
    # cv2.imshow("image",image)
    # print(image.shape)
    have_sign=0
    flag = 1#播放视频
    # count = 0;#记录照相的次数
    # image = cv2.undistort(image, intrinsicMat, distortionCoe, None, intrinsicMat)
    # cv2.imshow("image",image)
    # cv2.waitKey(25)
    result=[]
    
    gray_a = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray_a, (9, 9), 2, 2)
    gray=cv2.Canny(gray, 40, 80)#40,80            
    # cv2.imshow('img',gray)#显示当前摄像头画面
    # cv2.waitKey(55)
    (cnts, _) = cv2.findContours(gray.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)#检测物体轮廓
    # loop over our contours to find hexagon
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:30]
    # print(cnts)
    screenCnt = None
    max_rect=100000
    max_cri=100000
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)#轮廓周长
        area=cv2.contourArea(c)
        rect_err=abs((peri/4)*(peri/4)-area)
        cri_err=abs((peri*peri/(4*3.14159))-area)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)#折线化曲线
        x,y,w,h = cv2.boundingRect(c)
        # if our approximated contour has four points, then
        # we can assume that we have found our squeare
        
        if peri>300 and cri_err<area*0.15:
            if max_cri>area:
                max_cri=area

        if peri>300 and rect_err<area*0.3:
            if max_rect>area:
                max_rect=area
    
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)#轮廓周长
        area=cv2.contourArea(c)
        rect_err=abs((peri/4)*(peri/4)-area)
        cri_err=abs((peri*peri/(4*3.14159))-area)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)#折线化曲线
        # if our approximated contour has four points, then
        # we can assume that we have found our squeare
        
        if area==max_rect or area==max_cri:
            # print(len(approx))
            # print("rect_err",rect_err)
            # print("cri_err" ,cri_err)
            # print('peri', peri)
            # print('area', area)
            screenCnt = approx
            x,y,w,h = cv2.boundingRect(c)#用一个矩形框住像素
            imgmid_x=x+w/2
            imgmid_y=y+h/2
            sign_cri = image[int(y):int(y+h),int(x):int(x+w)]#提取矩形部分
            # sign_cri=cv2.resize(sign_cri,(200,200))
            have_sign=1
            # cv2.imshow("rect or cri",sign_cri)
            # cv2.waitKey(50)
            # # time.sleep(1)
    red_lower = np.array([0,43,46])  
    red_upper = np.array([10,255,255])
    green_lower = np.array([35,43,46])
    green_upper = np.array([77,255,255])
    light_cmd = 0
    if have_sign==1:
        hsv = cv2.cvtColor(sign_cri, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)   #红色的两种界限
        green_mask =cv2.inRange(hsv, green_lower, green_upper)#绿色的界限
        count_red=len(red_mask[red_mask==255])
        count_green=len(green_mask[green_mask==255])
        # print("count_red = ",count_red)
        # print("count_blue = ",count_green)
        if count_red>4000:
            redLight = 1
        else:
            redLight = 0
        if count_green>4000:
            greenLight = 1
        else:
            greenLight = 0
        if redLight == 1:
            light_cmd = 0
        if greenLight == 1:
            light_cmd = 1
    #print("redLight=",redLight)
    # print("greenLight=",greenLight)
    return light_cmd

def direction_test(image,tem):

    #cv.imshow('canny',canny)
    #cv.waitKey(3)
    undstrt = cv2.undistort(image, intrinsicMat, distortionCoe, None, intrinsicMat)
    #masked,x,y,w,h = findSquare(undstrt)
    gray = cv2.cvtColor(undstrt, cv2.COLOR_BGR2GRAY)
    
    tem = cv2.cvtColor(tem, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 100, 200)
    tem_result = cv2.matchTemplate(canny, tem, 4)
    #print((left.max(),right.max()))
    '''
    l = left.max()
    r = right.max()
    print(max(l,r))
    if l<2000000 and r<2000000:
        print('null)
    if l>r:
        print('left')
        return -1
    else:
        print('right')
        return 1
        '''
   # print(tem_result.max())
    return tem_result.max()
    


def mbpp(image):
    time_mbpp=time.time()
    imgmid_x=0
    imgmid_y=0
   # image=cv2.resize(image,(480,390))
    # cv2.imshow("image",image)
    # print(image.shape)
    have_sign=0
    flag = 1#播放视频
    # count = 0;#记录照相的次数
    # image = cv2.undistort(image, intrinsicMat, distortionCoe, None, intrinsicMat)
    # cv2.imshow("image",image)
    # cv2.waitKey(25)
    result=[]
    
    gray_a = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray_a, (3, 3), 2, 2)
    # cv2.imshow('gray',gray)#显示当前摄像头画面
    # cv2.waitKey(55)
    gray=cv2.Canny(gray, 150, 300)#40,80            
    # cv2.imshow('img',gray)#显示当前摄像头画面
    # cv2.waitKey(55)
    (cnts, _) = cv2.findContours(gray.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # loop over our contours to find hexagon
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:30]
    # print(cnts)
    screenCnt = None
    max_rect=0
    max_cri=0
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)#轮廓周长
        area=cv2.contourArea(c)
        rect_err=abs((peri/4)*(peri/4)-area)
        cri_err=abs((peri*peri/(4*3.14159))-area)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)#折线化曲线
        x,y,w,h = cv2.boundingRect(c)
        # if our approximated contour has four points, then
        # we can assume that we have found our squeare
        
        if area>500 and cri_err<area*0.15   and abs(h-w)<0.2*h:
            if max_cri<area:
                max_cri=area

        if area>500 and rect_err<area*0.2 and abs(h-w)<0.2*h:
            if max_rect<area:
                max_rect=area
    
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)#轮廓周长
        area=cv2.contourArea(c)
        rect_err=abs((peri/4)*(peri/4)-area)
        cri_err=abs((peri*peri/(4*3.14159))-area)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)#折线化曲线
        # if our approximated contour has four points, then
        # we can assume that we have found our squeare
        
        if area==max_rect or area==max_cri:
            # print(len(approx))
            # print("rect_err",rect_err)
            # print("cri_err" ,cri_err)
            # print('peri', peri)
            # print('area', area)
            screenCnt = approx
            x,y,w,h = cv2.boundingRect(c)#用一个矩形框住像素
            imgmid_x=x+w/2
            imgmid_y=y+h/2
            sign_cri = gray_a[int(y):int(y+h),int(x):int(x+w)]#提取矩形部分
            sign_cri=cv2.resize(sign_cri,(200,200))
            have_sign=1
            # cv2.imshow("front_rect or cri",sign_cri)
            # # # time.sleep(1)
            # cv2.waitKey(45)
    tpl_go= cv2.imread(os.path.join(TEMPLATE_DIR, "go.png"),0)
    tpl_tr= cv2.imread(os.path.join(TEMPLATE_DIR, "tr.png"),0)
    tpl_tl= cv2.imread(os.path.join(TEMPLATE_DIR, "tl.png"),0)
    # print(tpl_go.shape)
    tpl_tb= cv2.imread(os.path.join(TEMPLATE_DIR, "tb.png"),0)
    tpl_st= cv2.imread(os.path.join(TEMPLATE_DIR, "st.png"),0)
    
    if have_sign==1:
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv2.matchTemplate(sign_cri, tpl_go, cv2.TM_CCORR_NORMED))
        result.append(max_val)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv2.matchTemplate(sign_cri, tpl_tr, cv2.TM_CCORR_NORMED))
        result.append(max_val)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv2.matchTemplate(sign_cri, tpl_tl, cv2.TM_CCORR_NORMED))
        result.append(max_val)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv2.matchTemplate(sign_cri, tpl_tb, cv2.TM_CCORR_NORMED))
        result.append(max_val)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv2.matchTemplate(sign_cri, tpl_st, cv2.TM_CCORR_NORMED))
        result.append(max_val)
        
        best_value=max(result)
        best_class=result.index(best_value)%5  
        print('time_mbpp',time.time()-time_mbpp)
        if best_value<0.95:
            return 10,0,0,0
        else:
            # print('best_class',best_class)
            return best_class,best_value,imgmid_x,imgmid_y

    else:
        print('time_mbpp',time.time()-time_mbpp)
        return 10,0,0,0

def display(img,title,color=1):
    '''
    func:display image
    img: rgb or grayscale
    title:figure title
    color:show image in color(1) or grayscale(0)
    '''
    if showMe:
        if color:
            plt.imshow(img)
        else:
            plt.imshow(img ,cmap='gray')
        plt.title(title)
        plt.axis('off')
        plt.show()




def birdView(img,M):
    '''
    Transform image to birdeye view
    img:binary image
    M:transformation matrix
    return a wraped image
    '''
    img_sz = (img.shape[1],img.shape[0])
    img_warped = cv2.warpPerspective(img,M,img_sz,flags = cv2.INTER_LINEAR)
    #透视变换后进行缩放
    # wrows,wcols= img_warped.shape[0:2]
    # wsize = (int(wrows*0.6),int(wcols*0.6))
    # img_warped = cv2.resize(img_warped,wsize)
    return img_warped
def perspective_transform(src_pts,dst_pts):
    '''
    perspective transform
    args:source and destiantion points
    return M and Minv
    '''
    M = cv2.getPerspectiveTransform(src_pts,dst_pts)
    Minv = cv2.getPerspectiveTransform(dst_pts,src_pts)
    return {'M':M,'Minv':Minv}
# original image to bird view (transformation)


def yolov5detection():
    #os.system('python /home/nano/config/teleop/src/yolov5_detect/detect.py')
    # time.sleep(20)
    # path ="/home/nano/config/teleop/src/yolov5_detect/runs/detect/exp10/labels"
    # detect_list=os.listdir(path)  #文件夹里的每一个文件名收集
    #按照角标的顺序，0是stop 1是右转，2是左转，3是直行，4是掉头
    labeldetect=[0,0,0,0,0]
    # #print(detect_list)
    # for  i in range(0,len(detect_list)-1):
    #     txt=detect_list[i]
    #     f= open("/home/nano/config/teleop/src/yolov5_detect/runs/detect/exp10/labels/%s" % txt) 
    #     #print(f)
    #     line = f.readline(while True:
    

    # 获取labels文件夹中的所有txt文件
    while True:
        files = [f for f in os.listdir("/home/nano/config/teleop/src/yolov5_detect/runs/detect/exp10/labels") if f.endswith(".txt")]
            
            # 如果没有txt文件，则退出循环
        if len(files) == 0:
            break
            
        # 循环处理每个txt文件
        for file in files:
            # 打开txt文件并读取一行内容
            with open(os.path.join("labels", file), "r") as f:
                line = f.readline() #读文件里的每一行
        #print(line)

                mark=[] #标志
                x1=[]   #两个角的xy坐标
                y1=[]
                x2=[]
                y2=[]
                area=[]
                for line in f:
                    trainingSet = line.split(' ')
                    mark.append(float(trainingSet[0]))  #第一列是标志
                    x1.append(float(trainingSet[1]))
                    y1.append(float(trainingSet[2]))
                    x2.append(float(trainingSet[3]))
                    y2.append(float(trainingSet[4]))
                f.close()
            if len(mark)>1:
                for j in range(0,len(mark)-1):
                    area.append(abs(x1[j]-x2[j])*abs(y1[j]-y2[j]))
            maxx=max(area)
            k=area.index(maxx) #找到最大值对应的下标
            labeldetect[mark[k]]=labeldetect[mark[k]]+1  #对应下标的对应数字作为labeldetect下标（这个下标就代表识别出来的标志位置）+1
    maximun=max(labeldetect)                           #找到最大值代表识别出来的最终结果
    labelout=labeldetect.index(maximun)                      #找到对应下标输出
    #del_files("/home/nano/config/teleop/src/yolov5_detect/runs/detect/exp10/labels/")
    return labelout





def blue_line(img):
    """
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    cv2.imshow("corr_img",corr_img)
    hsv=cv2.cvtColor(corr_img,cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv',hsv)
#提取蓝色区域

    blue_lower=np.array([100,50,50])
    blue_upper=np.array([124,255,255])
    blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)
#模糊
    blue_blurred=cv2.blur(blue_mask,(9,9))
    #cv2.imshow('blurred',blue_blurred)
#二值化
    ret,binary=cv2.threshold(blue_blurred,127,255,cv2.THRESH_BINARY)
    cv2.imshow("binary",binary)
    transform_matrix = perspective_transform(src_pts,dst_pts)
    blue_warped_image = birdView(binary*1.0,transform_matrix['M'])
    img_bird = birdView(corr_img,transform_matrix['M'])
    cv2.imshow('blurred binary',blue_warped_image)
    #cv2.imshow("img_bird",img_bird)
    #统计非0像素值的个数，其中输入图像必须是单通道图
    blue_point = cv2.countNonZero(blue_warped_image)
    return blue_point
    """

    blue_line_time=time.time()
    leng=0        
    k=0
    mid_x=0
    mid_y=0
    img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)#去畸变
    img=cv2.resize(img,(96,128))
    sp = img.shape[0:2]  
    h = sp[0]
    w = sp[1]
    image = img[int(2*h/5):int(h),0:int(w)-20]
    # cv2.imshow("image",image)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    # hsv1=hsv.copy()
    #cv2.imshow('hsv',hsv)
#提取蓝色区域

    blue_lower=np.array([100,50,50])
    blue_upper=np.array([124,255,255])
    blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)
    # # cv2.imshow("mask",blue_mask)
    # blue_mask = cv2.GaussianBlur(blue_mask, (7, 7), 7, 7)
    # ret,binary = cv2.threshold(blue_mask,127,255,cv2.THRESH_BINARY)
    # cv2.imshow("blue_mask",blue_mask)
    # cv2.waitKey(50)
    count_blue = len(blue_mask[blue_mask==255])
    if count_blue >2:
        px=[]
        py=[]
        for j in range(blue_mask.shape[1]):
            sj=0
            for i in range(blue_mask.shape[0]):
                if blue_mask[i][j]==255:
                    px.append(j)
                    py.append(i)
                    if sj==0:
                        leng+=1
                        sj=1
        mid_x=np.average(px)
        mid_y=np.average(py)
        k,c0=np.polyfit(px,py,1)
    # print(count_blue,np.arctan(k)*180/np.pi,mid_x,mid_y,leng)

    '''
    ret1,binary1 = cv2.threshold(image,127,255,cv2.THRESH_BINARY)
    blue_target = cv2.bitwise_and(binary1,binary1,mask = blue_mask)
    #cv2.imshow("target",blue_target)
    blue_gray = cv2.cvtColor(blue_target,cv2.COLOR_BGR2GRAY)
    ret,binary = cv2.threshold(blue_gray,127,255,cv2.THRESH_BINARY)
    gray2 = cv2.Canny(binary, 200, 400)
    gray2 = cv2.dilate(gray2, (15, 15))
    gray2 = cv2.erode(gray2, (7, 7))
    cv2.imshow("gray2",gray2)       
    g = gray2[:,:] == 255
    count_blue = len(gray2[g])
    '''
    #count_blue大于1000的时候认为识别到了蓝线
    # print('count_blue',count_blue)
    print('blue_line_time',time.time()-blue_line_time)
    return count_blue,k,mid_x,mid_y,leng
 
def switch (labelout):#yolov5设置的识别路牌从0-4是0停车1右转2左转3直行4调头   ,控制程序中0-4是前右左后停
    if labelout == 0:
        label_out = 4
    elif labelout == 1:
        label_out = 1
    elif labelout == 2:
        label_out =2
    elif labelout == 3:
        label_out = 0
    elif labelout == 4:
        label_out = 3
    return label_out


global leng
leng=0
global turn_right_angle
turn_right_angle=0
def front_camera_callback(data):
   #class_idx, confidence = guide_board(data)
    #print(class_idx,confidence)
    global mode
    global _traffic_sign
    global traffic_sign
    global time_go_setup
    global stop_turn
    global time_turn_setup
    global last_time
    global mode2_time_inte
    global mid_angle
    global  mode1_1
    global time_mode1
    global line_k
    global mid_x
    global mid_y
    global mode1_m
    global e_time
    global leng
    global rearfront
    global time_avo
    global ts3_mode
    global now_signal
    global wanpai
    global turn_right_angle
    
    threshold=100
    time1=time.time()
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    
    if laser_cmd == False:
        if mode == 0:#识别蓝线
            traffic_sign,condif,imid_x,imid_y=mbpp(img)
            blue_point,line_k,mid_x,mid_y,leng = blue_line(img)
            print('blue_point,line_k,mid_x,mid_y,leng,angle',blue_point,line_k,mid_x,mid_y,leng,np.arctan(line_k)*180/np.pi)
            if leng>50 and time.time()-last_time >0:
                print("stop line was detected")

                mode =1  #识别到了蓝线进入mode1
                if traffic_sign==1 and mid_y>20:#右转转弯幅度过小需要提前调整和控制
                    turn_right_angle=np.arctan(line_k)*180/np.pi
                    mode=3
                    time_turn_setup=time.time()
                    msg.drive.steering_angle = -22#以下四个角度都需要调参
                    msg.drive.speed = -20
                    pub.publish(msg)
            stop_turn=False
            so_pub.publish(stop_turn) #stop_turn 为真由本程序控制小车行进，为假由巡线程序控制小车行进
        elif mode == 1:#识别蓝线后调整位姿使其正确通过蓝线
            stop_turn=True
            so_pub.publish(stop_turn)  #交由巡线控制
            blue_point,line_k,mid_x,mid_y,leng = blue_line(img)
            print('blue_point,line_k,mid_x,mid_y,leng,angle',blue_point,line_k,mid_x,mid_y,leng,np.arctan(line_k)*180/np.pi)
            if traffic_sign==1 and mid_y>20:  #同上单独对右转设置条件
                turn_right_angle=np.arctan(line_k)*180/np.pi
                mode=3
                time_turn_setup=time.time()
                msg.drive.steering_angle = -22#以下四个角度都需要调参
                msg.drive.speed = -20
                pub.publish(msg)
            elif leng>50 and mid_y<80:       #超过50开始调整角度
                # if traffic_sign==5 and mid_y>70:
                #     msg.drive.steering_angle = mid_angle
                #     msg.drive.speed = 20
                #     pub.publish(msg)
                #     time_mode1=time.time()
                #     mode=2
                # else:
                msg.drive.steering_angle=mid_angle-(mid_x-48)*0.00-(np.arctan(line_k)*180/np.pi-2.2)*1.5
                print('steering_angle',msg.drive.steering_angle)
                msg.drive.speed = -20
                pub.publish(msg)
            else :
                mode=2
                msg.drive.steering_angle=mid_angle#3.5
                msg.drive.speed = -20
                pub.publish(msg)
                time_mode1=time.time()
        elif mode == 2:#1-3的过渡阶段
            stop_turn=True
            so_pub.publish(stop_turn)  #交由巡线控制
            time_go_sum=time.time()-time_go_setup
            time_mode1_sum=time.time()-time_mode1  #这个mode=2的程序持续时间
            if traffic_sign==0:
                mode=3
                time_turn_setup=time.time()
            elif traffic_sign==1:   #右转
                mode=3
                time_turn_setup=time.time()
                msg.drive.steering_angle = -22#以下四个角度都需要调参
                msg.drive.speed = -20
                pub.publish(msg)
                
            elif traffic_sign==2:   #左转
                msg.drive.steering_angle = mid_angle-(mid_x-48)*0.00-(np.arctan(line_k)*180/np.pi-4)*0.1
                msg.drive.speed = -20
                pub.publish(msg)
                if time_mode1_sum>0.5:
                    mode=3
                    time_turn_setup=time.time()
            elif traffic_sign==3:  #后
                msg.drive.steering_angle = mid_angle
                msg.drive.speed = -20
                pub.publish(msg)
                if time_mode1_sum>3:
                    mode=3
                    time_turn_setup=time.time()
                    # if traffic_sign==3:
                    ts3_mode=0   #掉头中的小程序步骤从0开始
               
            elif traffic_sign==4:
                mode=3
                time_turn_setup=time.time()
            elif traffic_sign==5:  #倒车  
                msg.drive.steering_angle = mid_angle
                msg.drive.speed = 20  #为正向后为负向前
                pub.publish(msg)
                if time_mode1_sum>5:
                    mode=5  #停车入库模式
                    rearfront=1
                    msg.drive.steering_angle=mid_angle
                    msg.drive.speed = -20
                    pub.publish(msg)
        elif mode == 3:#识别交通路牌
            if traffic_sign != 1:
                time_turn_sum=time.time()-time_turn_setup
                traffic_sign,condif,imid_x,imid_y=mbpp(img)
                if traffic_sign== 0:
                
                    msg.drive.steering_angle =mid_angle
                    msg.drive.speed = -20
                    mode = 0
                    # if time_turn_sum> 7.5:#补偿手段
                    #     mode=0
                    #     if now_signal==1:
                    #         last_time=time.time()+5
                    #     elif now_signal==2:
                    #         last_time=time.time()+1.5
                    #     elif now_signal==3:
                    #         last_time=time.time()+2
                    #     elif now_signal==4:
                    #         last_time=time.time()
                    #     elif now_signal==5:
                    #         last_time=time.time()
                    #     msg.drive.steering_angle = mid_angle
                
                elif traffic_sign== 2:
                    msg.drive.steering_angle = 16
                    msg.drive.speed = -20
                    mode=0
                    # if time_turn_sum> 11:   #补偿手段
                    #     mode=0
                    #     if now_signal==1:
                    #         last_time=time.time()+5
                    #     elif now_signal==2:
                    #         last_time=time.time()+1.5
                    #     elif now_signal==3:
                    #         last_time=time.time()+2
                    #     elif now_signal==4:
                    #         last_time=time.time()
                    #     elif now_signal==5:
                    #         last_time=time.time()
                    #     msg.drive.steering_angle = mid_angle
                elif traffic_sign== 3:
                    if time.time()-time_turn_setup>6 and ts3_mode==0:
                        msg.drive.steering_angle = 22
                        msg.drive.speed = -20
                        ts3_mode=1
                        time_turn_setup=time.time()
                    if time.time()-time_turn_setup>4 and ts3_mode==1:
                        msg.drive.steering_angle = -22
                        msg.drive.speed = 20
                        ts3_mode=2
                        time_turn_setup=time.time()
                    if  time.time()-time_turn_setup>5.5 and ts3_mode==2:
                        msg.drive.steering_angle = 18
                        msg.drive.speed = -20
                        ts3_mode=0
                        mode=0
                        last_time=time.time()
                        msg.drive.steering_angle = mid_angle


                    print('msg.drive.speed',msg.drive.speed)
                    # print('ts3_mode',ts3_mode)
                elif traffic_sign== 4:
                    
                    msg.drive.steering_angle = mid_angle
                    msg.drive.speed = -20
                    pub.publish(msg)
                    if time_turn_sum> 1:
                        mode=5
                        rearfront=0

            else :#traffic_sign == 1 右转的时候
                msg.drive.steering_angle = -18#以下四个角度都需要调参
                msg.drive.speed = -20
                mode=0
                print("turn_right_angle",turn_right_angle)
                # if time_turn_sum>5.5:
                #     mode=0
                #     if now_signal==1:#补偿手段
                #         last_time=time.time()+5
                #     elif now_signal==2:
                #         last_time=time.time()+1
                #     elif now_signal==3:
                #         last_time=time.time()+2
                #     elif now_signal==4:
                #         last_time=time.time()
                #     elif now_signal==5:
                #         last_time=time.time()
                #     msg.drive.steering_angle = mid_angle
            print("traffic_sign",traffic_sign)
            pub.publish(msg)
            stop_turn=True
            so_pub.publish(stop_turn)
        elif mode==4 :#识别红绿灯
            
            light_now= light_detection(img)
            print(light_now)
            stop_turn=True
            so_pub.publish(stop_turn)#真有程序控制 假由巡线控制
            msg.drive.steering_angle = mid_angle
            msg.drive.speed = 00
            pub.publish(msg)
            if light_now==1:
                mode=0
                last_time=time.time()+6
                time_avo=time.time()
        elif mode==5:#倒车模式
            stop_turn=True
            so_pub.publish(stop_turn)
        print('msg.drive.steering_angle ',msg.drive.steering_angle )
        time2=time.time()
        print('front_cam_totaltime',time2-time1)
        print("mode=",mode)
            

def decide_sign(sign_class):#汇总模板匹配结果
    #sign_class=mbpp(img)返回的第一个数
    global _traffic_sign#一个过程中存放路标的数组
    global traffic_sign#最终的路标
    global _accumulator

    if sign_class!=10:
        _traffic_sign.append(sign_class)#如果有识别，就放到这个里面
    else:
        _traffic_sign=None

    _accumulator=[0,0,0,0,0,0] 

    if _traffic_sign is not None: 
    # print("in accu") 
        for i in _traffic_sign:
            _accumulator[i]+=1
            #在accumulator中统计各类别的个数
            _traffic_sign=[]#统计完后清除之前识别记录的路标
            traffic_sign=_accumulator.index(max(_accumulator))#取最大值作为路标
            return traffic_sign
    else:
        return None
            



            



    if laser_cmd == False:
        if mode==0 :#识别蓝线
            sign_class,condif,imid_x,imid_y=mbpp(img)
            # sign_class=0
            print('------------------sign:',sign_class)
            if sign_class!=10:
                _traffic_sign.append(sign_class)
            # lane_detection(img)
            blue_point,line_k,mid_x,mid_y,leng = blue_line(img)
            print('blue_point,line_k,mid_x,mid_y,leng,angle',blue_point,line_k,mid_x,mid_y,leng,np.arctan(line_k)*180/np.pi)
            # blue_point=0
    # print('The number of blue pixels',blue_point)
            if leng>50 and time.time()-last_time>0:
            # if False:
                # print('blue_point',blue_point)
                # print('leng',leng)
                # print('blue_line',blue_point,line_k,mid_x,mid_y)
                print("stop line was detected")
                # traffic_sign = yolov5detection()
                traffic_sign_new = yolov5detection()
                traffic_sign = switch(traffic_sign_new)
                # print(_traffic_sign)
                _accumulator=[0,0,0,0,0,0]   #角标顺序是前右左后停
                if _traffic_sign is not None:   
                    # print("in accu")         
                    for i in _traffic_sign:
                        _accumulator[i]+=1
                
                    _traffic_sign=[]
                    if _accumulator is not None:
                        if _accumulator[0]!=0 and _accumulator[4]>2:
                            traffic_sign=4
                        else:
                            traffic_sign=_accumulator.index(max(_accumulator))
                        mode=1
                # else :
                #     traffic_sign=wanpai[now_signal]
                #     mode=1
                # if traffic_sign!=wanpai[now_signal]:
                #     traffic_sign=wanpai[now_signal]
                # now_signal+=1
                print('---------------------------------------------------------------------------------------------------')
                if traffic_sign==1 and mid_y>20:#右转
                    turn_right_angle=np.arctan(line_k)*180/np.pi

                    time_turn_setup=time.time()
                    msg.drive.steering_angle = -22#以下四个角度都需要调参
                    msg.drive.speed = -25
                    pub.publish(msg)
                    time.sleep(4)
                    mode=0
            stop_turn=False
            so_pub.publish(stop_turn)    # 
        elif mode==1:#识别蓝线后调整位姿使其正确通过蓝线
            stop_turn=True
            so_pub.publish(stop_turn)
            
            # elif traffic_sign==4:
            #     mode=5
            #     rearfront=0
            #     time_mode1=time.time()
            # elif traffic_sign== 5:
            #     mode=2
            #     rearfront=1
            #     time_mode1=time.time()
            blue_point,line_k,mid_x,mid_y,leng = blue_line(img)
            print('blue_point,line_k,mid_x,mid_y,leng,angle',blue_point,line_k,mid_x,mid_y,leng,np.arctan(line_k)*180/np.pi)
            # sign_class,condif,imid_x,imid_y=mbpp(img)
            # if sign_class==4:
            #     mode==5
            #     rearfront=0
            #     msg.drive.steering_angle=mid_angle
            #     msg.drive.speed = -20
            #     pub.publish(msg)
            # print()
            if traffic_sign==1 and mid_y>20:
                turn_right_angle=np.arctan(line_k)*180/np.pi
                mode=3
                time_turn_setup=time.time()
                msg.drive.steering_angle = -22#以下四个角度都需要调参
                msg.drive.speed = -20
                pub.publish(msg)
            elif leng>50 and mid_y<80:
                # if traffic_sign==5 and mid_y>70:
                #     msg.drive.steering_angle = mid_angle
                #     msg.drive.speed = 20
                #     pub.publish(msg)
                #     time_mode1=time.time()
                #     mode=2
                # else:
                msg.drive.steering_angle=mid_angle-(mid_x-48)*0.00-(np.arctan(line_k)*180/np.pi-2.2)*1.5
                print('steering_angle',msg.drive.steering_angle)
                msg.drive.speed = -20
                pub.publish(msg)
            else :
                mode=2
                msg.drive.steering_angle=mid_angle#3.5
                msg.drive.speed = -20
                pub.publish(msg)
                time_mode1=time.time()
        elif mode==2:#1-3的过渡阶段
            stop_turn=True
            so_pub.publish(stop_turn)
            time_go_sum=time.time()-time_go_setup
            time_mode1_sum=time.time()-time_mode1
            if traffic_sign==0:
                mode=3
                time_turn_setup=time.time()
            elif traffic_sign==1:
                mode=3
                time_turn_setup=time.time()
                msg.drive.steering_angle = -22#以下四个角度都需要调参
                msg.drive.speed = -20
                pub.publish(msg)
                
            elif traffic_sign==2:
                msg.drive.steering_angle = mid_angle-(mid_x-48)*0.00-(np.arctan(line_k)*180/np.pi-4)*0.1
                msg.drive.speed = -20
                pub.publish(msg)
                if time_mode1_sum>0.5:
                    mode=3
                    time_turn_setup=time.time()
            elif traffic_sign==3:
                msg.drive.steering_angle = mid_angle
                msg.drive.speed = -20
                pub.publish(msg)
                if time_mode1_sum>3:
                    mode=3
                    time_turn_setup=time.time()
                    # if traffic_sign==3:
                    ts3_mode=0
               
            elif traffic_sign==4:
                mode=3
                time_turn_setup=time.time()
            elif traffic_sign==5:
                msg.drive.steering_angle = mid_angle
                msg.drive.speed = 20
                pub.publish(msg)
                if time_mode1_sum>5:
                    mode=5
                    rearfront=1
                    msg.drive.steering_angle=mid_angle
                    msg.drive.speed = -20
                    pub.publish(msg)
                    # time_turn_setup=time.time()
            # elif traffic_sign==3:

           
        
        elif mode==3:#识别交通路牌走
            time_turn_sum=time.time()-time_turn_setup
            traffic_sign_new = yolov5detection()
            traffic_sign = switch(traffic_sign_new)
            # sign_class,condif,imid_x,imid_y=mbpp(img)
            # # sign_class=0
            # print('------------------sign:',sign_class)
            # if sign_class!=10:
            #     _traffic_sign.append(sign_class)
            if traffic_sign== 0:
                
                msg.drive.steering_angle =mid_angle
                msg.drive.speed = -20
                mode = 0
                # if time_turn_sum> 7.5:
                #     mode=0
                #     if now_signal==1:
                #         last_time=time.time()+5
                #     elif now_signal==2:
                #         last_time=time.time()+1.5
                #     elif now_signal==3:
                #         last_time=time.time()+2
                #     elif now_signal==4:
                #         last_time=time.time()
                #     elif now_signal==5:
                #         last_time=time.time()
                #     msg.drive.steering_angle = mid_angle
            elif traffic_sign== 1:
                msg.drive.steering_angle = -18#以下四个角度都需要调参
                msg.drive.speed = -20
                print("turn_right_angle",turn_right_angle)

                # if time_turn_sum>5.5:
                #     mode=0
                #     if now_signal==1:
                #         last_time=time.time()+5
                #     elif now_signal==2:
                #         last_time=time.time()+1
                #     elif now_signal==3:
                #         last_time=time.time()+2
                #     elif now_signal==4:
                #         last_time=time.time()
                #     elif now_signal==5:
                #         last_time=time.time()
                #     msg.drive.steering_angle = mid_angle
            elif traffic_sign== 2:
                msg.drive.steering_angle = 16
                msg.drive.speed = -20
                if time_turn_sum> 11:
                    mode=0
                    if now_signal==1:
                        last_time=time.time()+5
                    elif now_signal==2:
                        last_time=time.time()+1.5
                    elif now_signal==3:
                        last_time=time.time()+2
                    elif now_signal==4:
                        last_time=time.time()
                    elif now_signal==5:
                        last_time=time.time()
                    msg.drive.steering_angle = mid_angle
            elif traffic_sign== 3:
                if time.time()-time_turn_setup>6 and ts3_mode==0:
                    ts3_mode=1
                    time_turn_setup=time.time()
                if time.time()-time_turn_setup>4 and ts3_mode==1:
                    ts3_mode=2
                    time_turn_setup=time.time()
                if  time.time()-time_turn_setup>5.5 and ts3_mode==2:
                    # ts3_mode=0
                    mode=0
                    last_time=time.time()
                    msg.drive.steering_angle = mid_angle

                if ts3_mode==0:
                    msg.drive.steering_angle = 22
                    msg.drive.speed = -20
                    # pub.publish(msg)
                elif ts3_mode==1:
                    msg.drive.steering_angle = -22
                    msg.drive.speed = 20
                    # pub.publish(msg)
                elif ts3_mode==2:
                    msg.drive.steering_angle = 18
                    msg.drive.speed = -20
                    # pub.publish(msg)
                print('msg.drive.speed',msg.drive.speed)
                # print('ts3_mode',ts3_mode)
            elif traffic_sign== 4:
                
                msg.drive.steering_angle = mid_angle
                msg.drive.speed = -20
                pub.publish(msg)
                if time_turn_sum> 1:
                    mode=5
                    rearfront=0
                    
            
            pub.publish(msg)
            stop_turn=True
            so_pub.publish(stop_turn)
            time.sleep(2)
            blue_point,line_k,mid_x,mid_y,leng = blue_line(img)
            if leng >50:
                mode = 2
            else:
                mode = 3


        elif mode==4 :#识别红绿灯
            light_now= light_detection(img)
            print(light_now)
            stop_turn=True
            so_pub.publish(stop_turn)#真有程序控制 假由巡线控制
            msg.drive.steering_angle = mid_angle
            msg.drive.speed = 00
            pub.publish(msg)
            if light_now==1:
                mode=0
                last_time=time.time()+6
                time_avo=time.time()
        elif mode==5:#倒车模式
            stop_turn=True
            so_pub.publish(stop_turn)

        print('msg.drive.steering_angle ',msg.drive.steering_angle )
        time2=time.time()
        print('front_cam_totaltime',time2-time1)
        print("mode=",mode)

def findSquare( image ):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edged = cv2.Canny(blurred, 60, 120)
        # cv2.imshow("edged",edged  )
        # cv2.waitKey(25)

        (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # loop over our contours to find hexagon
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:50]
        screenCnt = None
        for c in cnts:
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.004 * peri, True)
            # if our approximated contour has four points, then
            # we can assume that we have found our squeare

            if len(approx) >= 4:
                screenCnt = approx
                x,y,w,h = cv2.boundingRect(c)
                cv2.drawContours(image, [approx], -1, (0, 0, 255), 1)
                #cv2.imshow("Screen", image)
                #create the mask and remove rest of the background
                mask = np.zeros(image.shape[:2], dtype = "uint8")
                cv2.drawContours(mask, [screenCnt], -1, 255, -1)
                masked = cv2.bitwise_and(image, image, mask = mask)
                cv2.circle(masked, (int(x+w/2), int(y+h/2)), 3, (0, 0, 255), -1)
                # cv2.imshow("Masked",masked  )
                # cv2.waitKey(25)
                #cv2.imwrite('004.jpg',masked)
                #crop the masked image to to be compared to referance image
                #cropped = masked[y:y+h,x:x+w]
                #scale the image so it is fixed size as referance image
                #cropped = cv2.resize(cropped, (200,200), interpolation =cv2.INTER_AREA)

                return masked,x,y,w,h


global time_avo
time_avo=0
global avo
avo=0
global avo_mode
avo_mode=0
global avo_mainmode
avo_mainmode=0

def parking(ls) :
    global mode, msg, pub, so_pub, slpub, mid_angle, signal_line, P_sum, nearest_dist, far_dist, tt, far_dis
    global imid_x
    global imid_y
    global rearfront
    global time_avo
    global stop_turn
    global avo
    global avo_mode
    global laser_cmd
    global avo_mainmode
    global now_signal
    time_star=time.time()
    set_dist=0
    num_in=0
    if mode==0 and time.time()-time_avo>3 and laser_cmd==False and avo_mainmode==0:
        for i in range(720-240,720+240):
            # ang=2*np.pi-(ls.angle_min+i*ls.angle_increment)-np.pi
            ang=ls.angle_min+i*ls.angle_increment-np.pi/2
            # print('ang',ang)
            y=np.sin(ang)*ls.ranges[i]
            x=np.cos(ang)*ls.ranges[i]
            # print('x,y',x,y)
            if abs(x)<0.12 and y<0.7:
                num_in+=1
                # print('x,y',x,y)
        print('num_in',num_in)
        if num_in>20:
            laser_cmd=True
            avo=time.time()
            avo_mode=0
            avo_mainmode=1
    elif laser_cmd==True and avo_mainmode==2:
        nest_dist=min(ls.ranges[0:720])
        nest_idex=ls.ranges.index(nest_dist)
        ang=(ls.angle_min+nest_idex*ls.angle_increment)*180/np.pi
        # print('nest_dist',nest_dist)
        if nest_dist>0.4:
            avo_mainmode=3
            avo_mode=0
            avo=time.time()
            laser_cmd=True
            signal_line=False
            slpub.publish(signal_line)


    if laser_cmd==True and avo_mainmode==1:
        stop_turn=True
        so_pub.publish(stop_turn)
        if avo_mode==0 and time.time()-avo>2.7:
            avo_mode=1
            avo=time.time()
        if avo_mode==1 and time.time()-avo>0:
            laser_cmd=True
            msg.drive.speed = -20
            msg.drive.steering_angle =mid_angle
            pub.publish(msg)
            signal_line=True
            slpub.publish(signal_line)
            avo_mainmode=2
        if avo_mode==0:
            msg.drive.speed = -20
            msg.drive.steering_angle = 22
        elif avo_mode==1:
            msg.drive.speed = -20
            msg.drive.steering_angle = -22
        pub.publish(msg)
        # print('msg.drive.steering_angle ',msg.drive.steering_angle )
        # print('avo_mode',avo_mode) 

    elif laser_cmd==True and avo_mainmode==3:
        stop_turn=True
        so_pub.publish(stop_turn)
        if avo_mode==0 and time.time()-avo>3:
            avo_mode=1
            avo=time.time()
        if avo_mode==1 and time.time()-avo>0:
            laser_cmd=False
            avo_mainmode=0
            msg.drive.speed = -20
            msg.drive.steering_angle =mid_angle
            pub.publish(msg)

        if avo_mode==0:
            msg.drive.speed = -20
            msg.drive.steering_angle =- 22
        elif avo_mode==1:
            msg.drive.speed = -20
            msg.drive.steering_angle = 22
        pub.publish(msg)
        # print('msg.drive.steering_angle ',msg.drive.steering_angle )
        
    
    elif laser_cmd==False:
        if mode==5:
            px=[]
            py=[]
            lines_list=[]
            pxs=[]
            pys=[]
            lines_lists=[]
            lines_list.append(ls.ranges[439])  
            j=0
            if rearfront==1:
                # nest_dist1=min(ls.ranges[0:180])
                nest_dist=min(ls.ranges[1440-180:1440])
                # if nest_dist>nest_dist1:
                #     nest_dist=nest_dist1
                for a in range(440,1000):
                    i=(a+720)%1440
                    ang=ls.angle_min+i*ls.angle_increment-np.pi/2
                    if abs(ls.ranges[i]-lines_list[j])<0.1:
                        py.append(np.sin(ang)*ls.ranges[i])
                        px.append(np.cos(ang)*ls.ranges[i])
                        lines_list.append(ls.ranges[i])
                        j+=1
                        # print(j)
                    else:
                        if len(lines_list)>10 and np.average(lines_list)<2:
                            pxs.append(px)
                            pys.append(py)
                            lines_lists.append(lines_list)
                            # print('nice')
                        px=[]
                        py=[]
                        lines_list=[]
                        lines_list.append(ls.ranges[i])
                        j=0
            else:
                
                nest_dist=min(ls.ranges[680:900])
                for i in range(440,1000):
                    ang=ls.angle_min+i*ls.angle_increment-np.pi/2
                    if abs(ls.ranges[i]-lines_list[j])<0.1:
                        py.append(np.sin(ang)*ls.ranges[i])
                        px.append(np.cos(ang)*ls.ranges[i])
                        lines_list.append(ls.ranges[i])
                        j+=1
                        # print(j)
                    else:
                        if len(lines_list)>10 and np.average(lines_list)<2:
                            pxs.append(px)
                            pys.append(py)
                            lines_lists.append(lines_list)
                            # print('nice')
                        px=[]
                        py=[]
                        lines_list=[]
                        lines_list.append(ls.ranges[i])
                        j=0
            if len(lines_lists)>=2:
                lines_lists_a=sorted(lines_lists,key=len,reverse=True)
                lengths=[]
                for i in range(len(lines_lists)):
                    lengths.append(len(lines_lists[i]))
                if lengths.index( len(lines_lists_a[0]))>lengths.index( len(lines_lists_a[1])):
                    longist=lengths.index( len(lines_lists_a[0]))
                else:
                    longist=lengths.index( len(lines_lists_a[1]))
                px=pxs[longist]
                py=pys[longist]
                lines_list=lines_lists[longist]
                print('lengths',lengths)
                k,c0=np.polyfit(px,py,1)
                linemid_x=np.average(px)
                linemid_y=np.average(py)
                print('k,c0',k,c0)
                # print('k,c0,len,average',k,c0,len(lines_list), np.average(lines_list))
                x1=-(c0/(k+1/k))
                y1=k*x1+c0
                if x1-linemid_x>0:
                    dist=np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
                else :
                    dist=-np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
                # print('dist',dist)
                if rearfront==0:
                    best_angle=(dist+set_dist)*-200
                else:
                    best_angle=(dist-set_dist)*100
                if (best_angle<-45):
                    best_angle=-45
                elif(best_angle>45):
                    best_angle=45
                now_angle=np.arctan(k)*180/np.pi
                if rearfront==0:
                    msg.drive.steering_angle = mid_angle+1*(now_angle-best_angle)
                else:
                    msg.drive.steering_angle = mid_angle-3*(now_angle-best_angle)
                # print('x1,y1',x1,y1)
                # print('linemid_x,linemid_y',linemid_x,linemid_y)
                # print('dist',dist)
                # print('best_angle',best_angle)
                # print('now_angle',now_angle)
                # print('msg.drive.steering_angle',msg.drive.steering_angle)
                
                if rearfront==0:
                    msg.drive.speed = -20
                else:
                    msg.drive.speed = 20
               
            elif len(lines_lists)==1:
                longist=0
                px=pxs[longist]
                py=pys[longist]
                lines_list=lines_lists[longist]
                # print(len(px),len(py))
                k,c0=np.polyfit(px,py,1)
                linemid_x=np.average(px)
                linemid_y=np.average(py)
                # print('px,py,k,c0',px,py,k,c0)
                # print('k,c0,len,average',k,c0,len(lines_list), np.average(lines_list))
                x1=-(c0/(k+1/k))
                y1=k*x1+c0
                if x1-linemid_x>0:
                    dist=np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
                else :
                    dist=-np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
                # print('dist',dist)
                if rearfront==0:
                    best_angle=(dist+set_dist)*-200
                else:
                    best_angle=(dist-set_dist)*100
                if (best_angle<-45):
                    best_angle=-45
                elif(best_angle>45):
                    best_angle=45
                now_angle=np.arctan(k)*180/np.pi
                if rearfront==0:
                    msg.drive.steering_angle = mid_angle+1*(now_angle-best_angle)
                else:
                    msg.drive.steering_angle = mid_angle-3*(now_angle-best_angle)
                # print('x1,y1',x1,y1)
                # print('linemid_x,linemid_y',linemid_x,linemid_y)
                # print('dist',dist)
                # print('best_angle',best_angle)
                # print('now_angle',now_angle)
                # print('msg.drive.steering_angle',msg.drive.steering_angle)
                
                if rearfront==0:
                    msg.drive.speed = -20
                else:
                    msg.drive.speed = 20
            if (nest_dist<0.32 and rearfront==0) or (nest_dist<0.38 and rearfront==1):
                while 1:

                    msg.drive.speed = 0
                    msg.drive.steering_angle = mid_angle
                    pub.publish(msg)
                if nearest_dist<=D1:
                    msg.drive.steering_angle = 35                                   #右转,调整
                    msg.drive.speed = -40
                    pub.publish(msg)
                                                                                                                                                                                                    
                                                                                    
                D2=40 #参数调整                                                              #与P2标志距离D2
                msg.drive.steering_angle =   0
                msg.drive.speed =-20
                pub.publish(msg) 

                if far_dist<=D2:                                                                                               
                    msg.drive.steering_angle =  -35                           #左转,调整
                    msg.drive.speed =-20
                    pub.publish(msg)

            pub.publish(msg)
            # print('totaltime',time.time()-time_star)
            # if flag==0:
            #     px=[]
            #     py=[]
            #     lines_list=[]
            #     pxs=[]
            #     pys=[]
            #     lines_lists=[]
            #     lines_list.append(ls.ranges[439])  
            #     j=0
            

            #     if rearfront==0:
            #         nest_dist1=min(ls.ranges[0:180])
            #         nest_dist=min(ls.ranges[1440-180:1440])
            #         if nest_dist>nest_dist1:
            #             nest_dist=nest_dist1
            #         for a in range(440,1000):
            #             i=(a+720)%1440
            #             ang=ls.angle_min+i*ls.angle_increment
            #             if abs(ls.ranges[i]-lines_list[j])<0.05:
            #                 px.append(-np.sin(ang)*ls.ranges[i])
            #                 py.append(np.cos(ang)*ls.ranges[i])
            #                 lines_list.append(ls.ranges[i])
            #                 j+=1
            #                 # print(j)
            #             else:
            #                 if len(lines_list)>20 and np.average(lines_list)<2.5:
            #                     pxs.append(px)
            #                     pys.append(py)
            #                     lines_lists.append(lines_list)
            #                     # print('nice')
            #                 px=[]
            #                 py=[]
            #                 lines_list=[]
            #                 lines_list.append(ls.ranges[i])
            #                 j=0
            #     else:
                    
            #         nest_dist=min(ls.ranges[540:900])
            #         for i in range(440,1000):
            #             ang=ls.angle_min+i*ls.angle_increment
            #             if abs(ls.ranges[i]-lines_list[j])<0.05:
            #                 px.append(-np.sin(ang)*ls.ranges[i])
            #                 py.append(np.cos(ang)*ls.ranges[i])
            #                 lines_list.append(ls.ranges[i])
            #                 j+=1
            #                 # print(j)
            #             else:
            #                 if len(lines_list)>10 and np.average(lines_list)<2.5:
            #                     pxs.append(px)
            #                     pys.append(py)
            #                     lines_lists.append(lines_list)
            #                     # print('nice')
            #                 px=[]
            #                 py=[]
            #                 lines_list=[]
            #                 lines_list.append(ls.ranges[i])
            #                 j=0
            #     lengths=[]
                
            #     for i in range(len(lines_lists)):
            #         lengths.append(len(lines_lists[i]))
            #     print('lengths',lengths)
            #     longist=lengths.index(max(lengths))
            #     px=pxs[longist]
            #     py=pys[longist]
            #     lines_list=lines_lists[longist]
            #     print(len(px),len(py))
            #     k,c0=np.polyfit(px,py,1)
            #     linemid_x=np.average(px)
            #     linemid_y=np.average(py)
            #     # print('px,py,k,c0',px,py,k,c0)
            #     print('k,c0,len,average',k,c0,len(lines_list), np.average(lines_list))
            #     x1=-(c0/(k+1/k))
            #     y1=k*x1+c0
            #     if x1-linemid_x>0:
            #         dist=np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
            #     else :
            #         dist=-np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
            #     print('dist',dist)
            #     if rearfront==0:
            #         best_angle=dist*-300
            #     else:
            #         best_angle=dist*60
            #     if (best_angle<-45):
            #         best_angle=-45
            #     elif(best_angle>45):
            #         best_angle=45
            #     now_angle=np.arctan(k)*180/np.pi
            #     if rearfront==0:
            #         msg.drive.steering_angle = mid_angle+1*(now_angle-best_angle)
            #     else:
            #         msg.drive.steering_angle = mid_angle-3*(now_angle-best_angle)
                
            #     print('best_angle',best_angle)
            #     print('now_angle',now_angle)
            #     print('msg.drive.steering_angle',msg.drive.steering_angle)
                
            #     if rearfront==0:
            #         msg.drive.speed = -20
            #     else:
            #         msg.drive.speed = 20
            #     if (nest_dist<0.3 and rearfront==0) or (nest_dist<0.4 and rearfront==1):
            #         msg.drive.speed = 0
            #         msg.drive.steering_angle = mid_angle
            #     pub.publish(msg)
            #     print('totaltime',time.time()-time_star)
            #     flag=1
            # # else flag==1:
            #     px=[]
            #     py=[]
            #     lines_list=[]
            #     pxs=[]
            #     pys=[]
            #     lines_lists=[]
            #     lines_list.append(ls.ranges[439])  
            #     j=0
            #     if rearfront==0:
            #         nest_dist1=min(ls.ranges[0:180])
            #         nest_dist=min(ls.ranges[1440-180:1440])
            #         if nest_dist>nest_dist1:
            #             nest_dist=nest_dist1
            #         for a in range(440,1000):
            #             i=(a+720)%1440
            #             ang=ls.angle_min+i*ls.angle_increment
            #             if abs(ls.ranges[i]-lines_list[j])<0.05:
            #                 px.append(-np.sin(ang)*ls.ranges[i])
            #                 py.append(np.cos(ang)*ls.ranges[i])
            #                 lines_list.append(ls.ranges[i])
            #                 j+=1
            #                 # print(j)
            #             else:
            #                 if len(lines_list)>20 and np.average(lines_list)<last_dis and np.average(lines_list)>last_dis-0.2 and angle:
            #                     pxs.append(px)
            #                     pys.append(py)
            #                     lines_lists.append(lines_list)
            #                     # print('nice')
            #                 px=[]
            #                 py=[]
            #                 lines_list=[]
            #                 lines_list.append(ls.ranges[i])
            #                 j=0
            #     else:
            #         # print('nice')
            #         # for i in range(540,900):
            #         #     ang=ls.angle_min+i*ls.angle_increment
            #         #     if abs(ls.ranges[i]-lines_list[j])<0.05:
            #         #         px.append(np.sin(ang)*ls.ranges[i])
            #         #         py.append(np.cos(ang)*ls.ranges[i])
            #         #         lines_list.append(ls.ranges[i])
            #         #         j+=1
            #         #         print(j)
            #         #     elif abs(ls.ranges[i+1]-lines_list[j])<0.05 and abs(ls.ranges[i+2]-lines_list[j])<0.05:
            #         #         if len(lines_list)>10 and np.average(lines_list)<2.0:
            #         #             pxs.append(px)
            #         #             pys.append(py)
            #         #             lines_lists.append(lines_list)
            #         #             print('nice')
            #         #         px=[]
            #         #         py=[]
            #         #         lines_list=[]
            #         #         lines_list.append(ls.ranges[i])
            #         #         j=0
            #         nest_dist=min(ls.ranges[540:900])
            #         for i in range(440,1000):
            #             ang=ls.angle_min+i*ls.angle_increment
            #             if abs(ls.ranges[i]-lines_list[j])<0.05:
            #                 px.append(-np.sin(ang)*ls.ranges[i])
            #                 py.append(np.cos(ang)*ls.ranges[i])
            #                 lines_list.append(ls.ranges[i])
            #                 j+=1
            #                 # print(j)
            #             else:
            #                 if len(lines_list)>10 and np.average(lines_list)<2.5:
            #                     pxs.append(px)
            #                     pys.append(py)
            #                     lines_lists.append(lines_list)
            #                     # print('nice')
            #                 px=[]
            #                 py=[]
            #                 lines_list=[]
            #                 lines_list.append(ls.ranges[i])
            #                 j=0
            #     lengths=[]
                
            #     for i in range(len(lines_lists)):
            #         lengths.append(len(lines_lists[i]))
            #     print('lengths',lengths)
            #     longist=lengths.index(max(lengths))
            #     px=pxs[longist]
            #     py=pys[longist]
            #     lines_list=lines_lists[longist]
            #     print(len(px),len(py))
            #     k,c0=np.polyfit(px,py,1)
            #     linemid_x=np.average(px)
            #     linemid_y=np.average(py)
            #     # print('px,py,k,c0',px,py,k,c0)
            #     print('k,c0,len,average',k,c0,len(lines_list), np.average(lines_list))
            #     x1=-(c0/(k+1/k))
            #     y1=k*x1+c0
            #     if x1-linemid_x>0:
            #         dist=np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
            #     else :
            #         dist=-np.sqrt((x1-linemid_x)*(x1-linemid_x)+(y1-linemid_y)*(y1-linemid_y))
            #     print('dist',dist)
            #     if rearfront==0:
            #         best_angle=dist*-300
            #     else:
            #         best_angle=dist*60
            #     if (best_angle<-45):
            #         best_angle=-45
            #     elif(best_angle>45):
            #         best_angle=45
            #     now_angle=np.arctan(k)*180/np.pi
            #     if rearfront==0:
            #         msg.drive.steering_angle = mid_angle+1*(now_angle-best_angle)
            #     else:
            #         msg.drive.steering_angle = mid_angle-3*(now_angle-best_angle)
                
            #     print('best_angle',best_angle)
            #     print('now_angle',now_angle)
            #     print('msg.drive.steering_angle',msg.drive.steering_angle)
                
            #     if rearfront==0:
            #         msg.drive.speed = -20
            #     else:
            #         msg.drive.speed = 20
            #     if (nest_dist<0.3 and rearfront==0) or (nest_dist<0.4 and rearfront==1):
            #         msg.drive.speed = 0
            #         msg.drive.steering_angle = mid_angle
            #     pub.publish(msg)
            #     print('totaltime',time.time()-time_star)
            # # print('dist,best_angle,now_angle',dist,best_angle,now_angle)
    
    # print('avo_mainmode',avo_mainmode)
    # print('laser_cmd',laser_cmd)
    # print('avo_mode',avo_mode)
    # if mode==6:
    #     if avo_mode==0 and time.time()-time_avo>3:
    #         avo_mode=1
    #         time_avo=time.time()
    #     if avo_mode==1 and time.time()-time_avo>3:
    #         mode=0
    #     if avo_mode==0:
    #         msg.drive.speed = -20
    #         msg.drive.steering_angle = 22
    #     elif avo_mode==1:
    #         msg.drive.speed = -20
    #         msg.drive.steering_angle = -22
        mode = 5

def rear_camera_callback(data):   
    global condif
    global imid_x
    global imid_y
    global mode
    global rearfront
    global _traffic_sign
    if mode==0 or mode==3:
        img = CvBridge().imgmsg_to_cv2(data, "bgr8")
        sign_class,condif,imid_x,imid_y=mbpp(img)  
        if sign_class==4:
            _traffic_sign.append(5)
    pubmode.publish(mode)
    
        
def laser_callback(fot):
    global laser_cmd
    laser_cmd=bool(fot.laser_control)

def detector():

    global pub
    global so_pub
    global slpub
    global pubresult
    global pubmode
    # global Vehicle_PID
    # global Reverse_PID
    rospy.init_node('stop_obj', anonymous=False)
    rospy.Subscriber("/usb_cam_2/image", Image, front_camera_callback, queue_size=1, buff_size=2**24)
    # rospy.Subscriber("/usb_cam_1/image", Image, rear_camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/laser_control", LaserControl, laser_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, parking, queue_size=1)
    pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
    slpub = rospy.Publisher('/signal_line', Bool, queue_size=1)
    so_pub = rospy.Publisher('/stop_turn_cmd', Bool, queue_size=1)
    pubmode=rospy.Publisher('/mode', Int8, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    

    detector()
    
