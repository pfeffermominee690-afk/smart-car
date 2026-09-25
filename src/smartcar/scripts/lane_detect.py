#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
# from laser_test.msg import laser_control
# import jetson.inference
# import jetson.utils
import argparse
import sys

import math
import os
#from skimage import morphology
import time
import warnings

global signal_line
signal_line=False
global stop_turn
stop_turn=False
global time_setup
time_setup=0
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
mode=2
global _traffic_sign
_traffic_sign=[]
global traffic_sign
traffic_sign=0
global c0,c1
c0=0
c1=0
intrinsicMat = np.array([[489.3828, 0.8764, 297.5558],
                            [0, 489.8446, 230.0774],
                            [0, 0, 1]])
distortionCoe = np.array([-0.4119,0.1709,0,0.0011, 0.018])


#src_pts = np.float32([[128,378],[1,435],[639,435],[488,378]]) #[220,306],[1,435],[639,435],[451,306]   [132,358],[1,435],[639,435],[501,353]   [160,334],[38,378],[628,378],[502,334] 
src_pts = np.float32([[1,260],[81,212],[556,212],[638,260]])
dst_pts = np.float32([[100,300],[100,0],[540,0],[540,300]])   #dst_pts = np.float32([[70,0],[70,480],[570,480],[570,0]])
signal_src_pts = np.float32([[24,454],[154,279],[506,281],[638,453]])
signal_dst_pts = np.float32([[0,480],[0,0],[540,0],[540,480]])   #dst_pts = np.float32([[70,0],[70,480],[570,480],[570,0]])
showMe = 0

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
    
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
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


def find_centroid(image,peak_thresh,window,showMe):
    '''
    find centroid in a window using histogram of hotpixels
    img:binary image
    window with specs {'x0','y0','width','height'}
    (x0,y0) coordinates of bottom-left corner of window
    return x-position of centroid ,peak intensity and hotpixels_cnt in window
    '''
    #crop image to window dimension
    mask_window = image[int(window['y0']-window['height']):int(window['y0']),
                        int(window['x0']):int(window['x0']+window['width'])]
    histogram = np.sum(mask_window,axis=0)
    centroid = np.argmax(histogram)
    hotpixels_cnt = np.sum(histogram)
    peak_intensity = histogram[centroid]
    if peak_intensity<=peak_thresh:
        centroid = int(round(window['x0']+window['width']/2))
        peak_intensity = 0
    else:
        centroid = int(round(centroid+window['x0']))
    '''
    if showMe:
        plt.plot(histogram)
        plt.title('Histogram')
        plt.xlabel('horzontal position')
        plt.ylabel('hot pixels count')
        plt.show()
    '''
    return (centroid,peak_intensity,hotpixels_cnt)
def find_starter_centroids(image,y0,peak_thresh,showMe):
    '''
    find starter centroids using histogram
    peak_thresh:if peak intensity is below a threshold use histogram on the full height of the image
    returns x-position of centroid and peak intensity
    '''
    window = {'x0':0,'y0':y0,'width':image.shape[1],'height':image.shape[0]/5}
    # get centroid
    centroid , peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    if peak_intensity<peak_thresh:
        window['height'] = image.shape[0]
        centroid,peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    return {'centroid':centroid,'intensity':peak_intensity}
# if number of histogram pixels in window is below 10,condisder them as noise and does not attempt to get centroid





class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0
    def update(self, feedback_value):     
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain
    def setKi(self, integral_gain):
        self.Ki = integral_gain
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    def setWindup(self, windup):     
        self.windup_guard = windup
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

def compute_radOfCurvature(coeffs,pt):
    return ((1+(2*coeffs['a2']*pt+coeffs['a1'])**2)**1.5)/np.absolute(2*coeffs['a2'])
def binarize(img):
    """Binarize a grayscale image.

    Binarize the input grayscale image by ostu threshold method.

    Args:
        img: an image. Grayscale image is preffered.

    Returns:
       img_binary: the binarized image
    """

    # Make sure that img_gray is a grayscale image.
    if len(img.shape) == 2:
        img_gray = img
    elif len(img.shape) == 3 and img.shape[2] == 3:
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    else:
        #print("Converting image failed:", img.shape)
        return None
    # Apply the threshold method. It can be improved by changing the arguments.
    _, img_binary = cv2.threshold(img_gray, 170, 255, cv.THRESH_OTSU)

    return img_binary

global reset
reset=1
global last_line
last_line=0
global now_line
now_line=0
global last_bx
last_bx=0
global last_angle
last_angle=0
def lane_detection(img):
    global c0,c1 ##################################
    global reset
    global last_line
    global now_line
    global last_bx
    global last_angle
    start_time=time.time()
    # corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    #cv2.imwrite('000.jpg',corr_img)
    # cv2.imshow('corr_img',corr_img)
    # cv2.waitKey(25)
    
    # cv2.imwrite('0.png',corr_img)#保存图片
    gray_ex = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    # display(gray_ex,'Apply Camera Correction',color=0)
    #ret, combined_output = cv2.threshold(gray_ex, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    combined_output = cv2.Canny(gray_ex, 400, 800) #100, 200 75,200
    # cv2.imshow('combined_output',combined_output)
    # cv2.waitKey(25)
    #cv2.imwrite('001.jpg',combined_output)

    #combined_output = image_process(gray_ex)
    # display(combined_output,'Combined output',color=0)
    #mask = np.zeros_like(combined_output)
    #vertices = np.array([[(300,278),(0,435),(640,435),(450,250)]],dtype=np.int32)
    #cv2.fillPoly(mask,vertices,1)
    #masked_image = cv2.bitwise_and(combined_output,mask)
    #display(masked_image,'Masked',color=0)
    
    min_sz = 50
    #cleaned =              morphology.remove_small_objects(masked_image.astype('bool'),min_size=min_sz,connectivity=2)
    cleaned = combined_output
    # display(cleaned,'cleaned',color=0)
    # original image to bird view (transformation)
   
    transform_matrix = perspective_transform(src_pts,dst_pts)
    
    warped_image = birdView(cleaned*1.0,transform_matrix['M'])
    
    warped_image = cv2.dilate(warped_image, np.ones((18,18), np.uint8), 2)
    warped_image = cv2.erode(warped_image, np.ones((10,10), np.uint8))
    #cv2.imwrite('002.jpg',warped_image)
    #pubbrid_view.publish(CvBridge().cv2_to_imgmsg(warped_image))
    
    # display(cleaned,'undistorted',color=0)
    # display(warped_image,'BirdViews',color=0)
    
    #white_Left = cv2.countNonZero(warped_image[:,0:warped_image.shape[1]/2-50])
    #white_Right = cv2.countNonZero(warped_image[:,warped_image.shape[1]/2+50:warped_image.shape[1]])
######mid_time
    mid_time=time.time()
    warped_image = cv2.resize(warped_image,(64*2,48*2))
    HoughLine_image = np.array(warped_image,np.uint8)
    
    lines = cv2.HoughLinesP(HoughLine_image,1,np.pi/180,30,None,35,25)
    if lines is not None :
        x1,y1,x2,y2=lines[0][0]
            # cv2.line(HoughLine_image,(x1,y1),(x2,y2),(255,0,0),1)
########################################################################fit##################################################################################

    bottom_crop = -40
    #warped_image = warped_image[0:bottom_crop,:]
    peak_thresh = 10
    showMe = 1


    # centroid_starter_top = find_starter_centroids(warped_image,y0=warped_image.shape[0],
    #                                            peak_thresh=peak_thresh,showMe=showMe)
    # centroid_starter_bottom = find_starter_centroids(warped_image,y0=warped_image.shape[0]/5,
    #                                            peak_thresh=peak_thresh,showMe=showMe)
    # print('centroid_starter_TOP',centroid_starter_top['centroid'])
    # print('centroid_starter_BOTTOM',centroid_starter_bottom['centroid'])


  
    ''' 
    centroid_starter_right = find_starter_centroids(warped_image,x0=warped_image.shape[1]/2,
                                               peak_thresh=peak_thresh,showMe=showMe)
    centroid_starter_left = find_starter_centroids(warped_image,x0=0,peak_thresh=peak_thresh,
                                              showMe=showMe)
    sliding_window_specs = {'width': 60, 'n_steps': 10}
    print('centroid_starter_right',centroid_starter_right)
    #cv2.imshow('warped_image',warped_image)
    #cv2.waitKey(25)

####window#####
   
    '''

    MD_thresh = 1.8
    #log_lineLeft['x'], log_lineLeft['y'] = \
    #MD_removeOutliers(log_lineLeft['x'], log_lineLeft['y'], MD_thresh)
    #log_lineRight['x'], log_lineRight['y'] = \
    #MD_removeOutliers(log_lineRight['x'], log_lineRight['y'], MD_thresh)
    
    ym_per_pix = 0.6/480
    xm_per_pix = 0.6/640

    ###log_lineLeft,out_img_Left = run_sliding_window(warped_image, centroid_starter_left['centroid'],sliding_window_specs, showMe=showMe)
    ###log_lineRight,out_img = run_sliding_window(out_img_Left, centroid_starter_right['centroid'] , sliding_window_specs,showMe=showMe)

    

    if lines is not None :
        
        if x1==x2:
            x1=x1+0.1
        c1=(y1-y2)/(x1-x2)
        c0=y1-c1*x1
        
            
        print('HoughLine is detected') 
        print('c1',c1)  
    cv2.imshow('warped_image',warped_image)
    cv2.waitKey(30)
    
    ''' 
    for i in range(len(log_lineRight['y'])):
        log_lineRight['y'][i] = log_lineRight['y'][i] + out_img.shape[1]/2
    
    #log_lineRight['x'] = log_lineRight['x'] + out_img.shape[1]/2


    fit_lineRight_singleframe = polynomial_fit(log_lineRight)
    fit_lineLeft_singleframe = polynomial_fit(log_lineLeft)
    dis_Left = log_lineLeft['y'][len(log_lineLeft['y'])-1] - log_lineLeft['y'][0]
    dis_Right = log_lineRight['y'][len(log_lineRight['y'])-1] - log_lineRight['y'][0]
    var_pts = np.linspace(0,corr_img.shape[0]-1,num=corr_img.shape[0])
    pred_lineLeft_singleframe = predict_line(0,corr_img.shape[0],fit_lineLeft_singleframe)
    fit_lineLeft_real = polynomial_fit({'x':[i*xm_per_pix for i in log_lineLeft['x']],
                                    'y':[i*ym_per_pix for i in log_lineLeft['y']]})
    pred_lineRight_sigleframe = predict_line(0,corr_img.shape[0],fit_lineRight_singleframe)
    fit_lineRight_real = polynomial_fit({'x':[i*xm_per_pix for i in log_lineRight['x']],
                                               'y':[i*ym_per_pix for i in log_lineRight['y']]})
   
    pt_curvature = corr_img.shape[0]
    radOfCurv_r = compute_radOfCurvature(fit_lineRight_real,pt_curvature*ym_per_pix)
    radOfCurv_l = compute_radOfCurvature(fit_lineLeft_real,pt_curvature*ym_per_pix)
    average_radCurv = (radOfCurv_r+radOfCurv_l)/2
    
    center_of_lane = (pred_lineLeft_singleframe[:,1][-1]+pred_lineRight_sigleframe[:,1][-1])/2
    offset = (corr_img.shape[1]/2 - center_of_lane)*xm_per_pix

    side_pos = 'right'
    if offset <0:
        side_pos = 'left'
    wrap_zero = np.zeros_like(gray_ex).astype(np.uint8)
    color_wrap = np.dstack((wrap_zero,wrap_zero,wrap_zero))
    left_fitx = fit_lineLeft_singleframe['a2']*var_pts**2 + fit_lineLeft_singleframe['a1']*var_pts + fit_lineLeft_singleframe['a0']
    right_fitx = fit_lineRight_singleframe['a2']*var_pts**2 +     fit_lineRight_singleframe['a1']*var_pts+fit_lineRight_singleframe['a0']
    pts_left = np.array([np.transpose(np.vstack([left_fitx,var_pts]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx,var_pts])))])
    pts = np.hstack((pts_left,pts_right))
    cv2.fillPoly(color_wrap,np.int_([pts]),(0,255,0))
    cv2.putText(color_wrap,'|',(int(corr_img.shape[1]/2),corr_img.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,255),8)
    cv2.putText(color_wrap,'|',(int(center_of_lane),corr_img.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),8)
    newwrap = cv2.warpPerspective(color_wrap,transform_matrix['Minv'],(corr_img.shape[1],corr_img.shape[0])) 
    result = cv2.addWeighted(corr_img,1,newwrap,0.3,0)
    cv2.putText(result,'Vehicle is ' + str(round(offset,3))+'m '+side_pos+' of center',
            (50,100),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),thickness=2)
    cv2.putText(result,'Radius of curvature: '+str(round(average_radCurv,3))+'m',(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),thickness=2)
   # cv2.imshow("result",result)
    #cv2.waitKey(25)
    '''
    end_time=time.time()

    time_diff1=mid_time-start_time
    time_diff2=end_time-mid_time
    # print('time_diff1',time_diff1)
    # print('time_diff2',time_diff2)
    #print('offset ', offset)
    msg.drive.speed =-20
    
    #print('dis_Left',dis_Left)
    #print('dis_Right',dis_Right)
#     if  math.fabs(centroid_starter_top['centroid']-centroid_starter_bottom['centroid'])<10:             #origin 80
#   ##((dis_Left < 90)&(dis_Left > 30) & (dis_Right > -10 )&(dis_Right < 70 )) :#四个dis的阈值都需要调参
#         print('both of the lanes are detected')
#         offset=centroid_starter_top['centroid']-centroid_starter_bottom['centroid']
#         Vehicle_PID.update(offset)
#         msg.drive.steering_angle = Vehicle_PID.output###正负未测试
        
#     else :
    print('only single lane is detected')
    k1 = 0#50  45# k1,k2 需要调参
    k2 = 0.5#0.02  0.035
#####need to adjust para####
    if c1 != 0:
        bx=(96-c0)/c1
    else:
        return
    line_angle=np.arctan(c1)*180/np.pi
    if line_angle>0:
        line_angle=90-line_angle
    else :
        line_angle=-90-line_angle
    print('line_angle',line_angle)
    rl_s=10
    if bx<64:
        now_line=0
        msg.drive.steering_angle =  line_angle*k1 - k2*(96- c0)/c1+3.5
        print('Left Lane')           
    else:
        now_line=1
        msg.drive.steering_angle =  line_angle*k1 + k2*(128-(96-c0)/c1)+3.5
        print('Right Lane')     

    if reset==0:
        if now_line!=last_line:
            if (last_bx-bx<40 and now_line==0) or (bx-last_bx<40 and now_line==1):
                now_line=last_line
                if now_line==0:
                    msg.drive.steering_angle =  line_angle*k1 - k2*bx+3.5
                elif now_line==1:
                    msg.drive.steering_angle =  line_angle*k1 + k2*(128-bx)+3.5
        elif abs(bx-last_bx)>40:
                bx=last_bx
                line_angle=last_angle
                if now_line==0:
                    msg.drive.steering_angle =  line_angle*k1 - k2*bx+3.5
                elif now_line==1:
                    msg.drive.steering_angle =  line_angle*k1 + k2*(128-bx)+3.5
    else :
        reset=0
    print('now_line',now_line)
    last_line=now_line
    last_bx=bx 
    last_angle=line_angle
    print('steering_angle',msg.drive.steering_angle)

global last_bx_signal
last_bx_signal=0
def signal_lane_detection(img):
    global c0,c1 ##################################
    global last_bx_signal
    start_time=time.time()
    # gray_ex = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    # combined_output = cv2.Canny(gray_ex, 400, 800) #100, 200 75,200
    
    gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    # cv2.imshow('gray_ex',gray)
    # cv2.waitKey(30)
    gray = cv2.GaussianBlur(gray, (5, 5), 2, 2)
    # cv2.imshow('gray',gray)
    # cv2.waitKey(30)
    combined_output = cv2.Canny(gray, 120, 300,5) #100, 200 75,200
    # cv2.imshow('combined_output',combined_output)
    # cv2.waitKey(30)
    # min_sz = 50
    cleaned = combined_output
   
    transform_matrix = perspective_transform(signal_src_pts,signal_dst_pts)
    warped_image = birdView(cleaned*1.0,transform_matrix['M'])
    
    # warped_image = cv2.dilate(warped_image, np.ones((35,35), np.uint8), 2)
    # warped_image = cv2.erode(warped_image, np.ones((10,10), np.uint8))
    # warped_image = cv2.dilate(warped_image, np.ones((50,50), np.uint8), 2)
    # warped_image = cv2.erode(warped_image, np.ones((62,62), np.uint8))
    # warped_image = cv2.dilate(warped_image, np.ones((15,15), np.uint8), 2)
    mid_time=time.time()
    warped_image = cv2.resize(warped_image,(64*2,48*2))
    HoughLine_image = np.array(warped_image,np.uint8)
    
    # lines = cv2.HoughLinesP(HoughLine_image,1,np.pi/180,30,None,35,48)
    # if lines is not None :
    #     x1,y1,x2,y2=lines[0][0]

    # if lines is not None :
    #     if x1==x2:
    #         x1=x1+0.1
    #     c1=(y1-y2)/(x1-x2)
    #     c0=y1-c1*x1
        
            
    #     print('HoughLine is detected') 
    #     print('c1',c1)  
    sum_cen=0
    sum_size=0
    for i in range(warped_image.shape[0]):
        for j in range(warped_image.shape[1]):
            if warped_image[i][j]>0:
                sum_size+=1
                sum_cen+=j
    print('sum_size',sum_size)
    if sum_size>10:
        bx=sum_cen/sum_size-64
    else:
        bx=last_bx_signal
    cv2.imshow('warped_image',warped_image)
    cv2.waitKey(30)
    
    
    # end_time=time.time()

    # time_diff1=mid_time-start_time
    # time_diff2=end_time-mid_time
    msg.drive.speed =-20
    
    print('only single lane is detected')
    k1 = 0#50  45# k1,k2 需要调参
    k2 = 0.5#0.02  0.035
#####need to adjust para####
    # bx=(96-c0)/c1-48
    # line_angle=np.arctan(c1)*180/np.pi
    # if line_angle>0:
    #     line_angle=90-line_angle
    # else :
    #     line_angle=-90-line_angle
    # print('line_angle',line_angle)
    print('bs',bx)
    msg.drive.steering_angle =-k2*bx+3.5  
    last_bx_signal=bx
    
    print('steering_angle',msg.drive.steering_angle)



def front_camera_callback(data):
   #class_idx, confidence = guide_board(data)
    #print(class_idx,confidence)
    global mode
    global _traffic_sign
    global traffic_sign
    global time_setup
    global stop_turn
    global laser_cmd
    global reset
    threshold=500
    time1=time.time()
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    # sign=2
    # if(sign == 0):
    #     msg.drive.steering_angle = 0
    #     msg.drive.speed = -20
    # if(sign == 1):
    #     msg.drive.steering_angle = 11#以下四个角度都需要调参
    #     msg.drive.speed = -20
    # if(sign == 2):
    #     msg.drive.steering_angle = -200
    #     msg.drive.speed = -20
    # if(sign == 3):
    #     msg.drive.steering_angle = 0
    #     msg.drive.speed = 00
        
    # if(sign == 4):
    #     msg.drive.steering_angle = 11
    #     msg.drive.speed = -20
    # pub.publish(msg)
    # print(msg.drive.steering_angle,msg.drive.speed)
    # print(laser_cmd)
    
    if signal_line==False:
        lane_detection(img)
    if laser_cmd == False and stop_turn==False:
        pub.publish(msg)   
    elif signal_line==True:
        print("sigal_line")
        signal_lane_detection(img)
        pub.publish(msg)   
        reset=1
    else :
        reset=1
    # signal_lane_detection(img)
    # pub.publish(msg)   
    # if laser_cmd == 0:
    #     if(n>=1):
    #         traffic_sign_detection(img)
    #     else:
    #         if(1):#light_detection(img)
                
    # 	        n=n+1
    #         else:
    #             msg.drive.speed = 0
    #             pub.publish(msg)
    #             n = n
    # lane_detection(img)  
    time2=time.time()
    print('lane_detection_totaltime',time2-time1)
def findSquare( image ):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edged = cv2.Canny(blurred, 60, 120)
        cv2.imshow("edged",edged  )
        cv2.waitKey(25)
        # find contours in the edge map
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
                cv2.imshow("Masked",masked  )
                cv2.waitKey(25)
                #cv2.imwrite('004.jpg',masked)
                #crop the masked image to to be compared to referance image
                #cropped = masked[y:y+h,x:x+w]
                #scale the image so it is fixed size as referance image
                #cropped = cv2.resize(cropped, (200,200), interpolation =cv2.INTER_AREA)

                return masked,x,y,w,h

def rear_camera_callback(data):   
    global flag3 
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    #cv2.imwrite('005.jpg',corr_img)
    down_img = corr_img[:, corr_img.shape[1]/2:corr_img.shape[1]]
    masked,x,y,w,h = findSquare(down_img)
    #k1,k2,k3,k4,k5,k6=turn(img)
    cv2.waitKey(25)
    dis = int(-1.05*w)##dis需要调参
    cv2.circle(corr_img, (int(x+w/2+corr_img.shape[1]/2), int(y+h/2)), 3, (0, 255, 255), -1)
    cv2.circle(corr_img, (int(x+w/2+dis+corr_img.shape[1]/2), int(y+h/2)), 3, (0, 0, 255), -1)
    cv2.circle(corr_img, (corr_img.shape[1]/2, int(y+h/2)), 3, (255, 0, 0), -1)
    cv2.imshow('result',corr_img)  
    #cv2.imwrite('003.jpg',corr_img)
    print('w= ',w,'h= ',h)
			
    if laser_cmd == 0:        
        msg.drive.speed = -20				
        if (w <180) & (flag3 == 0):#w<180需要调参
       
            offset = - (dis + x+w/2)
            print('offset',offset)
            Reverse_PID.update(offset)
            msg.drive.steering_angle = Reverse_PID.output
        else:
            msg.drive.steering_angle =   0
            flag3 = 1
            '''
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            edged = cv2.Canny(gray, 400, 800)
            transform_matrix = perspective_transform(src_pts,dst_pts)
            warped_image = birdView(edged*1.0,transform_matrix['M']) 
            element1 = cv2.getStructuringElement(0,(2,80)) 
            element2 = cv2.getStructuringElement(0,(2,2)) 
            eroded = cv2.erode(warped_image,element1)  
            dilated = cv2.dilate(warped_image,element2)    
            HoughLine_image = np.array(dilated,np.uint8)
            lines = cv2.HoughLinesP(HoughLine_image,1,np.pi/180,100,100,100,50)
            if lines is not None :
                for x1,y1,x2,y2 in lines[0]:
                    cv2.line(HoughLine_image,(x1,y1),(x2,y2),(255,0,0),1)
            for i in range(HoughLine_image.shape[1]/5):
                x = 5*i
                px = [x1,x2]
                py = [y1,y2]
                c1,c0 = np.polyfit(px, py, 1)
                y = int(c1*x + c0)
                cv2.circle(HoughLine_image, (x, y), 3, (255, 0, 0), -1)
            cv2.imshow("HoughLine_image",HoughLine_image )
            cv2.waitKey(25)
        
            k1 = 55
            k2 = 0.02
            if (480-c0)/c1<320:
                msg.drive.steering_angle = - 1/c1*k1 + k2*(480- c0)/c1                                  
            if (480-c0)/c1>320:
                msg.drive.steering_angle = - 1/c1*k1 - k2*(640-(480-c0)/c1) 
           
            '''
   
    else :
        msg.drive.speed = 0
        msg.drive.steering_angle = 0
        flag3 = 0
    print('The speed is ',msg.drive.speed)
    print('The steering angle is ',msg.drive.steering_angle)
    pub.publish(msg)
def stop_turn_callback(fot):
    global stop_turn
    stop_turn=fot.data
    print('stop_turn',stop_turn)

def laser_callback(fot):
    global laser_cmd
    laser_cmd=fot.data
    print('laser_cmd',laser_cmd)

def signal_line_callback(fot):
    global signal_line
    signal_line=fot.data
    print('signal_line_____________________--------------------------------',signal_line)

def detector():

    global pub
    global pubresult
    global Vehicle_PID
    global Reverse_PID
    Vehicle_PID = PID(0.05,0,0)#PID需要调参(2,0,0)
    Reverse_PID = PID(0.25,0,0)#PID需要调参
    rospy.init_node('lane_detec', anonymous=False)
    rospy.Subscriber("/usb_cam_2/image", Image, front_camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/stop_turn_cmd',Bool, stop_turn_callback, queue_size=1)
    rospy.Subscriber('/signal_line',Bool, signal_line_callback, queue_size=1)
    # rospy.Subscriber("/usb_cam_1/image", Image, rear_camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/laser_control", Bool, laser_callback, queue_size=1)
    pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
    warnings.simplefilter('ignore', np.RankWarning)
    rospy.spin()

if __name__ == '__main__':
    

    detector()
    


