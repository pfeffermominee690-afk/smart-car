#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from laser_test.msg import laser_control
import tensorflow as tf
import math
import os
import threading
#from skimage import morphology
import time
global x0 
x0 = 1
global peak_thresh
peak_thresh = 50 
global n 
n = 0
global laser_cmd
laser_cmd = 0
# Front camera calibration for /usb_cam_2/image at 640x480.
# Source: smartcar-data/calibration/session-20260925-203626-24mm-reprocessed-a4dfcb/
#         front/results/20260925-204413-7be6b392/camera.yaml
intrinsicMat = np.array([[444.92179141947196, 0.0, 337.9399577215935],
                         [0.0, 443.59684870127415, 285.85005027454673],
                         [0.0, 0.0, 1.0]])
distortionCoe = np.array([-0.41251110946304526,
                           0.3572865990594976,
                           0.0010943780467337042,
                           0.0007363828658040413,
                          -0.2652600879338458])

showMe = 0
show_windows = True

# The bird-view destination maps the 60 cm lane to x=70..570.
# Follow only the solid right boundary and infer the lane centre 30 cm left of it.
LANE_WIDTH_PX = 500.0
HALF_LANE_WIDTH_PX = LANE_WIDTH_PX / 2.0
RIGHT_EXPECTED_X = 570
RIGHT_MIN_PIXELS = 300
RIGHT_MIN_VERTICAL_BINS = 6
RIGHT_MAX_BOTTOM_JUMP_PX = 70.0
RIGHT_MAX_FAR_JUMP_PX = 90.0
RIGHT_HOLD_FRAMES = 1
RIGHT_SMOOTHING_ALPHA = 0.75
right_lane_tracker = {'fit': None, 'lost_frames': 0}
debug_images = {}
debug_images_lock = threading.Lock()


def show_cv_window(title, image):
    """Queue the newest debug frame; the ROS main thread displays it."""
    if show_windows and title == 'result':
        with debug_images_lock:
            debug_images[title] = image.copy()


def run_debug_window_loop():
    """Run OpenCV HighGUI on the main thread to avoid callback deadlocks."""
    cv2.namedWindow('result', cv2.WINDOW_NORMAL)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        with debug_images_lock:
            frames = [('result', debug_images.get('result'))]
        for title, frame in frames:
            if frame is not None:
                cv2.imshow(title, frame)
        cv2.waitKey(1)
        rate.sleep()
    cv2.destroyAllWindows()


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
def light_detection(origin_img):      
    light_cmd=0               
    hsv=cv2.cvtColor(origin_img,cv2.COLOR_BGR2HSV)   #转化为HSV格式图像，更好得处理光线对图像的影响。（由于处理器处理能力限制那个1w的摄像头没法用，读入的图像只有5帧以内，调节不及时）
    hsv1 = hsv.copy()
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    red_lower = np.array([0,95,230])        #这两个是阈值，红灯的上界和下界
    red_upper = np.array([5,255,255])
    red_mask = cv2.inRange(hsv,red_lower,red_upper)
    red_target = cv2.bitwise_and(hsv,hsv,mask = red_mask)
    red_target = cv2.erode(red_target,element)
    red_target = cv2.dilate(red_target,element)
    red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)
    r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
    r_gray2 = cv2.Canny(r_binary, 100, 200) 
    r = r_gray2[:,:] == 255
    count_red = len(r_gray2[r])
    if count_red>700:
        redLight = 1
    else:
        redLight = 0
    green_lower = np.array([80,95,230])    #这个阈值有问题,在寝室调不出来
    green_upper = np.array([130,255,255])
    green_mask = cv2.inRange(hsv1,green_lower,green_upper)
    green_target = cv2.bitwise_and(hsv1,hsv1,mask = green_mask)
    green_target = cv2.erode(green_target,element)
    green_target = cv2.dilate(green_target,element)
    green_gray = cv2.cvtColor(green_target,cv2.COLOR_BGR2GRAY)
    g_ret,g_binary = cv2.threshold(green_mask,127,255,cv2.THRESH_BINARY)
    g_gray2 = cv2.Canny(g_binary, 100, 200)       
    g = g_gray2[:,:] == 255
    count_green = len(g_gray2[g])
    if count_green>700:
        greenLight = 1
    else:
        greenLight = 0
    if redLight ==1 :
        light_cmd = 0
    if greenLight == 1 :
        light_cmd = 1
    return light_cmd

def birdView(img,M):
    '''
    Transform image to birdeye view
    img:binary image
    M:transformation matrix
    return a wraped image
    '''
    img_sz = (img.shape[1],img.shape[0])
    img_warped = cv2.warpPerspective(img,M,img_sz,flags = cv2.INTER_LINEAR)
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
    # This value is used as a density ratio by the sliding-window tracker.
    # Count pixels instead of summing their 0/255 intensities.
    hotpixels_cnt = np.count_nonzero(mask_window)
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
def find_starter_centroids(image,x0,peak_thresh,showMe):
    '''
    find starter centroids using histogram
    peak_thresh:if peak intensity is below a threshold use histogram on the full height of the image
    returns x-position of centroid and peak intensity
    '''
    window = {'x0':x0,'y0':image.shape[0],'width':image.shape[1]/2,'height':image.shape[0]/2}
    # get centroid
    centroid , peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    if peak_intensity<peak_thresh:
        window['height'] = image.shape[0]
        centroid,peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    return {'centroid':centroid,'intensity':peak_intensity}
# if number of histogram pixels in window is below 10,condisder them as noise and does not attempt to get centroid


def find_right_starter(image, peak_thresh):
    """Find the solid right boundary near its expected bird-view position."""
    height, width = image.shape[:2]
    x_min = width / 2
    x_max = min(width, int(width * 0.98))
    histogram = np.sum(image[height / 2:height, x_min:x_max], axis=0)
    if histogram.size == 0 or np.max(histogram) <= peak_thresh:
        histogram = np.sum(image[:, x_min:x_max], axis=0)
    if histogram.size == 0 or np.max(histogram) <= peak_thresh:
        return {'centroid': RIGHT_EXPECTED_X, 'intensity': 0}

    # Pick the strong response closest to the expected right-lane position.
    strong_threshold = max(float(peak_thresh), float(np.max(histogram)) * 0.35)
    candidates = np.where(histogram >= strong_threshold)[0] + x_min
    centroid = int(candidates[np.argmin(np.abs(candidates - RIGHT_EXPECTED_X))])
    return {'centroid': centroid,
            'intensity': histogram[centroid - x_min]}



def run_sliding_window(image, centroid_starter, sliding_window_specs, showMe=showMe):
    '''
    Run sliding window from bottom to top of the image and return indexes of the hotpixels associated with lane
    image:binary image
    centroid_starter:centroid starting location sliding window
    sliding_window_specs:['width','n_steps']
        width of sliding window
        number of steps of sliding window alog vertical axis
    return {'x':[],'y':[]}
        coordiantes of all hotpixels detected by sliding window
        coordinates of alll centroids recorded but not used yet!
    '''
    # Initialize sliding window
    '''
    result = image
    if Left_or_Right == 0:
        image = image[:,0:image.shape[1]/2]
    if Left_or_Right == 1:
        image = image[:,image.shape[1]/2:image.shape[1]]
    '''
    window = {'x0': centroid_starter - int(sliding_window_specs['width'] / 2),
              'y0': image.shape[0], 'width': sliding_window_specs['width'],
              'height': round(image.shape[0] / sliding_window_specs['n_steps'])}
    hotpixels_log = {'x': [], 'y': []}
    centroids_log = []
    out_img = image
    if showMe:
        out_img = (np.dstack((image, image, image)) * 255).astype('uint8')
    for step in range(sliding_window_specs['n_steps']):
        if window['x0'] < 0: window['x0'] = 0
        if (window['x0'] + sliding_window_specs['width']) > image.shape[1]:
            window['x0'] = image.shape[1] - sliding_window_specs['width']
        centroid, peak_intensity, hotpixels_cnt = find_centroid(image, peak_thresh, window, showMe=0)
        if step == 0:
            starter_centroid = centroid
        if hotpixels_cnt / (window['width'] * window['height']) > 0.6:
            window['width'] = window['width'] * 2
            window['x0'] = round(window['x0'] - window['width'] / 2)
            if (window['x0'] < 0): window['x0'] = 0
            if (window['x0'] + window['width']) > image.shape[1]:
                window['x0'] = image.shape[1] - window['width']
            centroid, peak_intensity, hotpixels_cnt = find_centroid(image, peak_thresh, window, showMe=0)

            # if showMe:
            # print('peak intensity{}'.format(peak_intensity))
            # print('This is centroid:{}'.format(centroid))
        mask_window = np.zeros_like(image)
        mask_window[int(window['y0'] - window['height']):int(window['y0']), int(window['x0']):int(window['x0'] + window['width'])] = image[int(window['y0'] - window['height']):int(window['y0']), int(window['x0']):int(window['x0'] + window['width'])]

        hotpixels = np.nonzero(mask_window)
        # print(hotpixels_log['x'])

        hotpixels_log['x'].extend(hotpixels[0].tolist())
        hotpixels_log['y'].extend(hotpixels[1].tolist())
        # update record of centroid
        centroids_log.append(centroid)
        if showMe:
            out_img = cv2.rectangle(
                out_img,
                (int(window['x0']), int(window['y0'] - window['height'])),
                (int(window['x0'] + window['width']), int(window['y0'])),
                (255, 0, 0), 2)
        
        ''' 
        if Left_or_Right == 0:
            result[0:image.shape[0],0:image.shape[1]] = out_img
        if Left_or_Right == 1:
            result[0:image.shape[0],image.shape[1]:image.shape[1]*2] = out_img
            
       
        
        if showMe:
            cv2.rectangle(out_img,
                          (int(window['x0']), int(window['y0'] - window['height'])),
                          (int(window['x0'] + window['width']), int(window['y0'])), (0, 255, 0), 2)

            if step == 9:
                plt.imshow(out_img)
                plt.show()
            
            print(window['y0'])
            plt.imshow(out_img)
         
        '''
        # set next position of window and use standard sliding window width
        window['width'] = sliding_window_specs['width']
        window['x0'] = round(centroid - window['width'] / 2)
        window['y0'] = window['y0'] - window['height']
    return hotpixels_log, out_img




def MahalanobisDist(x, y):
    '''
    Mahalanobis Distance for bi-variate distribution

    '''
    covariance_xy = np.cov(x, y, rowvar=0)
    inv_covariance_xy = np.linalg.inv(covariance_xy)
    xy_mean = np.mean(x), np.mean(y)
    x_diff = np.array([x_i - xy_mean[0] for x_i in x])
    y_diff = np.array([y_i - xy_mean[1] for y_i in y])
    diff_xy = np.transpose([x_diff, y_diff])

    md = []
    for i in range(len(diff_xy)):
        md.append(np.sqrt(np.dot(np.dot(np.transpose(diff_xy[i]), inv_covariance_xy), diff_xy[i])))
    return md


def MD_removeOutliers(x, y, MD_thresh):
    '''
    remove pixels outliers using Mahalonobis distance
    '''
    MD = MahalanobisDist(x, y)
    threshold = np.mean(MD) * MD_thresh
    nx, ny, outliers = [], [], []
    for i in range(len(MD)):
        if MD[i] <= threshold:
            nx.append(x[i])
            ny.append(y[i])
        else:
            outliers.append(i)
    return (nx, ny)




def update_tracker(tracker,new_value):
    '''
    update tracker(self.bestfit or self.bestfit_real or radO Curv or hotpixels) with new coeffs
    new_coeffs is of the form {'a2':[val2,...],'a1':[va'1,...],'a0':[val0,...]}
    tracker is of the form {'a2':[val2,...]}
    update tracker of radius of curvature
    update allx and ally with hotpixels coordinates from last sliding window
    '''
    allx = []
    ally = []
    if tracker =='bestfit':
        bestfit['a0'].append(new_value['a0'])
        bestfit['a1'].append(new_value['a1'])
        bestfit['a2'].append(new_value['a2'])
    elif tracker == 'bestfit_real':
        bestfit_real['a0'].append(new_value['a0'])
        bestfit_real['a1'].append(new_value['a1'])
        bestfit_real['a2'].append(new_value['a2'])
    elif tracker == 'radOfCurvature':
        radOfCurv_tracker.append(new_value)
    elif tracker == 'hotpixels':
        allx.append(new_value['x'])
        ally.append(new_value['y'])

def polynomial_fit(data):
    '''
    多项式拟合
    a0+a1 x+a2 x**2
    data:dictionary with x and y values{'x':[],'y':[]}
    '''
    a2,a1,a0 = np.polyfit(data['x'],data['y'],2)
    return {'a0':a0,'a1':a1,'a2':a2}


def predict_line(x0,xmax,coeffs):
    '''
    predict road line using polyfit cofficient
    x vaues are in range (x0,xmax)
    polyfit coeffs:{'a2':,'a1':,'a2':}
    returns array of [x,y] predicted points ,x along image vertical / y along image horizontal direction
    '''
    x_pts = np.linspace(x0,xmax-1,num=xmax)
    pred = coeffs['a2']*x_pts**2+coeffs['a1']*x_pts+coeffs['a0']
    return np.column_stack((x_pts,pred))


def evaluate_fit(coeffs, row):
    return coeffs['a2'] * row ** 2 + coeffs['a1'] * row + coeffs['a0']


def hold_previous_right_fit(reason):
    """Reject a noisy frame and briefly keep the last trusted right line."""
    right_lane_tracker['lost_frames'] += 1
    previous = right_lane_tracker['fit']
    if previous is not None and right_lane_tracker['lost_frames'] <= RIGHT_HOLD_FRAMES:
        return previous, 'held:' + reason
    return None, 'lost:' + reason


def update_right_lane_fit(hotpixels, image_height):
    """Validate and smooth one right-boundary fit across consecutive frames."""
    rows = np.asarray(hotpixels['x'])
    columns = np.asarray(hotpixels['y'])
    if rows.size < RIGHT_MIN_PIXELS:
        return hold_previous_right_fit('few_pixels')

    vertical_bins = np.unique(np.minimum(
        (rows * 10 / image_height).astype(np.int32), 9))
    if vertical_bins.size < RIGHT_MIN_VERTICAL_BINS:
        return hold_previous_right_fit('short_line')

    candidate = polynomial_fit(hotpixels)
    bottom_row = image_height - 1
    far_row = int(image_height * 0.50)
    bottom_x = evaluate_fit(candidate, bottom_row)
    far_x = evaluate_fit(candidate, far_row)
    if (not np.isfinite(bottom_x) or not np.isfinite(far_x) or
            bottom_x < 640 * 0.45 or bottom_x >= 640 or
            far_x < 220 or far_x >= 640):
        return hold_previous_right_fit('bad_geometry')

    previous = right_lane_tracker['fit']
    if previous is not None:
        previous_bottom_x = evaluate_fit(previous, bottom_row)
        previous_far_x = evaluate_fit(previous, far_row)
        if (abs(bottom_x - previous_bottom_x) > RIGHT_MAX_BOTTOM_JUMP_PX or
                abs(far_x - previous_far_x) > RIGHT_MAX_FAR_JUMP_PX):
            return hold_previous_right_fit('position_jump')

        alpha = RIGHT_SMOOTHING_ALPHA
        candidate = {
            'a0': alpha * candidate['a0'] + (1.0 - alpha) * previous['a0'],
            'a1': alpha * candidate['a1'] + (1.0 - alpha) * previous['a1'],
            'a2': alpha * candidate['a2'] + (1.0 - alpha) * previous['a2']
        }

    right_lane_tracker['fit'] = candidate
    right_lane_tracker['lost_frames'] = 0
    return candidate, 'tracked'


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
    _, img_binary = cv2.threshold(
        img_gray, 170, 255, cv.THRESH_OTSU)

    return img_binary

def image_process(img):
    """ Binarizes and skeletonizes the image.

    Args:
        img

    Returns:
        target_point
    """


    img_bin = binarize(img)
    #cv.imshow('1',img_bin)
    ele = cv2.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    img_bin_rev = cv2.morphologyEx(255 - img_bin, cv.MORPH_OPEN, ele)

    img_bin_rev = cv2.medianBlur(img_bin_rev, 11)
    white = cv2.countNonZero(img_bin_rev)

    skel = morphology.skeletonize(img_bin_rev//255).astype(np.uint8)*255
    #cv.imshow('4',skel)
    #img_bin_rev[skel == 255] = 120

 
    return skel
def lane_detection(img):
    
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    #cv2.imwrite('000.jpg',corr_img)
    gray_ex = cv2.cvtColor(corr_img,cv2.COLOR_RGB2GRAY)
    display(gray_ex,'Apply Camera Correction',color=0)
    #ret, combined_output = cv2.threshold(gray_ex, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    combined_output = cv2.Canny(gray_ex, 200, 400) #100, 200 75,200
  


    show_cv_window('combined_output', combined_output)
    #combined_output = image_process(gray_ex)
    display(combined_output,'Combined output',color=0)
    mask = np.zeros_like(combined_output)
    vertices = np.array([[(100,278),(0,435),(640,435),(500,250)]],dtype=np.int32)
    cv2.fillPoly(mask,vertices,1)
    masked_image = cv2.bitwise_and(combined_output,mask)
    display(masked_image,'Masked',color=0)
    
    min_sz = 50
    #cleaned =              morphology.remove_small_objects(masked_image.astype('bool'),min_size=min_sz,connectivity=2)
    cleaned = masked_image
    display(cleaned,'cleaned',color=0)
    # original image to bird view (transformation)
    src_pts = np.float32([[220,306],[1,435],[639,435],[451,306]])
    dst_pts = np.float32([[70,0],[70,480],[570,480],[570,0]])
    transform_matrix = perspective_transform(src_pts,dst_pts)
    warped_image = birdView(cleaned*1.0,transform_matrix['M'])
    warped_image = cv2.dilate(warped_image, np.ones((15,15), np.uint8))
    warped_image = cv2.erode(warped_image, np.ones((7,7), np.uint8))
    #pubbrid_view.publish(CvBridge().cv2_to_imgmsg(warped_image))
    display(cleaned,'undistorted',color=0)
    display(warped_image,'BirdViews',color=0)
    mid_time=time.time()
######mid_time
    end_time=time.time()
########################################################################fit##################################################################################

    peak_thresh = 10
    showMe = 0
    previous_fit = right_lane_tracker['fit']
    if previous_fit is None:
        centroid_starter_right = find_right_starter(warped_image, peak_thresh)
    else:
        previous_bottom = evaluate_fit(previous_fit, warped_image.shape[0] - 1)
        centroid_starter_right = {
            'centroid': int(max(warped_image.shape[1] / 2,
                                min(warped_image.shape[1] - 1, previous_bottom))),
            'intensity': 1
        }

    sliding_window_specs = {'width': 80, 'n_steps': 10}
    log_lineRight, out_img = run_sliding_window(
        warped_image.copy(), centroid_starter_right['centroid'],
        sliding_window_specs, showMe=showMe)

    fit_lineRight_singleframe, tracking_status = update_right_lane_fit(
        log_lineRight, corr_img.shape[0])

    msg = AckermannDriveStamped()
    if fit_lineRight_singleframe is None:
        result = corr_img.copy()
        cv2.putText(result, 'RIGHT LANE LOST - STOP', (50, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        show_cv_window('result', result)
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        pub.publish(msg)
        rospy.logwarn_throttle(1.0, 'Right lane rejected: %s', tracking_status)
        return

    ym_per_pix = 0.6 / 480.0
    xm_per_pix = 0.6 / LANE_WIDTH_PX
    var_pts = np.linspace(0, corr_img.shape[0] - 1,
                          num=corr_img.shape[0])
    right_fitx = (fit_lineRight_singleframe['a2'] * var_pts ** 2 +
                  fit_lineRight_singleframe['a1'] * var_pts +
                  fit_lineRight_singleframe['a0'])
    center_fitx = right_fitx - HALF_LANE_WIDTH_PX
    center_of_lane = center_fitx[-1]
    offset = (corr_img.shape[1] / 2.0 - center_of_lane) * xm_per_pix

    right_points = np.transpose(np.vstack([right_fitx, var_pts])).astype(np.int32)
    center_points = np.transpose(np.vstack([center_fitx, var_pts])).astype(np.int32)
    wrap_zero = np.zeros_like(gray_ex).astype(np.uint8)
    color_wrap = np.dstack((wrap_zero, wrap_zero, wrap_zero))
    cv2.polylines(color_wrap, [right_points], False, (0, 255, 0), 8)
    cv2.polylines(color_wrap, [center_points], False, (0, 255, 255), 5)
    cv2.line(color_wrap, (corr_img.shape[1] / 2, corr_img.shape[0] - 35),
             (corr_img.shape[1] / 2, corr_img.shape[0] - 5), (0, 0, 255), 5)
    newwrap = cv2.warpPerspective(
        color_wrap, transform_matrix['Minv'],
        (corr_img.shape[1], corr_img.shape[0]))
    result = cv2.addWeighted(corr_img, 1, newwrap, 0.8, 0)
    cv2.putText(result, 'Right lane: ' + tracking_status, (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(result, 'Offset: ' + str(round(offset, 3)) + 'm', (30, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    show_cv_window('result', result)

    print('right_lane_status', tracking_status)
    print('right_lane_pixels', len(log_lineRight['x']))
    print('offset ', offset)
    msg.drive.speed = -30
    # Keep the existing lateral controller; only its lane centre source changed.
    Vehicle_PID.update(offset)
    steering_command = -Vehicle_PID.output * 40.0
    msg.drive.steering_angle = max(-25.0, min(25.0, steering_command))
    print('steering_angle', msg.drive.steering_angle)
    pub.publish(msg)

def camera_callback(data):
    time1 = time.time()
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    global n
    print(laser_cmd)
    if laser_cmd == 0:
        if n >= 1:
            lane_detection(img)
        else:
            n += 1
    time2 = time.time()
    print('totaltime', time2-time1)


def laser_callback(data):
    global laser_cmd
    laser_cmd = data.laser_control
    


def detector():

    global pub
    global pubresult
    global Vehicle_PID
    global show_windows

    Vehicle_PID = PID(3,0,0)
    rospy.init_node('camera_cmd4', anonymous=False)
    camera_topic = rospy.get_param('~camera_topic', '/usb_cam_2/image')
    show_windows = rospy.get_param(
        '~show_windows', bool(os.environ.get('DISPLAY')))
    rospy.loginfo('Front lane camera: %s (640x480 calibration)', camera_topic)
    rospy.Subscriber(camera_topic, Image, camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/laser_control", laser_control, laser_callback, queue_size=1)
    pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
    if show_windows:
        run_debug_window_loop()
    else:
        rospy.spin()

if __name__ == '__main__':
    detector()
