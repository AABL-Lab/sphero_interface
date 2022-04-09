#!/usr/bin/env python3

'''
opencv code by
Email: siddharthchandragzb@gmail.com
'''

from concurrent.futures import process
from math import atan2, degrees
import traceback
from turtle import circle

from IPython import embed
from torch import bitwise_not, threshold
from plot_state import plot_spheros
import utils
import time

import rospy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, PoseWithCovariance, TwistWithCovariance, Pose, Twist, Point, Quaternion
from sphero_interface.msg import SpheroNames, PositionGoal, HeadingStamped
import matplotlib.pyplot as plt

import cv2
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from interface import IN_LAB

'''
Dictionary of Sphero Names (keys) and their corresponding colors (values)
'''
import TrackerParams as tp

'''
These parameters must be tuned for image size
'''
# range of acceptable blob sizes
# expected sphero radius
# upper and lower hsv bounds for color extraction
''''''

CHECK_FOR_UPDATED_PARAMS = True


global EXPECTED_SPHERO_RADIUS, MIN_CONTOUR_AREA, MAX_CONTOUR_AREA, POSITION_COVARIANCE, ORIENTATION_COVARIANCE, FWD_H_RANGE, FWD_S_RANGE, FWD_V_RANGE, VERBOSE, SHOW_IMAGES, LOWER_THRESHOLD, UPPER_THRESHOLD, NINE_SHARPEN_KERNEL, BLUR_KERNEL_SIZE, MORPH_RECT_SIZE
def update_rosparams():
    global EXPECTED_SPHERO_RADIUS, MIN_CONTOUR_AREA, MAX_CONTOUR_AREA, POSITION_COVARIANCE, ORIENTATION_COVARIANCE, FWD_H_RANGE, FWD_S_RANGE, FWD_V_RANGE, VERBOSE, SHOW_IMAGES, LOWER_THRESHOLD, UPPER_THRESHOLD, NINE_SHARPEN_KERNEL, BLUR_KERNEL_SIZE, MORPH_RECT_SIZE

    VERBOSE = rospy.get_param('/param_server/verbose', False)
    SHOW_IMAGES = rospy.get_param('/param_server/show_pipeline', False)
    EXPECTED_SPHERO_RADIUS = rospy.get_param("/param_server/expected_sphero_radius")
    MIN_CONTOUR_AREA = rospy.get_param("/param_server/min_contour_area")
    MAX_CONTOUR_AREA = rospy.get_param("/param_server/max_contour_area")
    POSITION_COVARIANCE = rospy.get_param("/param_server/position_covariance")
    ORIENTATION_COVARIANCE = rospy.get_param("/param_server/orientation_covariance")

    # threshold values for brightness
    NINE_SHARPEN_KERNEL = rospy.get_param("/param_server/nine_sharpen_kernel", True)
    BLUR_KERNEL_SIZE = rospy.get_param("/param_server/blur_kernel_size")
    MORPH_RECT_SIZE = rospy.get_param("/param_server/morph_rect_size")
    LOWER_THRESHOLD = rospy.get_param("/param_server/lower_threshold")
    UPPER_THRESHOLD = rospy.get_param("/param_server/upper_threshold")

    tp.BLUE_HSV = (rospy.get_param("/param_server/blue_h"), rospy.get_param("/param_server/blue_s"), rospy.get_param("/param_server/blue_v"))
    tp.RED_HSV = (rospy.get_param("/param_server/red_h"), rospy.get_param("/param_server/red_s"), rospy.get_param("/param_server/red_v"))
    tp.GREEN_HSV = (rospy.get_param("/param_server/green_h"), rospy.get_param("/param_server/green_s"), rospy.get_param("/param_server/green_v"))
    tp.populate_hsv_dict()

    FWD_H_RANGE = (rospy.get_param("/param_server/fwd_hue_min"), rospy.get_param("/param_server/fwd_hue_max"))
    FWD_S_RANGE = (rospy.get_param("/param_server/fwd_sat_min"), rospy.get_param("/param_server/fwd_sat_max"))
    FWD_V_RANGE = (rospy.get_param("/param_server/fwd_val_min"), rospy.get_param("/param_server/fwd_val_max"))

    rospy.loginfo("Updating ros params")

update_rosparams()
circle_radiuses = dict()
frame_hsv = None

'''
Each sphero gets its own instance of this class
TODO: This class obviously needs a refactor
'''
class VisionDetect:
    def __init__(self, sphero_id=None, imageName=None):
        self.sphero_id = sphero_id
        # self.tracker_params = Sphero_Params_by_ID[sphero_id] if sphero_id is not None else None
        if imageName is not None:
            self.read_image(imageName)

        # This is probably a bad idea
        self.last_detected_color_ts = 0.
        self.last_detected_color_pose = None
        self.theta_smoother = [] # low pass filter for theta

        self.odom_pub = rospy.Publisher(f"/{self.sphero_id}/odom", Odometry, queue_size=10)
        # self.raw_pose_pub = rospy.Publisher(f"/{self.spheroq_id}/pose_raw", Pose2D, queue_size=10)
        # self.pose_pub = rospy.Publisher(f"/{self.sphero_id}/pose", Pose2D, queue_size=1)
        self.initial_heading_pub = rospy.Publisher(f"/{self.sphero_id}/initial_heading", HeadingStamped, queue_size=1, latch=True)

        self.goal = None
        self.goal_sub = rospy.Subscriber(f"/{self.sphero_id}/goal", PositionGoal, goal_cb) # this should be in rviz probably
        self.ekf_sub = rospy.Subscriber(f"/{self.sphero_id}_ekf/odom_combined", PoseWithCovarianceStamped, self.ekf_cb, callback_args=self.sphero_id)
        # self.pose_sub = rospy.Subscriber(f"/{self.sphero_id}/pose", Pose2D, pose_cb, callback_args=self.sphero_id)

        self.initial_heading_samples = []
        self.initial_heading = None

    def read_image(self, image):
        self.image = image
        self.grayimage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

    # Detect Hough Circles on an image. No Sphero specific code. 
    def detectCircle(self, image, minRad = 0, maxRad = 0):
        img = image.copy()
        # grayimage = cv2.cvtColor(img, cv2.COLOR_HSV2GRAY)
        _, _, grayimage = cv2.split(img) # Just use the value channel for gray
        # Blur using 3 * 3 kernel.

        gray_blurred = cv2.blur(grayimage, (3, 3))
        # gray_blurred = grayimage
        
        # Apply Hough transform on the blurred image.
        circles = cv2.HoughCircles(gray_blurred,cv2.HOUGH_GRADIENT, 1, minDist=EXPECTED_SPHERO_RADIUS*1.2, param1=20, param2=15,
            minRadius=int(0.9*EXPECTED_SPHERO_RADIUS),
            maxRadius=int(1.1*EXPECTED_SPHERO_RADIUS))

        if circles is not None:
            circles =  np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(gray_blurred, (i[0], i[1]), i[2], (0,255,0), 2)

        # print(circle_radiuses)
        return gray_blurred, circles

    def processColor(self, hsv_img, lower=None, upper=None, note=""):
        '''
        Produce a mask of a large circle around any the spheros color range. Store result.        
        Returns: The mask or None if colors aren't detected
        '''
        if (lower is None) or (upper is None):
            lower, upper = tp.hsv_bounds_for_id(self.sphero_id)
            # lower = self.tracker_params.hsv_lower
            # upper = self.tracker_params.hsv_upper
            # print(f"{self.sphero_id} lower: {lower} upper: {upper}")

        mask = cv2.inRange(hsv_img, lower, upper)

        # cv2.imshow(f"mask_{self.sphero_id}_{note}", cv2.bitwise_and(hsv_img, hsv_img, mask=mask))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        center = None
        if len(contours) == 0:
            biggest_blob = 0
            pass
            # self.center = None
        else:
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            biggest_blob = cv2.contourArea(blob)
            M = cv2.moments(blob)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            except:
                # rospy.logerr("Couldn't calculate center from moment: {}".format(M))
                center = None

        # if (center):
        #     cv2.circle(mask, center, int(EXPECTED_SPHERO_RADIUS*1.5), (255,255,255), -1)

        return mask, center, biggest_blob

    def set_detected_position(self, x_img, y_img, theta):
        # self.theta_smoother.append(theta)
        # if len(self.theta_smoother) > 10: self.theta_smoother.pop(0)
        # theta = sum(self.theta_smoother)/len(self.theta_smoother)

        self.last_detected_color_pose = (x_img, y_img, theta)
        self.last_detected_color_ts = rospy.get_time()

    def goal_callback(self, msg):
        self.goal = msg

def mouse_cb(event, x, y, flags, params):
    # checking for right mouse clicks
    imagek = frame_hsv.copy()
    if event==cv2.EVENT_MOUSEMOVE:  
        H = imagek[y, x, 0]
        S = imagek[y, x, 1]
        V = imagek[y, x, 2]
        print(f'coord_hsv ({x},{y})-  (h,s,v) {H}, {S}, {V}')

ekf_pose2d = dict()
def pose_cb(data, sphero_id):
    ekf_pose2d[sphero_id] = (data.x, data.y, data.theta)
    # rospy.loginfo(f"{sphero_id} ekf_cb: {ekf_pose2d[sphero_id][0]:1.1f}, {ekf_pose2d[sphero_id][1]:1.1f}, {ekf_pose2d[sphero_id][2]:1.1f}")

def ekf_cb(msg, name):
    r,p,y = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    yaw = utils.cap_0_to_2pi(y)
    ekf_pose2d[name] = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

goal_pose2d = dict()
def goal_cb(position_goal):
    goal_pose2d[position_goal.sphero_name] = position_goal.goal

detectors_dict = dict()
def sphero_names_cb(msg):
    for sphero_name in msg.data:
        if sphero_name not in detectors_dict:
            print(f"Adding {sphero_name} to detectors_dict")
            detectors_dict[sphero_name] = VisionDetect(sphero_name)

# Given the mask, whats the dominant color in expected led grid region?
def get_id_from_hue(masked_hsv):
    possible_ids = detectors_dict.keys()
    closest_id = None
    mean_hue = -1
    mean_sat = -1
    if (len(possible_ids) > 0):
        # Get the mean of the unmasked values
        hues = masked_hsv[:,:,0].flatten()
        mean_hue = np.mean(hues[np.nonzero(hues)])
        sats = masked_hsv[:,:,1].flatten()
        mean_sat = np.mean(sats[np.nonzero(sats)])
        
        closest_hsv = None
        for id, hsv in tp.Sphero_HSV_Color.items():
            if not (id in possible_ids): continue

            if closest_hsv is None:
                closest_hsv = hsv
                closest_id = id
            else:
                if abs(hsv[0] - mean_hue) < abs(closest_hsv[0] - mean_hue):
                # if (sum([abs(hsv[i] - hsv_mean[i]) for i in range(3)] < sum([abs(closest_hsv[i] - hsv_mean[i]) for i in range(3)]))):
                    closest_hsv = hsv
                    closest_id = id

    return closest_id, mean_hue, mean_sat

def get_average_hsv(img_hsv):
    hues = img_hsv[:,:,0].flatten()
    saturations = img_hsv[:,:,1].flatten()
    values = img_hsv[:,:,2].flatten()
    hues = hues[hues > 70]
    return np.mean(hues[np.nonzero(hues)]), np.mean(saturations[np.nonzero(saturations)]), np.mean(values[np.nonzero(values)])

def main():
    rospy.init_node("tracker")
    rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
    I_generic = VisionDetect()

    if (SHOW_IMAGES):
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.moveWindow('image', 0, 0)

    # >>>> Open Video Stream
    video = utils.init_videocapture(channel=0 if IN_LAB else 2)
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        return 
    # <<<< Open Video Stream

    updates = -1
    start_time = time.time()
    while not rospy.is_shutdown(): # do work
        if len(detectors_dict.keys()) == 0:
            rospy.sleep(0.1)
            continue
        elif (CHECK_FOR_UPDATED_PARAMS and updates % 50 == 0): update_rosparams()
        
        updates += 1
        if updates % 100 == 0:
            rospy.loginfo(f"Tracker running at {100/(time.time() - start_time):1.1f} Hz")
            start_time = time.time()

        # Read first frame.
        global frame
        ok, frame = video.read()
        if not ok:
            print('Couldnt read frame')
            continue

        '''
        Mask each detected circle, check what color the grid is, and update the corresponding sphero
        '''
        global frame_hsv
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        image = frame.copy()
        height,width,depth = image.shape

        hue_range = (rospy.get_param("/param_server/hue_min"), rospy.get_param("/param_server/hue_max"))
        sat_range = (rospy.get_param("/param_server/sat_min"), rospy.get_param("/param_server/sat_max"))
        val_range = (rospy.get_param("/param_server/val_min"), rospy.get_param("/param_server/val_max"))

        bl, bu = (tp.BLUE_HSV[0] - hue_range[0], tp.BLUE_HSV[1] - sat_range[0], tp.BLUE_HSV[2] - val_range[0]), (tp.BLUE_HSV[0] + hue_range[1], tp.BLUE_HSV[1] + sat_range[1], tp.BLUE_HSV[2] + val_range[1])
        gl, gu = (tp.GREEN_HSV[0] - hue_range[0], tp.GREEN_HSV[1] - sat_range[0], tp.GREEN_HSV[2] - val_range[0]), (tp.GREEN_HSV[0] + hue_range[1], tp.GREEN_HSV[1] + sat_range[1], tp.GREEN_HSV[2] + val_range[1])
        rl, ru = (tp.RED_HSV[0] - hue_range[0], tp.RED_HSV[1] - sat_range[0], tp.RED_HSV[2] - val_range[0]), (tp.RED_HSV[0] + hue_range[1], tp.RED_HSV[1] + sat_range[1], tp.RED_HSV[2] + val_range[1])

        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # blue = cv2.inRange(frame_hsv, np.array([bl,0,LOWER_THRESHOLD]), np.array([bu,255,UPPER_THRESHOLD]))
        # red = cv2.inRange(frame_hsv, np.array([rl,0,LOWER_THRESHOLD]), np.array([ru,255,UPPER_THRESHOLD]))
        # green = cv2.inRange(frame_hsv, np.array([gl,0,LOWER_THRESHOLD]), np.array([gu,255,UPPER_THRESHOLD]))

        green = cv2.inRange(frame_hsv, gl, gu)
        red = cv2.inRange(frame_hsv, rl, ru)
        blue = cv2.inRange(frame_hsv, bl, bu)
        # green = cv2.inRange(frame_hsv, np.array([min([bl, gl, rl]), 0, LOWER_THRESHOLD]), np.array([max([bu, gu, ru]), 255, UPPER_THRESHOLD]))

        all_colors = cv2.bitwise_or(cv2.bitwise_or(red, green), blue)

        for f, color_string in zip([green, red, blue], ["green", "red", "blue"]):
            blur = cv2.GaussianBlur(f, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), cv2.BORDER_DEFAULT)
            sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
            sharpen = cv2.filter2D(blur, -1, sharpen_kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_RECT_SIZE,MORPH_RECT_SIZE))
            close = cv2.morphologyEx(sharpen, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            id = tp.colorstring_to_id[color_string] # we know the id because we're searching specifically for the color
            if id not in detectors_dict.keys(): continue

            cnts = cv2.findContours(close, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]
            if len(cnts) == 0: continue
            c = cnts[0]
            area = cv2.contourArea(c)
            if (area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA):
                cv2.imshow(f'rejected_close_{color_string}', close)
                # print(f"Contour for {id} is too small or too large: {area}")
                continue
            else:
                cv2.imshow(f'close_{color_string}', close)

            x,y,w,h = cv2.boundingRect(c)
            cx, cy = x+(w//2), y+(h//2)

            mask0 = np.zeros((height,width), np.uint8)
            cv2.circle(mask0, (cx, cy), int(EXPECTED_SPHERO_RADIUS*0.8), (255,255,255), -1)
            mask1 = np.zeros((height,width), np.uint8)
            cv2.circle(mask1, (cx, cy), int(EXPECTED_SPHERO_RADIUS*1.5), (255,255,255), -1)
            rim_mask = cv2.bitwise_and(cv2.bitwise_not(mask0), mask1)
            rim_img = cv2.bitwise_and(image, image, mask=rim_mask)
            rim_img = cv2.cvtColor(rim_img, cv2.COLOR_RGB2HSV)
            cv2.imshow(f'rim_img_{color_string}', rim_img)

            _, fwd_center, _ = detectors_dict[id].processColor(rim_img, lower=(FWD_H_RANGE[0], FWD_S_RANGE[0], FWD_V_RANGE[0]), upper=(FWD_H_RANGE[1], FWD_S_RANGE[1], FWD_V_RANGE[1]), note="fwd")
            if not fwd_center: continue
            fx,fy = fwd_center

            theta = atan2(-fy + cy, fx - cx)
            theta = utils.cap_0_to_2pi(theta)
            detectors_dict[id].set_detected_position(cx, cy, theta)

            cv2.line(image, (cx,cy), (fx,fy), (255,0,0), 2)

        cv2.imshow('image', image)
        # cv2.imshow('frame_hsv', frame_hsv)

        # if (SHOW_IMAGES):
            # cv2.setMouseCallback('image', mouse_cb)
        cv2.setMouseCallback('image', mouse_cb)

        for I in detectors_dict.values():
            pose_img = I.last_detected_color_pose
            if pose_img is not None:
                x,y,theta = pose_img
                # make sure we have a baseline heading for each sphero
                if I.initial_heading is None:
                    I.initial_heading_samples.append(theta)
                    if (len(I.initial_heading_samples) > 10 and np.std(I.initial_heading_samples) < 0.1):
                        rospy.loginfo(f"Setting initial heading for {I.sphero_id} to {np.mean(I.initial_heading_samples):1.2f}")
                        rospy.loginfo([f"{entry:1.1f}" for entry in I.initial_heading_samples])
                        I.initial_heading = np.mean(I.initial_heading_samples)
                        I.initial_heading_pub.publish(HeadingStamped(rospy.get_time(), 0.0, I.initial_heading))
                    elif (len(I.initial_heading_samples) > 30): # Edge case where initial readings are wrong and we get a hug variance that will take a million years to drop
                        I.initial_heading_samples = []
                    else:
                        rospy.loginfo(f"Not enough samples to set initial heading for {I.sphero_id}")
                        rospy.loginfo(f"{I.sphero_id} n {len(I.initial_heading_samples)} {np.std(I.initial_heading_samples):1.2f} theta {theta:1.2f}")
                else:
                    if (rospy.get_time() - I.last_detected_color_ts < 0.2):
                        # data = Pose2D(x, y, theta)
                        # I.pose_pub.publish(data)
                        # pose_cb(data, I.sphero_id)
                        odom_msg = Odometry()
                        odom_msg.header.stamp = rospy.Time.now()
                        odom_msg.header.frame_id = "odom"
                        p = PoseWithCovariance()
                        p.pose.position = Point(x,y,0)

                        # Calculate the offset theta considering the initial heading of the sphero. This must be done so the ekf sees consistent data between the orientation published in interface.py and this detection

                        q = quaternion_from_euler(0., 0., theta)
                        p.pose.orientation = Quaternion(*q)
                        p.covariance = np.array([ # ignore covairance between factors, but the way we're doing this we get a lot of jitter, so x and y definitely have some variance. In pixels.
                            [POSITION_COVARIANCE, 0., 0., 0., 0., 0.], # pixels
                            [0., POSITION_COVARIANCE, 0., 0., 0., 0.],  # pixels
                            [0., 0., 1e-6, 0., 0., 0.],
                            [0., 0., 0., 1e-6, 0., 0.],
                            [0., 0., 0., 0., 1e-6, 0.],
                            [0., 0., 0., 0., 0., ORIENTATION_COVARIANCE], # radians
                            ]).flatten() # TODO
                        tw = TwistWithCovariance() # TODO
                        odom_msg.pose = p
                        odom_msg.twist = tw
                        I.odom_pub.publish(odom_msg)
                        I.last_detected_color_pose = None



        if (ekf_pose2d):
            efk_frame = frame.copy()
            for sphero_id, (x,y,theta) in ekf_pose2d.items():
                if (x > 0 and y > 0):
                    # cv2.circle(efk_frame, (int(x), int(y)), 5, (0,255,0), -1)
                    cv2.line(efk_frame, (int(x), int(y)), (int(x+15*np.cos(theta)), int(y-15*np.sin(theta))), (0,0,255), 2)
            
            for sphero_id, pose2d in goal_pose2d.items():
                cv2.circle(efk_frame, (int(pose2d.x), int(pose2d.y)), 25, tp.Sphero_RGB_Color[sphero_id], -1)
            cv2.imshow("tracked", efk_frame)

            # Plot spheros NOTE: Out of place, should be its own node probably
            plot_spheros([ekf_pose2d[key] for key in ekf_pose2d.keys()], [key for key in ekf_pose2d.keys()], ax_x_range=[0, frame.shape[1]], ax_y_range=[frame.shape[0], 0])



        # Exit if ESC pressed
        if cv2.waitKey(1) & 0xFF == ord('q'): # if press SPACE bar
            rospy.signal_shutdown("Quit")
            break

        # rospy.sleep(0.01) # sleep for messages and interrupts
    
    video.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


    