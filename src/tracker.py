#!/usr/bin/env python3

'''
opencv code by
Email: siddharthchandragzb@gmail.com
'''

from math import atan2, degrees
from re import S
import traceback
from IPython import embed

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, PoseWithCovariance, TwistWithCovariance, Pose, Twist, Point, Quaternion
from sphero_interface.msg import SpheroNames
import matplotlib.pyplot as plt

import cv2
import numpy as np
from scipy import ndimage

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

'''
Dictionary of Sphero Names (keys) and their corresponding colors (values)
'''
from TrackerParams import LOWER_GREEN, UPPER_GREEN, Sphero_Params_by_ID, TrackerParams

SHOW_IMAGES = True

EXPECTED_SPHERO_RADIUS = 35 # size of spheros in pixels
circle_radiuses = dict()
hsv_frame = None

class VisionDetect:
    def __init__(self, sphero_id=None, imageName=None):
        self.sphero_id = sphero_id
        self.tracker_params = Sphero_Params_by_ID[sphero_id] if sphero_id is not None else None
        if imageName is not None:
            self.read_image(imageName)

        # This is probably a bad idea
        self.last_detected_color_ts = 0.
        self.last_detected_color_pose = None
        self.theta_smoother = [] # low pass filter for theta

        self.odom_pub = rospy.Publisher(f"/{self.sphero_id}/odom", Odometry, queue_size=10)
        
        self.goal = None
        self.goal_sub = rospy.Subscriber(f"/{self.sphero_id}/goal", Pose2D, goal_cb, callback_args=self.sphero_id) # this should be in rviz probably
        self.ekf_odom_sub = rospy.Subscriber(f"/{self.sphero_id}_ekf/odom_combined", PoseWithCovarianceStamped, ekf_cb, callback_args=self.sphero_id)

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
        
        # Apply Hough transform on the blurred image.
        circles = cv2.HoughCircles(gray_blurred,cv2.HOUGH_GRADIENT, 1, minDist=EXPECTED_SPHERO_RADIUS,param1=20,param2=30,
            minRadius=int(0.9*EXPECTED_SPHERO_RADIUS),
            maxRadius=int(1.1*EXPECTED_SPHERO_RADIUS))

        if circles is not None:
            circles =  np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(gray_blurred, (i[0], i[1]), i[2], (0,255,0), 2)

        # print(circle_radiuses)
        return gray_blurred, circles

    def processColor(self, hsv_img, lower=None, upper=None):
        '''
        Produce a mask of a large circle around any the spheros color range. Store result.        
        Returns: The mask or None if colors aren't detected
        '''
        if (lower is None) or (upper is None):
            lower = self.tracker_params.hsv_lower
            upper = self.tracker_params.hsv_upper
            # print(f"{self.sphero_id} lower: {lower} upper: {upper}")

        mask = cv2.inRange(hsv_img, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        center = None
        if len(contours) == 0:
            pass
            # self.center = None
        else:
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            M = cv2.moments(blob)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            except:
                # rospy.logerr("Couldn't calculate center from moment: {}".format(M))
                center = None

        if (center):
            cv2.circle(mask, center, int(EXPECTED_SPHERO_RADIUS*1.5), (255,255,255), -1)

        return mask, center

        #cv2.imshow('mask',mask)
        # res = cv2.bitwise_and(img, img, mask=mask)
        # return res
    def set_detected_position(self, x_img, y_img, theta):
            self.theta_smoother.append(theta)
            if len(self.theta_smoother) > 10: self.theta_smoother.pop(0)
            theta = sum(self.theta_smoother)/len(self.theta_smoother)

            self.last_detected_color_pose = (x_img, y_img, theta)
            self.last_detected_color_ts = rospy.get_time()

    # Detect Hough Lines
    def detectLine(self, minLen = 20, maxLGap = 5):
        img = self.image.copy()
        edgeimg = cv2.Canny(img, 100, 200)
        lines = cv2.HoughLinesP(edgeimg, 1, np.pi/180, minLen,
                                maxLGap)
        for line in lines[0]:
            cv2.line(img, (line[0], line[1]), (line[2], line[3]), (0,0,255), 2)
        return img

    def goal_callback(self, msg):
        self.goal = msg

def img_to_world(coords_img):
    # TODO: Hardcoded until in lab
    x = coords_img[0]
    y = coords_img[1]
    return x, y

def mouse_cb(event, x, y, flags, params):
    # checking for right mouse clicks
    imagek = hsv_frame.copy()
    if event==cv2.EVENT_MOUSEMOVE:  
        H = imagek[y, x, 0]
        S = imagek[y, x, 1]
        V = imagek[y, x, 2]
        print(f'({x},{y})-  (h,s,v) {H}, {S}, {V}')

ekf_pose2d = dict()
def ekf_cb(data, sphero_id):
    ekf_pose2d[sphero_id] = (data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2])
    rospy.loginfo(f"{sphero_id} ekf_cb: {ekf_pose2d[sphero_id][0]:1.1f}, {ekf_pose2d[sphero_id][1]:1.1f}, {ekf_pose2d[sphero_id][2]:1.1f}")

goal_pose2d = dict()
def goal_cb(goal_pose, sphero_id):
    goal_pose2d[sphero_id] = goal_pose


detectors_dict = dict()
def sphero_names_cb(msg):
    for sphero_name in msg.data:
        if sphero_name not in detectors_dict:
            print(f"Adding {sphero_name} to detectors_dict")
            detectors_dict[sphero_name] = VisionDetect(sphero_name)

def main():
    rospy.init_node("tracker")
    rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
    I_generic = VisionDetect()

    if (SHOW_IMAGES):
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('color', cv2.WINDOW_NORMAL)
        # cv2.namedWindow('tracked', cv2.WINDOW_NORMAL)
        cv2.moveWindow('image', 0, 0)
        cv2.moveWindow('color', 750, 0)
        # cv2.moveWindow('tracked', 600, 500)

    # >>>> Open Video Stream
    video = cv2.VideoCapture(-1) # for using CAM
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        return 
    # <<<< Open Video Stream

    while not rospy.is_shutdown(): # do work
        if len(detectors_dict.keys()) == 0:
            rospy.sleep(0.1)
            continue

        # Read first frame.
        ok, frame = video.read()
        if not ok:
            print('Couldnt read frame')
            continue

        # >>>> Filter for all the spheros colors
        global hsv_frame
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = []
        for idx, I in enumerate(detectors_dict.values()):
            # Get a circle around the sphero-specific color
            mask, center = I.processColor(hsv_frame)
            masks.append(mask)
            # grab the green from this circle to indicate the forward direction
            if (center is not None):
                orientation_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)
                green_mask, green_center = I_generic.processColor(orientation_frame, lower=LOWER_GREEN, upper=UPPER_GREEN)
                # gray_blurred, circles = I_generic.detectCircle(orientation_frame)
                if (green_center is not None):
                    theta = atan2(-green_center[1] + center[1], green_center[0] - center[0]) # Image coords need to have y swapped
                    # print(f"{I.sphero_id} theta: {theta:1.2f}")q
                    cv2.line(frame, green_center, center, (255,0,0), 2)
                    
                    avg_center = ((center[0] + green_center[0])/2, (center[1] + green_center[1])/2)
                    I.set_detected_position(avg_center[0], avg_center[1], theta)
                    # if circles is not None:
                    #     cv2.line(frame, (circles[0][0], circles[0][1], circles[0][2]), green_center, (255,0,0), 2)


        color_mask = None
        for mask in masks:
            if (color_mask is None): color_mask = mask
            else:
                color_mask = cv2.bitwise_or(color_mask, mask)

        # expandedBlue = I.getBluePreprocessed(frame)
        # print(f"ORd {len(masks)} masks")
        color_frame = cv2.bitwise_and(frame, frame, mask=color_mask)
        # <<<<< Filter for all the spheros colors


        # >>>>> Detect Circles and add to position estimate
        # I_generic.read_image(color_frame)
        # circle = I_generic.detectCircle()
        # <<<<< Detect Circles and add to position estimate

        # cv2.imshow("circles", circle)

        if (SHOW_IMAGES):
            cv2.imshow("image", frame)
            cv2.imshow("color", color_frame)
            cv2.setMouseCallback('image', mouse_cb)

        if (SHOW_IMAGES and ekf_pose2d):
            efk_frame = frame.copy()
            for sphero_id, (x,y,theta) in ekf_pose2d.items():
                # rospy.loginfo(f"{sphero_id} {x:1.2f}, {y:1.2f}")
                if (x > 0 and y > 0):
                    cv2.circle(efk_frame, (int(x), int(y)), 5, (0,255,0), -1)
                    draw_theta = theta
                    cv2.line(efk_frame, (int(x), int(y)), (int(x+15*np.cos(draw_theta)), int(y-15*np.sin(draw_theta))), (0,0,255), 2)
            
            for pose2d in goal_pose2d.values():
                cv2.circle(efk_frame, (int(pose2d.x), int(pose2d.y)), 25, (255,255,255), -1)
            cv2.imshow("tracked", efk_frame)

        # >>>> convert image coordinates to scene coordinates
        # TODO: Connect to each individual sphero's ekf node (if resources allow)
        for I in detectors_dict.values():
            pose_img = I.last_detected_color_pose
            if pose_img is not None:
                # x,y = img_to_world(pose_img) # TODO: need to transform?
                x,y,theta = pose_img
                odom_msg = Odometry()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = "odom"
                p = PoseWithCovariance()
                p.pose.position = Point(x,y,0)
                # q = quaternion_from_euler(0., 0., theta)
                q = quaternion_from_euler(0., 0., theta)
                p.pose.orientation = Quaternion(*q)
                p.covariance = (1e-6*np.identity(6)).flatten() # TODO
                tw = TwistWithCovariance() # TODO
                odom_msg.pose = p
                odom_msg.twist = tw
                I.odom_pub.publish(odom_msg)
                I.last_detected_color_pose = None
        # <<<< convert image coordinates to scene coordinates

        # Exit if ESC pressed
        if SHOW_IMAGES and cv2.waitKey(1) & 0xFF == ord('q'): # if press SPACE bar
            rospy.signal_shutdown("Quit")
            break

        rospy.sleep(0.01) # sleep for messages and interrupts
    
    video.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


    