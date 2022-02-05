#!/usr/bin/env python3

'''
opencv code by
Email: siddharthchandragzb@gmail.com
'''

from math import atan2, degrees
import traceback

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

'''
Dictionary of Sphero Names (keys) and their corresponding colors (values)
'''
from TrackerParams import LOWER_GREEN, UPPER_GREEN, Sphero_Params_by_ID, TrackerParams

EXPECTED_SPHERO_RADIUS = 46 # size of spheros in pixels
circle_radiuses = dict()
hsv = None

class VisionDetect:
    def __init__(self, sphero_id=None, imageName=None):
        self.sphero_id = sphero_id
        self.tracker_params = Sphero_Params_by_ID[sphero_id] if sphero_id is not None else None
        if imageName is not None:
            self.read_image(imageName)

        # This is probably a bad idea
        self.last_detected_color_ts = 0.
        self.last_detected_color_pose = None

    def read_image(self, image):
        self.image = image
        self.grayimage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)


    # Detect Hough Circles on an image. No Sphero specific code. 
    def detectCircle(self, minRad = 0, maxRad = 0):
        img = self.image.copy()
        # Blur using 3 * 3 kernel.
        gray_blurred = cv2.blur(self.grayimage, (3, 3))
        
        # Apply Hough transform on the blurred image.
        circles = cv2.HoughCircles(gray_blurred,cv2.HOUGH_GRADIENT, 1, minDist=EXPECTED_SPHERO_RADIUS,param1=20,param2=30,
            minRadius=int(0.9*EXPECTED_SPHERO_RADIUS),
            maxRadius=int(1.1*EXPECTED_SPHERO_RADIUS))

        if circles is not None:
            circles =  np.uint16(np.around(circles))
            for i in circles[0, :]:
                r = i[2]
                if r not in circle_radiuses:
                    circle_radiuses[r] = 1
                else:
                    circle_radiuses[r] += 1
                cv2.circle(gray_blurred, (i[0], i[1]), i[2], (0,255,0), 2)

        # print(circle_radiuses)
        return gray_blurred

    def processColor(self, hsv_img, lower=None, upper=None):
        '''
        Produce a mask of a large circle around any the spheros color range. Store result.        
        Returns: The mask or None if colors aren't detected
        '''
        global hsv
        hsv = hsv_img.copy()

        if (lower is None) or (upper is None):
            lower = self.tracker_params.hsv_lower
            upper = self.tracker_params.hsv_upper

        mask = cv2.inRange(hsv, lower, upper)

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
            self.last_detected_color_pose = center
            self.last_detected_color_ts = rospy.get_time()

        return mask, center

        #cv2.imshow('mask',mask)
        # res = cv2.bitwise_and(img, img, mask=mask)
        # return res

    # Detect Hough Lines
    def detectLine(self, minLen = 20, maxLGap = 5):
        img = self.image.copy()
        edgeimg = cv2.Canny(img, 100, 200)
        lines = cv2.HoughLinesP(edgeimg, 1, np.pi/180, minLen,
                                maxLGap)
        for line in lines[0]:
            cv2.line(img, (line[0], line[1]), (line[2], line[3]), (0,0,255), 2)
        return img

def img_to_world(coords_img):
    # TODO: Hardcoded until in lab
    x = coords_img[0]
    y = coords_img[1]
    return x, y

def mouse_cb(event, x, y, flags, params):
    imgk = hsv.copy()
    # checking for right mouse clicks     
    if event==cv2.EVENT_MOUSEMOVE:  
        H = imgk[y, x, 0]
        S = imgk[y, x, 1]
        V = imgk[y, x, 2]
        print(f'({x},{y})-  (h,s,v) {H}, {S}, {V}')

ekf_pose2d = None
def ekf_cb(data):
    global ekf_pose2d
    ekf_pose2d = data.pose.pose.position.x, data.pose.pose.position.y


detectors_dict = dict()
def sphero_names_cb(msg):
    for sphero_name in msg.data:
        if sphero_name not in detectors_dict:
            print(f"Adding {sphero_name} to detectors_dict")
            detectors_dict[sphero_name] = VisionDetect(sphero_name)

def main():
    rospy.init_node("tracker")
    rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, ekf_cb)

    odom_pub = rospy.Publisher("/pr2_base_odometry/odom", Odometry, queue_size=1)

    I_generic = VisionDetect()
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('color', cv2.WINDOW_NORMAL)
    cv2.namedWindow('circles', cv2.WINDOW_NORMAL)
    cv2.moveWindow('image', 0, 0)
    cv2.moveWindow('color', 750, 0)
    cv2.moveWindow('circles', 1500, 0)

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
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = []
        for idx, I in enumerate(detectors_dict.values()):
            mask, center = I.processColor(hsv_frame)
            masks.append(mask)
            # grab the green from this circle
            if (center is not None):
                orientation_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)
                green_mask, green_center = I_generic.processColor(orientation_frame, lower=LOWER_GREEN, upper=UPPER_GREEN)
                if (green_center is not None):
                    theta = degrees(atan2(green_center[1] - center[1], green_center[0] - center[0]))
                    print(f"{I.sphero_id} theta: {theta:1.2f}")
                    cv2.line(frame, center, green_center, (0,255,0), 2)


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
        cv2.imshow("image", frame)
        cv2.imshow("color", color_frame)
        cv2.setMouseCallback('image', mouse_cb)

        if (ekf_pose2d is not None):
            x, y = ekf_pose2d
            efk_frame = frame.copy()
            cv2.circle(efk_frame, (int(x), int(y)), 5, (0,255,0), -1)
            cv2.imshow("tracked", efk_frame)

        # >>>> convert image coordinates to scene coordinates
        # TODO: Connect to each individual sphero's ekf node (if resources allow)
        pose_img = detectors_dict["se9"].last_detected_color_pose
        if pose_img is not None:
            x,y = img_to_world(pose_img)
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            p = PoseWithCovariance()
            p.pose.position = Point(x,y,0)
            p.pose.orientation = Quaternion(0,0,0,1)
            p.covariance = np.identity(6).flatten() # TODO
            tw = TwistWithCovariance() # TODO
            odom_msg.pose = p
            odom_msg.twist = tw
            # print(odom_msg)
            odom_pub.publish(odom_msg)
            detectors_dict["se9"].last_detected_color_pose = None
        # <<<< convert image coordinates to scene coordinates

        # Exit if ESC pressed
        if cv2.waitKey(1) & 0xFF == ord('q'): # if press SPACE bar
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


    