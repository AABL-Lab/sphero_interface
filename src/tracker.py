#!/usr/bin/env python3

'''
opencv code by
Email: siddharthchandragzb@gmail.com
'''

from math import atan2, degrees
import traceback
from turtle import circle

from IPython import embed
from torch import bitwise_not
from plot_state import plot_spheros
import utils

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
from TrackerParams import LOWER_GREEN, TRACK_WITH_CIRCLES, UPPER_GREEN, Sphero_Params_by_ID, Sphero_HSV_Color, Sphero_RGB_Color, TrackerParams, LOWER_WHITE, UPPER_WHITE

'''
These parameters must be tuned for image size
'''
# range of acceptable blob sizes
# expected sphero radius
# upper and lower hsv bounds for color extraction
''''''

VERBOSE = True
SHOW_IMAGES = True

EXPECTED_SPHERO_RADIUS = 50 # size of spheros in pixels
circle_radiuses = dict()
frame_hsv = None

'''
Each sphero gets its own instance of this class
TODO: This class obviously needs a refactor
'''
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
        self.initial_heading_pub = rospy.Publisher(f"/{self.sphero_id}/initial_heading", HeadingStamped, queue_size=1, latch=True)

        self.goal = None
        self.goal_sub = rospy.Subscriber(f"/{self.sphero_id}/goal", PositionGoal, goal_cb) # this should be in rviz probably
        self.pose_sub = rospy.Subscriber(f"/{self.sphero_id}/pose", Pose2D, pose_cb, callback_args=self.sphero_id)

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

    def processColor(self, hsv_img, lower=None, upper=None):
        '''
        Produce a mask of a large circle around any the spheros color range. Store result.        
        Returns: The mask or None if colors aren't detected
        '''
        if (lower is None) or (upper is None):
            lower = self.tracker_params.hsv_lower
            upper = self.tracker_params.hsv_upper
            # lower = (self.tracker_params.hsv_lower[0], 0, 240)
            # upper = (self.tracker_params.hsv_upper[0], 255, 255)
            # print(f"{self.sphero_id} lower: {lower} upper: {upper}")

        mask = cv2.inRange(hsv_img, lower, upper)

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
        self.theta_smoother.append(theta)
        if len(self.theta_smoother) > 10: self.theta_smoother.pop(0)
        theta = sum(self.theta_smoother)/len(self.theta_smoother)
        # if VERBOSE:
        #     rospy.loginfo(f"{self.sphero_id}: Detected position: {x_img}, {y_img}, {theta}")

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
        for id, hsv in Sphero_HSV_Color.items():
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

# Given the dominant color, which sphero is it?
def sphero_from_color(hsv):
    pass

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

    while not rospy.is_shutdown(): # do work
        if len(detectors_dict.keys()) == 0:
            rospy.sleep(0.1)
            continue

        # Read first frame.
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
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)
        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

        thresh = cv2.threshold(sharpen,160,255, cv2.THRESH_BINARY_INV)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts = cv2.findContours(close, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        min_area = 1000
        max_area = 2500
        circle_masks = [] # circles around and including polygon
        poly_masks = [] # polygons themselves
        centers = []
        for c in cnts:
            area = cv2.contourArea(c)
            # print(f" {len(c)} points area {area}")
            if area > min_area and area < max_area:
                mask = np.zeros((height,width), np.uint8)
                x,y,w,h = cv2.boundingRect(c)
                cx, cy = x+(w//2), y+(h//2)
                cv2.circle(mask, (cx, cy), int(EXPECTED_SPHERO_RADIUS*1.5), (255,255,255), -1)
                circle_masks.append(mask)
                centers.append((cx,cy))

                mask = np.zeros((height,width), np.uint8)
                # cv2.fillPoly(mask, pts=[c], color=(255,255,255))
                cv2.circle(mask, (cx, cy), int(EXPECTED_SPHERO_RADIUS*0.6), (255,255,255), -1)
                poly_masks.append(mask)
                
                for i in range(len(c)):
                    x0,y0 = c[i][0]
                    if i == len(c) - 1:
                        x1,y1 = c[0][0]
                    else:
                        x1,y1 = c[i+1][0]

                    cv2.line(image, (x0,y0), (x1,y1), (255,0,0), 2)
            #     # ROI = image[y:y+h, x:x+w]
            #     # cv2.imwrite('ROI_{}.png'.format(image_number), ROI)
            #     cv2.rectangle(image, (x, y), (x + w, y + h), (36,255,12), 2)
            #     image_number += 1
            elif area > min_area:
                # print(f"ignored contour area {area}. Too big")
                pass

        if SHOW_IMAGES:
            cv2.imshow('sharpen', sharpen)
            cv2.imshow('close', close)
            cv2.imshow('thresh', thresh)
        
        # print(f"centers {centers} n_circles {len(circle_masks)} n_polys {len(poly_masks)}")
        idx0 = 0
        for circle_mask, poly_mask, (cx, cy) in zip(circle_masks, poly_masks, centers):
            # print("=" * 60)
            # print(f"{cx:1.0f}, {cy:1.0f}")
            rim_mask = cv2.bitwise_and(circle_mask, cv2.bitwise_not(poly_mask))
            rim_img = cv2.bitwise_and(frame, frame, mask=rim_mask)
            if (SHOW_IMAGES): cv2.imshow(f'rim_img_rgb{idx0}', rim_img)
            rim_img = cv2.cvtColor(rim_img, cv2.COLOR_RGB2HSV)
            # cv2.imshow('rim_mask', rim_mask)

            for idx, detector in enumerate(detectors_dict.values()):
                color_mask, color_center, biggest_blob = detector.processColor(rim_img.copy())
                if (color_center) is not None:
                    color_id_mask = cv2.bitwise_or(poly_mask, color_mask)
                    # h,s,v = get_average_hsv(cv2.bitwise_and(frame, frame, mask=color_id_mask))
                    color_id_hsv = cv2.bitwise_and(frame, frame, mask=color_id_mask)
                    if (SHOW_IMAGES): cv2.imshow(f'color_id_{idx0}_{idx}', color_id_hsv)
                    id, mean_hue, _ = get_id_from_hue(cv2.cvtColor(color_id_hsv, cv2.COLOR_RGB2HSV))
                    if id in detectors_dict.keys():
                        theta = atan2(-color_center[1] + cy, color_center[0] - cx)
                        detectors_dict[id].set_detected_position(cx, cy, theta)
                        print(f"{id} h: {mean_hue} {cx:1.1f}, {cy:1.1f}, {theta:1.1f} blobsize: {biggest_blob:1.1f}")

            # for idx, detector in enumerate(detectors_dict.values()):
            #     color_mask, color_center, biggest_blob = detector.processColor(rim_img.copy())
            #     if (SHOW_IMAGES): cv2.imshow(f'color_mask_{idx0}_{idx}', color_mask)
            #     if (color_center is not None and biggest_blob > 40):
            #         theta = atan2(-color_center[1] + cy, color_center[0] - cx)
            #         cv2.line(image, (color_center[0], color_center[1]), (cx,cy), (255,0,0), 2)
            #         cv2.circle(rim_img, (color_center[0], color_center[1]), 5, (255,0,0), -1)
            #         cv2.imshow(f'rim_img', rim_img)

            #         color_id_mask = cv2.bitwise_or(poly_mask, color_mask)
            #         # h,s,v = get_average_hsv(cv2.bitwise_and(frame, frame, mask=color_id_mask))
            #         color_id_hsv = cv2.bitwise_and(frame, frame, mask=color_id_mask)
            #         if (SHOW_IMAGES): cv2.imshow(f'color_id_{idx0}_{idx}', color_id_hsv)
            #         id = get_id_from_hue(cv2.cvtColor(color_id_hsv, cv2.COLOR_RGB2HSV))
            #         rospy.loginfo(f"Got id {id} from detector_{detector.sphero_id} blobsize: {biggest_blob}")
            #         if (id == detector.sphero_id):
            #             detector.set_detected_position(cx, cy, theta)
            #     else:
            #         print(f"{detector.sphero_id} Skipping blob {biggest_blob}")
            #         pass
            idx0 += 1
                    

        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'): # if press SPACE bar
            rospy.signal_shutdown("Quit")
            break

        if (SHOW_IMAGES):
            # cv2.setMouseCallback('image', mouse_cb)
            cv2.setMouseCallback('rim_img_rgb', mouse_cb)

            if (ekf_pose2d):
                efk_frame = frame.copy()
                for sphero_id, (x,y,theta) in ekf_pose2d.items():
                    if (x > 0 and y > 0):
                        cv2.circle(efk_frame, (int(x), int(y)), 5, (0,255,0), -1)
                        draw_theta = theta
                        cv2.line(efk_frame, (int(x), int(y)), (int(x+15*np.cos(draw_theta)), int(y-15*np.sin(draw_theta))), (0,0,255), 2)
                
                for pose2d in goal_pose2d.values():
                    cv2.circle(efk_frame, (int(pose2d.x), int(pose2d.y)), 25, (255,255,255), -1)
                cv2.imshow("tracked", efk_frame)

        # Plot spheros NOTE: Out of place, should be its own node probably
        plot_spheros([ekf_pose2d[key] for key in ekf_pose2d.keys()], [key for key in ekf_pose2d.keys()], ax_x_range=[0, frame.shape[1]], ax_y_range=[frame.shape[0], 0])


        # >>>> convert image coordinates to scene coordinates
        # TODO: Connect to each individual sphero's ekf node (if resources allow)
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
                else:
                    odom_msg = Odometry()
                    odom_msg.header.stamp = rospy.Time.now()
                    odom_msg.header.frame_id = "odom"
                    p = PoseWithCovariance()
                    p.pose.position = Point(x,y,0)

                    # Calculate the offset theta considering the initial heading of the sphero. This must be done so the ekf sees consistent data between the orientation published in interface.py and this detection
                    offset_theta = theta - I.initial_heading
                    while offset_theta > np.pi: offset_theta -= np.pi*2
                    while offset_theta < -np.pi: offset_theta += np.pi*2
                    # rospy.loginfo(f"{I.sphero_id}: {theta:1.2f} - {I.initial_heading:1.2f} = {offset_theta:1.2f}")
                    # 

                    q = quaternion_from_euler(0., 0., offset_theta)
                    p.pose.orientation = Quaternion(*q)
                    p.covariance = np.array([ # ignore covairance between factors, but the way we're doing this we get a lot of jitter, so x and y definitely have some variance. In pixels.
                        [3., 0., 0., 0., 0., 0.], # pixels
                        [0., 3., 0., 0., 0., 0.],  # pixels
                        [0., 0., 1e-6, 0., 0., 0.],
                        [0., 0., 0., 1e-6, 0., 0.],
                        [0., 0., 0., 0., 1e-6, 0.],
                        [0., 0., 0., 0., 0., 0.1], # radians
                        ]).flatten() # TODO
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


    