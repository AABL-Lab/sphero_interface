#!/usr/bin/env python3

'''
opencv code by
Email: siddharthchandragzb@gmail.com
'''

from math import atan2, degrees
from re import S
import traceback
from turtle import circle
from IPython import embed
from plot_state import plot_spheros
from itertools import permutations

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
from TrackerParams import LOWER_GREEN, TRACK_WITH_CIRCLES, TRACK_WITH_COLOR, UPPER_GREEN, Sphero_Params_by_ID, Sphero_HSV_Color, Sphero_RGB_Color, TrackerParams, LOWER_WHITE, UPPER_WHITE

VERBOSE = True
SHOW_IMAGES = True

EXPECTED_SPHERO_RADIUS = 30 # size of spheros in pixels
circle_radiuses = dict()
frame_hsv = None

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
        print(f'({x},{y})-  (h,s,v) {H}, {S}, {V}')

ekf_pose2d = dict()
def ekf_cb(data, sphero_id):
    ekf_pose2d[sphero_id] = (data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2])
    # rospy.loginfo(f"{sphero_id} ekf_cb: {ekf_pose2d[sphero_id][0]:1.1f}, {ekf_pose2d[sphero_id][1]:1.1f}, {ekf_pose2d[sphero_id][2]:1.1f}")

goal_pose2d = dict()
def goal_cb(goal_pose, sphero_id):
    goal_pose2d[sphero_id] = goal_pose


detectors_dict = dict()
def sphero_names_cb(msg):
    for sphero_name in msg.data:
        if sphero_name not in detectors_dict:
            print(f"Adding {sphero_name} to detectors_dict")
            detectors_dict[sphero_name] = VisionDetect(sphero_name)

# Given the mask, whats the dominant color in expected led grid region?
def get_id_from_dominant_color(masked_hsv, possible_ids: list):
    closest_id = None
    mean_hue = -1
    mean_sat = -1
    if (len(detectors_dict) > 0):
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

        '''
        Mask each detected circle, check what color the grid is, and update the corresponding sphero
        '''
        global frame_hsv
        if TRACK_WITH_CIRCLES:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray_blurred, circles = I_generic.detectCircle(frame)
            height,width,depth = frame.shape
            circle_masks = []
            inner_circle_masks = [] # for clipping out the inner color

            if circles is not None:
                circles =  np.uint16(np.around(circles))
                for c in circles[0, :]:
                    # print(f"    {c}")
                    circle_mask = np.zeros((height,width), np.uint8)
                    cv2.circle(circle_mask, (c[0], c[1]), int(c[2]*1.1), (255,255,255), -1)
                    circle_masks.append((circle_mask, (c[0], c[1])))

                    inner_circle_mask = np.zeros((height,width), np.uint8)
                    cv2.circle(inner_circle_mask, (c[0], c[1]), int(c[2]*0.3), (255,255,255), -1)
                    inner_circle_masks.append((inner_circle_mask, (c[0], c[1])))

                circle_mask_viz = None
                inner_circle_mask_viz = None
                green_centers = []
                circle_centers = []
                avg_hues = []

                for_viz = []

                for (mask, (x,y)), (inner_mask, _) in zip(circle_masks, inner_circle_masks):
                # for mask, (x,y) in circle_masks:
                    circle_frame = cv2.bitwise_and(frame_hsv, frame_hsv, mask=mask)
                    green_mask, green_center = I_generic.processColor(circle_frame, lower=LOWER_GREEN, upper=UPPER_GREEN)
                    h,s,v = get_average_hsv(cv2.bitwise_and(frame_hsv, frame_hsv, mask=inner_mask))
                    if (green_center is not None) and s > 80:
                        print(f"{h:3.1f} {s:3.1f} {v:3.1f}")
                        cv2.line(frame, green_center, (x,y), (255,0,0), 2)
                        green_centers.append(green_center)
                        circle_centers.append((x,y))
                        avg_hues.append(h)
                        for_viz.append(inner_mask)

                        # for visualizing
                        if (circle_mask_viz is None):
                            circle_mask_viz = mask
                            inner_circle_mask_viz = inner_mask
                        else:
                            circle_mask_viz = cv2.bitwise_or(circle_mask_viz, mask)
                            inner_circle_mask_viz = cv2.bitwise_or(inner_circle_mask_viz, inner_mask)


                '''
                Now we have a set of detections, and a set of colors, and we have to make the best possible pairings. This is definitely a specific CS problem that has a known solution that I do not know.
                There's n**2 possible combos, but for small n lets just brute force it and pick the best one. 
                '''
                # print(f"{min(avg_hues):1.1f}")
                # NOTE: SO CONVOLUTED :(
                if len(avg_hues) == len(detectors_dict.keys()): # Only update when we see everyone (to avoid swapping)
                    # print(f"Trying to match {avg_hues} with {detectors_dict.keys()}")
                    # possible_keys = list(detectors_dict.keys())
                    # all_sphero_id_permutations = permutations(possible_keys)
                    # winning_permutation = None
                    # winning_score = float("inf") # low score is better
                    # print("==============================")
                    # for permutation in all_sphero_id_permutations:
                    #     score = 0
                    #     for avg_hue, sphero_id in zip(avg_hues, permutation):
                    #         score += abs(avg_hue - Sphero_HSV_Color[sphero_id][0])
                    #         print(f"{Sphero_HSV_Color[sphero_id][0]:1.1f}: {avg_hue:1.1f}")
                    #     if (score < winning_score):
                    #         print("WINNER")
                    #         winning_score = score
                    #         winning_permutation = permutation
                    #         # print(f"Winning permutation: {winning_permutation}")

                    # even simpler, just order them by hue
                    sorted_hues = sorted(zip(avg_hues, circle_centers, green_centers), key=lambda x: x[0])
                    sorted_ids = sorted(zip(detectors_dict.keys(), [Sphero_HSV_Color[key] for key in detectors_dict.keys()]), key=lambda x: x[1][0])
                    # now we should have centers and sphero ids ordered from least hue (observers and expected, respectively) to most hue
                    for (hue, (centerx, centery), (greenx, greeny)), (sphero_id, _) in zip(sorted_hues, sorted_ids):
                        theta = atan2(-greeny + centery, greenx - centerx) # Image coords need to have y swapped
                        # print(f"{sphero_id}: matched to {hue:1.1f}. Pose: {centerx:1.0f}, {centery:1.0f}, {theta:1.2f}. Green: {greenx:1.0f}, {greeny:1.0f} - ")
                        detectors_dict[sphero_id].set_detected_position(centerx, centery, theta)
                    # print("="*20)
                    ''''''
                else: # TODO
                    pass

                # Look for the green directions on the circles
                if circle_mask_viz is not None:
                    circle_frame = cv2.bitwise_and(frame, frame, mask=circle_mask_viz)
                    inner_circle_frame = cv2.bitwise_and(frame, frame, mask=inner_circle_mask_viz)
                    cv2.imshow("circles", circle_frame)
                    cv2.imshow("inner_circles", inner_circle_frame)
        ''''''


        if (SHOW_IMAGES):
            cv2.imshow("image", frame)
            cv2.setMouseCallback('image', mouse_cb)

        if (SHOW_IMAGES and ekf_pose2d):
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
        # pose_imgs = []
        # sphero_ids = []
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
                # rospy.loginfo(f"{I.sphero_id} publishing odom msg {odom_msg}")
                I.odom_pub.publish(odom_msg)
                I.last_detected_color_pose = None
        #         pose_imgs.append(pose_img)
        #         sphero_ids.append(I.sphero_id)
        # plot_spheros(pose_imgs, sphero_ids, ax_x_range=[0, frame.shape[1]], ax_y_range=[frame.shape[0], 0])
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


    