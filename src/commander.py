#!/usr/bin/env python3

'''
Simple sphero commander
@author james.staley625703@tufts.edu
'''

import traceback
from copy import deepcopy
import time, math, random

from numpy import rad2deg
from IPython import embed

import rospy
from rospy import Subscriber
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from sphero_interface.msg import HeadingStamped, SpheroNames
from tf.transformations import euler_from_quaternion

SPEED = 30
GOAL_THRESHOLD = 50 # How far we can be from a goal before its considered achieved
TEST_SQUARE_LEN = 200
LOOP_TRAJECTORY = False

command_history = []
pose_history = []

def pose2d_distance(a, b):
    return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 )

def speed_to_mps(speed):
    '''
    Convert a 0-255 sphero speed command to meters-per-second
    '''
    return (speed/255) * 2.01168 # max speed (4.5 miles per hour)

class Commander():
    def __init__(self, sphero_id="sd9"):
        self.sphero_id = sphero_id
        # <<<<<<<<<<< path to follow
        self.path_x_y = [Pose2D(x, y, 0) for x,y in [
            (TEST_SQUARE_LEN, TEST_SQUARE_LEN),
            (2*TEST_SQUARE_LEN, TEST_SQUARE_LEN),
            (2*TEST_SQUARE_LEN, 1.5*TEST_SQUARE_LEN),
            (TEST_SQUARE_LEN, 1.5*TEST_SQUARE_LEN),
            (TEST_SQUARE_LEN, TEST_SQUARE_LEN),
            # (0, TEST_SQUARE_LEN),
            # (TEST_SQUARE_LEN, TEST_SQUARE_LEN),
            # (0, TEST_SQUARE_LEN),
            # (0, 0)
            ]]
        self.path_complete = False
        self.trajectory_idx = 0
        self.t0 = time.time()
        # >>>>>>>>>>>

        self.pose = Pose2D()
        self.theta_array, self.max_theta_length = [], 10 # smooth it out
        self.initial_theta = None

        self.pub = rospy.Publisher(sphero_id+"/cmd", HeadingStamped, queue_size=1)
        self.goal_pub = rospy.Publisher(sphero_id+"/goal", Pose2D, queue_size=1)

        # Initialize the sphero's orientation to what it considers to be 0
        for i in range(10):
            self.pub.publish(HeadingStamped(v=0, theta=0))
            rospy.sleep(0.05)

        self.ekf_sub = rospy.Subscriber(sphero_id+"_ekf/odom_combined", PoseWithCovarianceStamped, self.ekf_callback)

    def is_complete(self):
        return self.path_complete

    def get_command(self, curr_pose, goal_pose):
        '''
        TODO: This makes sense as an actionserver 
        '''
        # Ys need to be flipped because of image coordinate system
        theta_goal = (math.atan2((-1*goal_pose.y) + curr_pose.y, goal_pose.x - curr_pose.x)) # NOTE: afaik we can't reset the sphero's heading through bluetooth
        
        print(f"goal_to_theta {theta_goal:1.2f} th0 {self.initial_theta:1.2f}")
        theta_goal = self.initial_theta - theta_goal


        theta_goal = theta_goal
        current_theta_local = self.initial_theta - self.pose.theta

        while theta_goal < 0: theta_goal += 2*math.pi
        while theta_goal > 2*math.pi: theta_goal -= 2*math.pi
        while current_theta_local < 0: current_theta_local += 2*math.pi
        while current_theta_local > 2*math.pi: current_theta_local -= 2*math.pi

        diff_theta = abs(theta_goal - current_theta_local)
        print(f"diff_theta {diff_theta:1.2f}")

        cmd = HeadingStamped()
        cmd.v = SPEED if diff_theta < (math.pi / 4.) else 0
        # cmd.v = 0
        cmd.theta = rad2deg(theta_goal)
        
        self.pub.publish(cmd)
        print(f"current {curr_pose.x:1.2f} {curr_pose.y:1.2f} {curr_pose.theta:1.2f} goal {goal_pose.x:1.2f} {goal_pose.y:1.2f} {goal_pose.theta:1.2f} cmd {cmd.v:1.2f} {cmd.theta:1.2f}")
        return cmd

    def set_tracked_position(self, pose):
        '''
        Set the pose of this commander's sphero. 
        '''
        # rospy.loginfo(f"Setting pose of {self.sphero_id} to {pose}")
        self.pose = pose

    def trajectory_step(self):
        if (self.is_complete()): return

        goal_pose = self.path_x_y[self.trajectory_idx]
        self.goal_pub.publish(Pose2D(goal_pose.x, goal_pose.y, 0))
        distance_to_goal = pose2d_distance(self.pose, goal_pose)
        print(f"distance_to_goal {distance_to_goal:1.2f}")
        if (distance_to_goal < GOAL_THRESHOLD):
            self.trajectory_idx += 1
            if (self.trajectory_idx >= len(self.path_x_y)): # done
                self.path_complete = True
                if (LOOP_TRAJECTORY):
                    self.trajectory_idx = 0
            else: # new goal
                goal_pose = self.path_x_y[self.trajectory_idx]
                print(f"new goal {goal_pose.x:1.2f} {goal_pose.y:1.2f}")


        if (self.path_complete):
            cmd = HeadingStamped()
        else:
            cmd = self.get_command(self.pose, goal_pose)
        # cmd = HeadingStamped()
        command_history.append(cmd)
        self.pub.publish(cmd)

    def random_orientation(self):
        '''
        Randomly orient the sphero
        '''
        cmd = HeadingStamped()
        cmd.v = 0
        cmd.theta = random.randint(0, 360)
        
        self.pub.publish(cmd)

    def step(self):
        self.random_orientation()
        # self.trajectory_step()

    def ekf_callback(self, msg):
        euler = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])
        self.theta_array.append(pose.theta)
        self.pose = pose
        if (self.initial_theta is None):
            print(f"{self.sphero_id} initial theta {self.pose.theta}")
            self.initial_theta = self.pose.theta

        # else:
        #     print(f"{self.sphero_id} current theta {self.pose.theta}")

all_commanders = []
def sphero_names_cb(msg: SpheroNames):
    '''
    Callback for the sphero names message.
    '''
    for name in msg.data:
        if any([name == c.sphero_id for c in all_commanders]): continue # we're tracking this one
        rospy.loginfo("Creating commander for " + name)
        all_commanders.append(Commander(name))

'''
Run a group of commanders that creates one sphero for each name on the topic
'''
def main_group():
    rospy.init_node("sphero_commander")
    rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
    while not rospy.is_shutdown(): # do work
        for commander in all_commanders:
            if commander.initial_theta is None:
                continue # can't plan without knowing sphero initial headings
            
            commander.step()
            # if commander.is_complete():
            #     for cmd, pose in zip(command_history, pose_history):
            #         print(f"{cmd.t:1.2f}s {cmd.v} {cmd.theta} -> {pose.x:1.2f} {pose.y:1.2f} {pose.theta:1.2f}")
            #     break
        rospy.sleep(1.0) # sleep for messages and interrupts

'''
Run the single commander to control on sphero.
'''
def main_single_commander():
    rospy.init_node("sphero_commander")
    commander = Commander()

    while not rospy.is_shutdown(): # do work
        commander.step()
        if commander.is_complete():
            for cmd, pose in zip(command_history, pose_history):
                print(f"{cmd.t:1.2f}s {cmd.v} {cmd.theta} -> {pose.x:1.2f} {pose.y:1.2f} {pose.theta:1.2f}")
            break
        rospy.sleep(1.0) # sleep for messages and interrupts


if __name__ == "__main__":
    try:
    # main_single_commander():
        main_group()
    except rospy.ROSInterruptException:
        pass