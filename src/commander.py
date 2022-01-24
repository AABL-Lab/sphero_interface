#!/usr/bin/env python3

'''
Simple sphero commander
@author james.staley625703@tufts.edu
'''

import traceback
from copy import deepcopy
import time, math

from numpy import rad2deg
from IPython import embed

import rospy
from rospy import Subscriber
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from sphero_interface.msg import HeadingStamped

import dwa_local_planner as dwa

SPEED = 30
USE_DEAD_RECKONING = True
GOAL_THRESHOLD = 0.1 # How far we can be from a goal before its considered achieved

TEST_SQUARE_LEN = 0.2

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
    def __init__(self, sphero_id="sD9"):
        self.sphero_id = sphero_id
        # <<<<<<<<<<< path to follow
        self.path_x_y = [Pose2D(x, y, 0) for x,y in [
            (0., 0.),
            (TEST_SQUARE_LEN, 0),
            (0, 0),
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
        self.dead_reckoning_pose = Pose2D()
        self.prev_cmd_time = None

        self.pub = rospy.Publisher("sD9/cmd", HeadingStamped, queue_size=5)
        self.tracker_sub = rospy.Subscriber("sD9/prev_cmd", HeadingStamped, self.tracker_cb, queue_size=20)

    def is_complete(self):
        return self.path_complete

    def get_command(self, curr_pose, goal_pose):
        '''
        TODO: This makes sense as an actionserver 
        '''
        theta_goal = self.pose.theta - math.atan2(goal_pose.y - curr_pose.y, goal_pose.x - curr_pose.x)
        theta_goal = rad2deg(theta_goal)
        while theta_goal < 0: theta_goal += 360.
        while theta_goal > 360: theta_goal -= 360.

        cmd = HeadingStamped()
        cmd.v = SPEED
        cmd.theta = theta_goal
        self.pub.publish(cmd)
        print(f"current {curr_pose.x:1.2f} {curr_pose.y:1.2f} {curr_pose.theta:1.2f} goal {goal_pose.x:1.2f} {goal_pose.y:1.2f} {goal_pose.theta:1.2f} cmd {cmd.v:1.2f} {cmd.theta:1.2f}")
        return cmd

    def tracker_cb(self, data: HeadingStamped):
        '''
        Track dead reckoning pose using the messsages that are actually sent to the sphero.
        '''
        if not self.prev_cmd_time:
            pass
        else:
            dt = data.t - self.prev_cmd_time
            v = speed_to_mps(data.v)
            self.dead_reckoning_pose.x += v * math.cos(data.theta) * dt
            self.dead_reckoning_pose.y += v * math.sin(data.theta) * dt
            self.dead_reckoning_pose.theta = data.theta

            if (USE_DEAD_RECKONING):
                self.set_tracked_position(self.dead_reckoning_pose)

        self.prev_cmd_time = data.t

    def set_tracked_position(self, pose):
        '''
        Set the pose of this commander's sphero. 
        '''
        # rospy.loginfo(f"Setting pose of {self.sphero_id} to {pose}")
        self.pose = pose

    def trajectory_step(self):
        if (self.is_complete()): return

        goal_pose = self.path_x_y[self.trajectory_idx]
        if (pose2d_distance(goal_pose, self.pose) < GOAL_THRESHOLD):
            self.trajectory_idx += 1
            if (self.trajectory_idx >= len(self.path_x_y)): # done
                self.path_complete = True 
                return
            else:
                goal_pose = self.path_x_y[self.trajectory_idx]
                print(f"new goal {goal_pose.x:1.2f} {goal_pose.y:1.2f}")


        cmd = self.get_command(self.pose, goal_pose)
        command_history.append(cmd)

    def step(self):
        self.trajectory_step()
        pose_history.append(deepcopy(self.dead_reckoning_pose))
        # rospy.loginfo(f"Dead Reckoned Pose (x, y, theta): {self.dead_reckoning_pose.x:1.2f} {self.dead_reckoning_pose.y:1.2f} {self.dead_reckoning_pose.theta:1.2f}")



def main():
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
        main()
    except rospy.ROSInterruptException:
        pass