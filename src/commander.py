#!/usr/bin/env python3

'''
Simple sphero commander
@author james.staley625703@tufts.edu
'''

import traceback
from copy import deepcopy
import rospy
import time
import math
from rospy import Subscriber
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from sphero_interface.msg import HeadingStamped

command_history = []
pose_history = []

class Commander():
    def __init__(self):
        # <<<<<<<<<<< path to follow
        self.path_t_v_theta = [HeadingStamped(t, y, theta) for t,y,theta in [
            (0.5, 0, 0),
            (0.5, 100, 0),
            (0.5, 0, 90),
            (0.5, 100, 90),
            (0.5, 0, 180),
            (0.5, 100, 180),
            (0.5, 0, 270),
            (0.5, 100, 270)
            ]]
        self.complete = False
        self.cmd_idx = 0
        self.t0 = time.time()
        # >>>>>>>>>>>

        self.dead_reckoning_pose = Pose2D()
        self.prev_cmd_time = None

        self.pub = rospy.Publisher("sD9/cmd", HeadingStamped, queue_size=5)
        self.tracker_sub = rospy.Subscriber("sD9/prev_cmd", HeadingStamped, self.tracker_cb, queue_size=20)

    def is_complete(self):
        return self.complete

    def get_command(self):
        if (self.cmd_idx >= len(self.path_t_v_theta)):
            self.complete = True
            return HeadingStamped(0, 0, 0)

        goal = self.path_t_v_theta[self.cmd_idx]
        if (time.time() - self.t0) > goal.t:
            self.cmd_idx += 1
            self.t0 = time.time()
        return goal

    def tracker_cb(self, data: HeadingStamped):
        '''
        Tracking the messsages that are actually sent to the sphero for dead reckoning test.
        This not actually tracking position.
        '''
        if not self.prev_cmd_time:
            pass
        else:
            dt = data.t - self.prev_cmd_time
            self.dead_reckoning_pose.x += data.v * math.cos(data.theta) * dt
            self.dead_reckoning_pose.y += data.v * math.sin(data.theta) * dt
            self.dead_reckoning_pose.theta = data.theta
        self.prev_cmd_time = data.t

    def step(self):
        cmd = self.get_command()

        command_history.append(cmd)
        self.pub.publish(cmd)

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
        rospy.sleep(0.05) # sleep for messages and interrupts


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass