#!/usr/bin/env python3

'''
Generate goals for the trajectory follower
@author james.staley625703@tufts.edu
'''

import traceback
import rospy
import random
import utils
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from sphero_interface.msg import SpheroNames, PositionGoal
from geometry_msgs.msg import Pose2D

w_range = (rospy.get_param("/tracker_params/min_width", 50.), rospy.get_param("/tracker_params/max_width", 550))
h_range = (rospy.get_param("/tracker_params/min_height", 50.), rospy.get_param("/tracker_params/max_height", 350))

active_goals = dict()
current_poses = dict()
goal_publishers = dict()
pose_subscribers = dict()
def main():
    rospy.init_node("goal_generator")
    rospy.loginfo("looping..")
    rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
    while not rospy.is_shutdown(): # do work
        for k,v in active_goals.items():
            if v is None:
                new_goal = PositionGoal(k, Pose2D(random.randint(*w_range), random.randint(*h_range), 0))
                goal_publishers[k].publish(new_goal)
                active_goals[k] = new_goal
            else:
                if utils.pose2d_distance(current_poses[k], v.goal) < 0.1:
                    rospy.loginfo(f"{k} reached goal {v.goal}")
                    active_goals[k] = None

        rospy.sleep(0.1) # sleep for messages and interrupts

def sphero_names_cb(msg: SpheroNames):
    '''
    Callback for the sphero names message.
    '''
    for name in msg.data:
        if name in goal_publishers:
            pass
        else:
            rospy.loginfo(f"Setting up goal publisher for {name}")
            active_goals[name] = None
            current_poses[name] = Pose2D()
            goal_publishers[name] = rospy.Publisher(name+"/goal", PositionGoal, queue_size=1)
            pose_subscribers[name] = rospy.Subscriber(name+"/pose", Pose2D, pose_cb, callback_args=name)


def pose_cb(msg: Pose2D, name: str):
    '''
    Callback for the pose message.
    '''
    current_poses[name] = msg


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass