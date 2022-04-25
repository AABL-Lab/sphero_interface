#!/usr/bin/env python3

'''
Generate goals for the trajectory follower
@author james.staley625703@tufts.edu
'''

import traceback

from pyrfc3339 import generate
import rospy
import random
import utils
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from sphero_interface.msg import SpheroNames, PositionGoal
from geometry_msgs.msg import Pose2D
from copy import deepcopy

w_range = (rospy.get_param("/tracker_params/min_width", 150.), rospy.get_param("/tracker_params/max_width", 800.))
h_range = (rospy.get_param("/tracker_params/min_height", 75.), rospy.get_param("/tracker_params/max_height", 450))
GOAL_THRESHOLD = 3 * rospy.get_param("/param_server/expected_sphero_radius", default=30) # How far we can be from a goal before its considered achieved


PRESET_GOALS = {} #{"sd1": [(350, 200), (750, 200)]}
PRESET_GOALS_IDX = {"sd1": 0}

active_goals = dict()
current_poses = dict()
goal_publishers = dict()
pose_subscribers = dict()

def generate_random_goal():
    return Pose2D(random.randint(*w_range), random.randint(*h_range), 0)

def main():
    rospy.init_node("goal_generator")
    rospy.sleep(1.0)
    rospy.loginfo("looping..")
    rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
    while not rospy.is_shutdown(): # do work
        goals = [entry for entry in deepcopy(list(active_goals.values())) if entry is not None]
        for k,v in active_goals.items():
            if v is None:
                if (k in PRESET_GOALS):
                    p_goals, p_idx = PRESET_GOALS[k], PRESET_GOALS_IDX[k]
                    p_idx = p_idx % len(p_goals)
                    PRESET_GOALS_IDX[k] += 1
                    new_goal = PositionGoal(k, Pose2D(p_goals[p_idx][0], p_goals[p_idx][1], 0))
                else:
                    rgoal = generate_random_goal()
                    idiot_idx = 0
                    while any([(utils.pose2d_distance(rgoal, g.goal) < 100) for g in goals]):
                        rgoal = generate_random_goal()
                        idiot_idx += 1
                        if idiot_idx > 100: rospy.logwarn(f"you wrote this wrong...")

                    new_goal = PositionGoal(k, rgoal)
                name = k
                
                rospy.loginfo(f"Publishing new goal {new_goal.goal.x:1.1f} {new_goal.goal.y:1.1f} for {name}")
                goal_publishers[k].publish(new_goal)
                active_goals[k] = new_goal
            else:
                if utils.pose2d_distance(current_poses[k], v.goal) < GOAL_THRESHOLD:
                    rospy.loginfo(f"{k} reached goal {v.goal.x:1.1f} {v.goal.y:1.1f}")
                    active_goals[k] = None

        rospy.sleep(1.0) # sleep for messages and interrupts

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