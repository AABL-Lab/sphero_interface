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
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from sphero_interface.msg import SpheroNames, PositionGoal
from geometry_msgs.msg import Pose2D
from copy import deepcopy
import numpy as np

w_range = (rospy.get_param("/tracker_params/min_width", 150.), rospy.get_param("/tracker_params/max_width", 800.))
h_range = (rospy.get_param("/tracker_params/min_height", 75.), rospy.get_param("/tracker_params/max_height", 450))
GOAL_THRESHOLD = 3 * rospy.get_param("/param_server/expected_sphero_radius", default=30) # How far we can be from a goal before its considered achieved
PRESET_GOALS = {} #{"sd1": [(350, 200), (750, 200)]}
PRESET_GOALS_IDX = {"sd1": 0}

active_goals = dict()
current_poses = dict()
goal_publishers = dict()
pose_subscribers = dict()

INTERACTION_ZONE = (rospy.get_param("/param_server/x_interaction_zone", default=470), rospy.get_param("/param_server/y_interaction_zone", default=80))
PRECOMPUTE_GOALS = True
precomputed_goals = []
if PRECOMPUTE_GOALS:
    MIN_DISTANCE_TO_IZ = rospy.get_param("/param_server/min_distance_to_iz", default=80)
    xres, yres = 40, 40 # x and y resolutions of random goals, the total number of goals will be < xres * yres because we're not using an goals within a radius of the interaction zone 
    for x in np.linspace(w_range[0], w_range[1], xres):
        for y in np.linspace(h_range[0], h_range[1], yres):
            r = np.sqrt((x-INTERACTION_ZONE[0])**2 + (y-INTERACTION_ZONE[1])**2)
            if (r >= MIN_DISTANCE_TO_IZ):
                precomputed_goals.append(Pose2D(x, y, 0))

# Show the valid goals
# import matplotlib.pyplot as plt
# plt.scatter([pt.x for pt in precomputed_goals], [pt.y for pt in precomputed_goals])
# plt.show()


def select_random_goal():
    goal = None
    if PRECOMPUTE_GOALS:
        goal = random.choice(precomputed_goals)
    else:
        goal = Pose2D(random.randint(*w_range), random.randint(*h_range), 0)
    return goal 

def main():
    rospy.init_node("goal_generator")
    rospy.sleep(1.0)
    rospy.loginfo("looping..")

    rospy.Subscriber("/start", Bool, reset_cb)
    rospy.Subscriber("/reset_experiment", Bool, reset_cb)

    spheronames = rospy.wait_for_message("/sphero_names", SpheroNames)
    sphero_names_cb(spheronames)
    # rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
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
                    rgoal = select_random_goal()
                    idiot_idx = 0
                    while any([(utils.pose2d_distance(rgoal, g.goal) < 100) for g in goals]):
                        rgoal = select_random_goal()
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

def reset_cb(msg):
    rospy.loginfo("GM: Resetting.")
    rospy.sleep(0.5)
    for key in active_goals.keys():
        active_goals[key] = None

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