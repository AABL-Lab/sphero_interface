#!/usr/bin/env python3
'''
Node to control an arbitrary number of spheros to their goals
@author james.staley625703@tufts.edu
'''

import string
import traceback
from copy import deepcopy
import time, math, random

from numpy import rad2deg, exp
from IPython import embed
from torch import rand

import rospy
import numpy as np
from rospy import Subscriber
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from sphero_interface.msg import HeadingStamped, SpheroNames, PositionGoal
from tf.transformations import euler_from_quaternion

import utils

VERBOSE = True

UPDATE_PERIOD = 0.2 # seconds for control loop
MIN_SPEED = 15
MAX_SPEED = 30
GOAL_THRESHOLD = 3 * rospy.get_param("/param_server/expected_sphero_radius", default=30) # How far we can be from a goal before its considered achieved
SLOWDOWN_DISTANCE = 5 * GOAL_THRESHOLD
TEST_SQUARE_LEN = 200
LOOP_TRAJECTORY = False

# keep some history
MAX_HISTORY_LENGTH = 20

'''
Parent class that handles all subscribers and publishers
'''
class TrajectoryFollowerGroup():
    def __init__(self) -> None:
        rospy.Subscriber("/sphero_names", SpheroNames, self.sphero_names_cb)
        
        self.cmd_publishers = dict()
        self.pose_subscribers = dict() # we dont really need to hold on to these but since they're autogenerated lets keep handles to them
        self.goal_subscribers = dict()
        self.priority_goal_subscribers = dict()
        self.initial_heading_subscribers = dict()

        self.initial_headings = dict()
        self.goal_poses = dict()
        self.priority_goal_poses = dict()

        # keep a short history of poses se 
        self.sphero_poses = dict()
        self.pose_ts = dict()

        self.task_complete_pub = rospy.Publisher("/task_complete", Bool, queue_size=1, latch=True)
        self.task_complete = False

    def sphero_names_cb(self, msg: SpheroNames):
        '''
        Callback for the sphero names message.
        '''
        for name in msg.data:
            if name in self.cmd_publishers: 
                pass
            else:
                self.cmd_publishers[name] = rospy.Publisher(name+"/cmd", HeadingStamped, queue_size=1)
                self.pose_subscribers[name] = rospy.Subscriber(name+"/pose", Pose2D, self.pose_callback, callback_args=name)
                self.goal_subscribers[name] = rospy.Subscriber(name+"/goal", PositionGoal, self.goal_callback, callback_args=name)
                self.priority_goal_subscriber = rospy.Subscriber("/priority_goal", PositionGoal, self.priority_goal_callback)
                self.initial_heading_subscribers[name] = rospy.Subscriber(name+"/initial_heading", HeadingStamped, self.initial_heading_callback, callback_args=name)

    def pose_callback(self, msg, name):
        if name not in self.sphero_poses.keys():
            self.sphero_poses[name] = [msg]
            self.pose_ts[name] = [time.time()]
        else:
            self.sphero_poses[name].append(msg)
            self.pose_ts[name].append(time.time())

        if len(self.sphero_poses[name]) > MAX_HISTORY_LENGTH:
            self.sphero_poses[name].pop(0)
            self.pose_ts[name].pop(0)

    def goal_callback(self, msg, name): self.goal_poses[name] = msg.goal # TODO: this should either have name as a callback arg or use the name in the messages, not both
    
    def priority_goal_callback(self, msg): # TODO: This is probably how all goals should work 
        self.priority_goal_poses[msg.sphero_name] = msg.goal # TODO: this should either have name as a callback arg or use the name in the messages, not both
    
    def initial_heading_callback(self, msg, name): self.initial_headings[name] = msg.theta

    def update(self):
        for name, priority_goal in self.priority_goal_poses.items():
            if name not in self.sphero_poses: 
                rospy.logwarn("PRIORITY Goal, but no pose for " + name)
                continue
            elif name not in self.initial_headings:
                rospy.logwarn("PRIORITY Goal, but no initial heading for " + name)
                continue

            if (time.time() - self.pose_ts[name][-1]) > 0.5: # late pose send stop
                rospy.logwarn("Stale pose data sending zero for " + name)
                cmd = HeadingStamped()
            else:
                cmd = self.cmd_for(name, priority_goal)
                if cmd.v == 0 and cmd.theta == 0: # we completed the priority goal HACK: send out a task complete signal
                    self.task_complete_pub.publish(True)
                    self.task_complete = True

            self.cmd_publishers[name].publish(cmd)
        
        for name, goal in self.goal_poses.items():
            if name not in self.sphero_poses: 
                rospy.logwarn("Goal, but no pose for " + name)
                continue
            elif name not in self.initial_headings:
                rospy.logwarn("Goal, but no initial heading for " + name)
                continue
            elif name in self.priority_goal_poses.keys():
                # Ignore goals for spheros that have priority goals
                continue

            if (time.time() - self.pose_ts[name][-1]) > 0.5: # late pose send stop
                rospy.logwarn("Stale pose data sending zero for " + name)
                cmd = HeadingStamped()
            else:
                cmd = self.cmd_for(name, goal)
            # cmd = self.cmd_for(self.sphero_poses[name], goal)
            self.cmd_publishers[name].publish(cmd)

    def distance_to_speed(self, distance_from_goal):
        '''
        Given a distance from the goal, returns the speed to use.
        '''
        if distance_from_goal > SLOWDOWN_DISTANCE:
            return MAX_SPEED
        else:
            return max(MIN_SPEED, MAX_SPEED * exp(- (SLOWDOWN_DISTANCE - distance_from_goal) / 50))

    def stuck_sphero(self, name):
        xvar, yvar = np.var([entry.x for entry in self.sphero_poses[name]]), np.var([entry.y for entry in self.sphero_poses[name]])
        return xvar < 5 and yvar < 5

    def cmd_for(self, name: string, goal_pose: Pose2D):
        cmd = HeadingStamped()
        curr_pose = self.sphero_poses[name][-1]
        initial_heading = self.initial_headings[name]
        distance_to_goal = utils.pose2d_distance(curr_pose, goal_pose)
        if (distance_to_goal < GOAL_THRESHOLD):
            if VERBOSE: rospy.loginfo(f"{name} at goal. distance: {distance_to_goal}")
        else:
            # Ys need to be flipped because of image coordinate system
            theta_goal = (math.atan2((-1*goal_pose.y) + curr_pose.y, goal_pose.x - curr_pose.x)) # NOTE: afaik we can't reset the sphero's heading through bluetooth
            theta_goal = utils.cap_0_to_2pi(theta_goal)

            # if VERBOSE: rospy.loginfo(f"target {theta_goal:1.1f} current {curr_pose.theta:1.1f}. distance_to_goal {distance_to_goal:1.2f}")

            diff_theta_world = utils.cap_npi_to_pi(abs(theta_goal - curr_pose.theta)) # dtheta in the world frame
            # diff_theta_local = diff_theta - initial_heading # dtheta in the sphero's frame
            # if VERBOSE: rospy.loginfo(f"dtheta_world {diff_theta:1.1f} from goal. dtheta_local {diff_theta_local:1.1f} from goal. th0 {initial_heading:1.1f}")

            adjusted_theta_goal = theta_goal - initial_heading
            # if VERBOSE: rospy.loginfo(f"target {theta_goal:1.1f} adjusted {adjusted_theta_goal:1.1f}. th0 {initial_heading:1.1f} distance_to_goal {distance_to_goal:1.2f}")

            cmd.v = self.distance_to_speed(distance_to_goal) if abs(diff_theta_world) < (math.pi / 3.) else 0 # Only move forward if we're mostly aligned with the goal
            cmd.theta = rad2deg(utils.cap_0_to_2pi(2*math.pi - adjusted_theta_goal)) # NOTE: The sphero treats clockwise as positive theta and counterclockwise as negative theta, so we're flipping it here so we can use a standard approach
            
            # if VERBOSE: rospy.loginfo(f"current x:{curr_pose.x:1.1f} y:{curr_pose.y:1.1f} theta:{curr_pose.theta:1.1f} goal {goal_pose.x:1.1f} {goal_pose.y:1.1f} {goal_pose.theta:1.1f} cmd {cmd.v:1.1f} {cmd.theta:1.1f} diff_theta_world {diff_theta_world:1.1f}")
        
            ## If we've been stuck for a while (on another sphero probably), alter the theta by 45 degrees for at least a step
            if self.stuck_sphero(name):
                # if VERBOSE: rospy.loginfo(f"{name} is stuck! Perturbing theta cmd.")
                cmd.theta += random.randint(-60, 60)
                cmd.theta = utils.cap_0_to_360(cmd.theta)

        return cmd

'''
Run a group of commanders that creates one sphero for each name on the topic
'''
def main():
    rospy.init_node("trajectory_follower")
    trajectory_follower_group = TrajectoryFollowerGroup()
    rospy.loginfo("Looping...")
    while not (rospy.is_shutdown()) and not trajectory_follower_group.task_complete:
        trajectory_follower_group.update()
        time.sleep(UPDATE_PERIOD)

    rospy.loginfo("Shutting down trajectory follower...")


if __name__ == "__main__":
    try:
    # main_single_commander():
        main()
    except rospy.ROSInterruptException:
        pass