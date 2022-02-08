#!/usr/bin/env python3

'''
Spoof file to produce fake sphero data for testing (written for transfer entropy testing)
@author james.staley625703@tufts.edu
'''

from cmath import pi
import traceback
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

TRAJECTORY_XYTHETA = [
    [0, 0, 0],
    [0.5, 0, 0],
    [1, 0, 0],
    [1, 0, pi/2],
    [1, 0.5, pi/2],
    [1, 1, pi/2],
    [1, 1, pi],
    [0.5, 1, pi],
    [0, 1, pi],
    [0, 1, 2*pi/3],
    [0, 0.5, 2*pi/3],
    [0, 0, 2*pi/3],
]
TRAJECTORY_INDEX = 0

def get_trajectory_pose():
    global TRAJECTORY_INDEX, TRAJECTORY_XYTHETA

    if (TRAJECTORY_INDEX >= len(TRAJECTORY_XYTHETA)):
        TRAJECTORY_INDEX = 0

    x,y,theta = TRAJECTORY_XYTHETA[TRAJECTORY_INDEX]
    TRAJECTORY_INDEX += 1

    p = PoseWithCovarianceStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "odom"
    p.pose.pose.position.x = x
    p.pose.pose.position.y = y
    p.pose.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, theta)
    p.pose.pose.orientation.x = q[0]
    p.pose.pose.orientation.y = q[1]
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]
    
    return p

def main():
    rospy.init_node("spoof")
    pub = rospy.Publisher("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, queue_size=10)

    while not rospy.is_shutdown(): # do work
        pub.publish(get_trajectory_pose())
        rospy.sleep(0.1) # sleep for messages and interrupts


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass