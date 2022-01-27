#!/usr/bin/env python3

'''
This is a quick template for any ros node.
@author james.staley625703@tufts.edu
'''

import traceback
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

def main():
    rospy.init_node("skelly")
    pub = rospy.Publisher("pub", Float32, queue_size=10)
    sub = rospy.Publisher("topic/sub", Pose2D, queue_size=10)

    while not rospy.is_shutdown(): # do work
        rospy.sleep(0.01) # sleep for messages and interrupts


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass