#!/usr/bin/env python3

'''
This is a quick template for any ros node.
@author james.staley625703@tufts.edu
'''

import traceback
import rospy
from std_msgs.msg import Float32
from sphero_interface.msg import HeadingStamped, SpheroNames
from geometry_msgs.msg import Pose2D
import random
import time

cmd_pubs = {}
def main():
    rospy.init_node("random_orientations")
    rospy.Subscriber("sphero_names", SpheroNames, sphero_names_cb)

    rospy.sleep(2.0)
    while not rospy.is_shutdown(): # do work
        for k,v in cmd_pubs.items():
            rand_theta = random.randint(0,360)
            v.publish(HeadingStamped(rospy.get_time(), 0.0, rand_theta))
            print(f"sending theta: {rand_theta:1.1f} for {k}.") 
            time.sleep(random.random()*1.5) # sleep for messages and interrupts
        # rospy.sleep(random.random()*1.5) # sleep for messages and interrupts

def sphero_names_cb(msg: SpheroNames):
    '''
    Callback for the sphero names message.
    '''
    for name in msg.data:
        if name in cmd_pubs:
            pass
        else:
            rospy.loginfo(f"Setting up goal publisher for {name}")
            cmd_pubs[name] = rospy.Publisher(name+"/cmd", HeadingStamped, queue_size=1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass