#!/usr/bin/env python3
'''
Because of all the shenanigans around the sphero orientation, it makes the most sense to offset all the ekf positions in one place to clean up the rest of the nodes
@author james.staley625703@tufts.edu
'''

import math
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

from sphero_interface.msg import SpheroNames, HeadingStamped

corrected_ekf_pubs = dict()
ekf_subs = dict()
initial_headings_subs = dict()

initial_headings = dict()


def sphero_names_cb(msg: SpheroNames):
    '''
    Callback for the sphero names message.
    '''
    for name in msg.data:
        if name in corrected_ekf_pubs: 
            pass
        else:
            corrected_ekf_pubs[name] = rospy.Publisher(name+"/pose", Pose2D, queue_size=1)
            ekf_subs[name] = rospy.Subscriber(name+"_ekf/odom_combined", PoseWithCovarianceStamped, ekf_callback, callback_args=name)
            initial_headings_subs[name] = rospy.Subscriber(name+"/initial_heading", HeadingStamped, initial_heading_cb, callback_args=name)

def ekf_callback(msg: PoseWithCovarianceStamped, name: str):
    # dont care until we have initial headings
    if (name not in initial_headings):
        return

    r,p,y = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    offset_heading = initial_headings[name] + y
    while (offset_heading > math.pi): offset_heading -= 2*math.pi
    while (offset_heading < -math.pi): offset_heading += 2*math.pi

    corrected_ekf_pubs[name].publish(Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, offset_heading))

def initial_heading_cb(msg: HeadingStamped, name: str): # callback for initial heading
    initial_headings[name] = msg.theta

def main():
    rospy.init_node("ekf_corrections")
    rospy.Subscriber("/sphero_names", SpheroNames, sphero_names_cb)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass