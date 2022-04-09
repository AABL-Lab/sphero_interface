#!/usr/bin/env python3

'''
quickie to turn poses into a bunch of odom_combined
@author james.staley625703@tufts.edu
'''

import traceback
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseWithCovariance, Point, TwistWithCovariance, Point, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import quaternion_from_euler
POSITION_COVARIANCE = 3.0
ORIENTATION_COVARIANCE = 0.1

def pose_cb(msg, id):
    x,y,theta = msg.x, msg.y, msg.theta
    global pubs
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    p = PoseWithCovariance()
    p.pose.position = Point(x,y,0)

    # Calculate the offset theta considering the initial heading of the sphero. This must be done so the ekf sees consistent data between the orientation published in interface.py and this detection

    q = quaternion_from_euler(0., 0., theta)
    p.pose.orientation = Quaternion(*q)
    p.covariance = np.array([ # ignore covairance between factors, but the way we're doing this we get a lot of jitter, so x and y definitely have some variance. In pixels.
        [POSITION_COVARIANCE, 0., 0., 0., 0., 0.], # pixels
        [0., POSITION_COVARIANCE, 0., 0., 0., 0.],  # pixels
        [0., 0., 1e-6, 0., 0., 0.],
        [0., 0., 0., 1e-6, 0., 0.],
        [0., 0., 0., 0., 1e-6, 0.],
        [0., 0., 0., 0., 0., ORIENTATION_COVARIANCE], # radians
        ]).flatten() # TODO
    tw = TwistWithCovariance() # TODO
    odom_msg.pose = p
    odom_msg.twist = tw

    pubs[id].publish(odom_msg)

def main():
    rospy.init_node("bag_remap")

    global pubs

    ids = ["sec", "sd1"]
    subs = [rospy.Subscriber(f"{id}/pose", Pose2D, queue_size=1, callback=pose_cb, callback_args=id) for id in ids]
    pubs = {id: rospy.Publisher(f"{id}/odom", Odometry, queue_size=1) for id in ids}

    while not rospy.is_shutdown(): # do work
        rospy.sleep(0.01) # sleep for messages and interrupts


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass