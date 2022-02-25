#!/usr/bin/env python3

'''
Spoof file to produce fake sphero data for testing (written for transfer entropy testing)
@author james.staley625703@tufts.edu
'''

from math import pi, cos, sin
import traceback
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from sphero_interface.msg import SpheroNames
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt
import random

half_period = 25
increment = 1./half_period
TRAJECTORY_XYTHETA = [(increment*i, increment*i, pi/4.) for i in range(half_period)]
TRAJECTORY_XYTHETA.extend([(1., 1., -3*pi/4.)])
TRAJECTORY_XYTHETA.extend([(increment*(half_period-i), increment*(half_period-i), -3*pi/4.) for i in range(half_period)])

T2 = [(0.01*(100-i), 0.01*(100-i), -3*pi/4.) for i in range(100)]
T2.extend([(0., 0., pi/4.)])
T2.extend([(0.01*i, 0.01*i, pi/4.) for i in range(100)])

#[
    # [0, 0, 0],
    # [0.5, 0, 0],
    # [1, 0, 0],
    # [1, 0, pi/2],
    # [1, 0.5, pi/2],
    # [1, 1, pi/2],
    # [1, 1, pi],
    # [0.5, 1, pi],
    # [0, 1, pi],
    # [0, 1, 2*pi/3],
    # [0, 0.5, 2*pi/3],
    # [0, 0, 2*pi/3],
# ]

trajs = [TRAJECTORY_XYTHETA, T2]

def get_trajectory_pose(traj, index):
    x,y,theta = traj[index]
    return create_PWCS(x, y, theta)

def create_PWCS(x, y, theta):
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
    sphero_names = ["se9", "se8"]
    pubs = [rospy.Publisher(f"/{name}_ekf/odom_combined", PoseWithCovarianceStamped, queue_size=10) for name in sphero_names]
    namespub = rospy.Publisher("/sphero_names", SpheroNames, queue_size=1, latch=True)
    namespub.publish(SpheroNames(sphero_names))

    ax_range = [-0.05, 1.05]
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title("Spoofed Sphero Positions")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_ylim(ax_range)
    ax.set_xlim(ax_range)
    ax.plot()
    colors = ['b', 'darkorange']
    traj_idx = 0

    random_update_period, random_update_counter = 10, 0
    random_heading, random_vel = 0, 0
    random_x, random_y, random_theta = 0, 0, 0 

    while not rospy.is_shutdown(): # do work
        ax.clear()
        for idx, publisher in enumerate(pubs):
            if (idx == 1):
                random_update_counter += 1
                if random_update_counter % random_update_period == 0:
                    random_heading, random_vel = (random.random() * 2* pi, random.random() * 0.1)
                # Random heading and forward velocity periodically
                rx = random_x + (random_vel*cos(random_heading))
                ry = random_y + (random_vel*sin(random_heading))
                rx = max(min(1.0, rx), 0)
                ry = max(min(1.0, ry), 0)
                
                rtheta = random_heading
                point = create_PWCS(rx, ry, rtheta)
                random_x, random_y = rx, ry
            else:
                point = get_trajectory_pose(trajs[idx], traj_idx)
                
            publisher.publish(point)
            ax.scatter(point.pose.pose.position.x, point.pose.pose.position.y, c=colors[idx])
        traj_idx = traj_idx + 1
        if (traj_idx >= len(trajs[0])):
            traj_idx = 0

        ax.set_title("Spoofed Sphero Positions")
        ax.set_ylim(ax_range)
        ax.set_xlim(ax_range)
        ax.legend(sphero_names)
        fig.canvas.draw()
        fig.canvas.flush_events()
        rospy.sleep(0.1) # sleep for messages and interrupts


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass