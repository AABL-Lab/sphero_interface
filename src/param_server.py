#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from sphero_interface.cfg import TrackerConfig

def callback(config, level):
    rospy.loginfo(f"Reconfigure Request: {[config]}")
    return config

if __name__ == "__main__":
    rospy.init_node("param_server", anonymous = False)

    srv = Server(TrackerConfig, callback)
    rospy.spin()
