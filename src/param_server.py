#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
import yaml
from sphero_interface.cfg import TrackerConfig

from IPython import embed

def callback(config, level):
    rospy.loginfo(f"Reconfigure Request: {[config]}")
    return config

if __name__ == "__main__":
    rospy.init_node("param_server", anonymous = False)

    srv = Server(TrackerConfig, callback)

    # client = Client("param_server")

    # fn = "config/working_tracking.yaml"
    # with open(fn, "r") as stream:
    #     try:
    #         print(yaml.safe_load(stream))
    #     except yaml.YAMLError as exc:
    #         print(exc)
    
    # embed()
    # client.update_configuration()
    rospy.spin()
