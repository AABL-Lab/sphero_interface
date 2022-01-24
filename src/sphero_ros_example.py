#!/usr/bin/env python3
# Author Jindan Huang


from time import sleep
import os
from pysphero.device_api.user_io import Color
from pysphero.core import Sphero
from pysphero.driving import Direction
import random
import roslibpy
import rospy
from std_msgs.msg import Float32


# Method1: communicate using roslibpy

def sphero_ros_connect1(mac_address):

    # Run roslaunch rosbridge_server rosbridge_websocket.launch first to create a WebSocket
    roslibpy_client = roslibpy.Ros(host="localhost", port=9090)  # default port 9090
    roslibpy_client.run()
    talker = roslibpy.Topic(roslibpy_client, '/ambient_light', 'std_msgs/Float32')


    with Sphero(mac_address=mac_address) as sphero:
        sphero.power.wake()
        sphero.user_io.set_all_leds_8_bit_mask(back_color=Color(red=0xff))
        sphero.user_io.set_led_matrix_text_scrolling(string="CATCH", color=Color(red=0xff))
        speed = 15
        heading = 90
        sphero.driving.drive_with_heading(speed, heading, Direction.forward)
        print("Ambient light sensor value:" + str(sphero.sensor.get_ambient_light_sensor_value()))

        # publish sensor data to ROS topic
        # you can see published data if you echo the corresponding ros topic
        talker.publish(roslibpy.Message({'data':sphero.sensor.get_ambient_light_sensor_value()}))
        sphero.power.enter_soft_sleep()
        



# Method2: communicate using rospy
def sphero_ros_connect2(mac_address):

    # Run roscore first
    rospy_client = rospy.Publisher('/ambient_light', Float32, queue_size=10)
    rospy.init_node('sphero_sensors')
    rate = rospy.Rate(10)

    with Sphero(mac_address=mac_address) as sphero:
        sphero.power.wake()
        sphero.user_io.set_all_leds_8_bit_mask(back_color=Color(red=0xff))
        sphero.user_io.set_led_matrix_text_scrolling(string="CATCH", color=Color(red=0xff))
        speed = 15
        heading = 90
        sphero.driving.drive_with_heading(speed, heading, Direction.forward)
        
        light_data = sphero.sensor.get_ambient_light_sensor_value()
        print("Ambient light sensor value:" + str(light_data))

        # publish sensor data to ROS topic
        rospy_client.publish(light_data)
        rospy.loginfo(str(light_data))
        rate.sleep()
        sphero.power.enter_soft_sleep()


if __name__ == "__main__":
    mac_address = "EC:73:F2:19:0E:CA"
    sphero_ros_connect1(mac_address)
    # sphero_ros_connect2(mac_address)
