# from pysphero.bluetooth.ble_adapter import AbstractBleAdapter
#!/usr/bin/env python3
# Author Jindan Huang

from time import sleep
import os
from pysphero.constants import Toy
from pysphero.device_api.user_io import Color
from pysphero.utils import toy_scanner

import random
from pysphero.core import Sphero
from pysphero.driving import Direction, StabilizationIndex

from std_msgs.msg import Float32
import rospy
# import roslibpy

# def listener_callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     # return data.data
#
# def sphero_listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("chatter", String, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

def get_mac_address():
    with toy_scanner(toy_type=Toy.sphero_bolt) as sphero:
        print(f"Found {sphero.mac_address}")
        # find toy: scan and get mac address from sphero bolt
        # print("Found" + sphero.mac_address)
        sphero.power.wake()
        sleep(2)
        sphero.power.enter_soft_sleep()


def main():
    rospy.init_node("sphero_interface")
    mac_address = "D9:81:9E:B8:AD:DB"

    # client = roslibpy.Ros(host="localhost", port=9090)
    # client.run()

    # talker = roslibpy.Topic(client, '/ambient_light', 'std_msgs/Float32')

    pub = rospy.Publisher("blah/test", Float32)

    with Sphero(mac_address=mac_address) as sphero:
        sphero.power.wake()
        # sphero.user_io.set_all_leds_8_bit_mask(back_color=Color(red=0xff))
        # sphero.user_io.set_led_matrix_one_color(color=Color(red=0xff))
        sphero.user_io.set_led_matrix_text_scrolling(string="CATCH", color=Color(red=0xff))
        for _ in range(10):
            speed = random.randint(50, 100)
            heading = random.randint(0, 360)
            print(f"Send drive with speed {speed} and heading {heading}")
            sphero.driving.drive_with_heading(speed, heading, Direction.forward)
            
            print("Ambient light sensor value:" + str(sphero.sensor.get_ambient_light_sensor_value()))
            # talker.publish(roslibpy.Message({'data':sphero.sensor.get_ambient_light_sensor_value()}))
            pub.publish(sphero.sensor.get_ambient_light_sensor_value())

        sphero.power.enter_soft_sleep()

def pure_sphero():
    mac_address = "D9:81:9E:B8:AD:DB"
    with Sphero(mac_address=mac_address) as sphero:
        print(f"Connected to {mac_address}")
        sphero.power.wake()
        sphero.user_io.set_all_leds_8_bit_mask(front_color=Color(blue=255), back_color=Color(red=255))
        # sphero.user_io.set_led_matrix_single_character(character='C', color=Color(red=255))
        sphero.driving.set_stabilization(StabilizationIndex.no_control_system)
        sphero.user_io.set_led_matrix_one_color(color=Color(red=0xff))
        # sphero.user_io.set_led_matrix_text_scrolling(string="CATCH", color=Color(red=0xff))
        try:
            while True:
                sleep(0.05)
        except KeyboardInterrupt:
            pass

        print("Sending sphero to sleep")
        sphero.power.enter_soft_sleep()

 

if __name__ == "__main__":
    # main()
    pure_sphero()
