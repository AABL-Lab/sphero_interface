#!/usr/bin/env python3
from time import sleep, time
import os
from pysphero.device_api.user_io import Color

import random
from pysphero.core import Sphero
from pysphero.driving import Direction
from pysphero.device_api.sensor import CoreTime, Accelerometer, Quaternion, Attitude, Gyroscope

from sphero_interface.msg import HeadingStamped, ColorRequest
from std_msgs.msg import Float32
import rospy
import traceback

ACTIVE_SENSORS = [CoreTime, Accelerometer, Quaternion, Attitude, Gyroscope]
heartbeat_period = 0
# Sphero set
spheros = {
    # "D9:81:9E:B8:AD:DB": None,
    "f0:35:04:88:07:76": None,
    # "c9:b4:ef:32:eC:28": None,
}

SENSOR_READ = False

'''
Wrapped Sphero wraps the subscribers and publishers for one sphero.
Also publishes the last issued command for state tracking.
Some statefullness around the connection.
'''
class WrappedSphero(Sphero):
    def __init__(self, mac_address: str):
        super().__init__(mac_address)
        self.name = "s"+mac_address[0:2]
        
        self.cmd = HeadingStamped()
        
        self.cmd_sub = rospy.Subscriber(self.name+"/cmd", HeadingStamped, self.cmd_cb, queue_size=1)
        self.color_sub = rospy.Subscriber(self.name+"/color_request", ColorRequest, self.color_cb, queue_size=1)

        self.prev_cmd_pub = rospy.Publisher(self.name+"/prev_cmd", HeadingStamped, queue_size=1)
        self.light_pub = rospy.Publisher(self.name+"/light", Float32, queue_size=1)
        
        self.sensors_setup = False
        self.is_connected = False # TODO: This should be calling a ble_adapter method to check
        self.setup()

    def setup(self):
            t0 = time()
            while (not self.is_connected and time() - t0 < 2.): # Try to connect for a little bit
                try:
                    self.ble_adapter = self._ble_adapter_cls(self.mac_address)
                    self.is_connected = True
                    rospy.loginfo(f"{self.name} connected.")
                except Exception as e:
                    traceback.print_exc()
                    rospy.sleep(0.1)
            
            if (self.is_connected):
                self.power.wake()
                self.user_io.set_led_matrix_text_scrolling(string=self.name, color=Color(red=0xff))
                self.driving.reset_yaw()
            else:
                print(f"WARN: {self.name} could not connect to bluetooth adapter: ", e)
                traceback.print_exc()
                self.is_connected = False
    
    def init_sensor_read(self):
        if not (SENSOR_READ): return
        try:
            # for sensor_type in ACTIVE_SENSORS:
            self.sensor.set_notify(self.sensor_bt_cb, *ACTIVE_SENSORS)
            self.sensors_setup = True  
        except:
            print("WARN: Could not connect to sensors for "+self.name)
            self.sensors_setup = False

    def reconnect(self):
        try:
            self.setup()
            self.init_sensor_read()
        except Exception as e:
            print(f"{self.name} failed to reconnect.")


    def cmd_cb(self, heading_magnitude_cmd):
        '''
        Cap and store the recieved command
        '''
        self.cmd = heading_magnitude_cmd
        self.cmd.t = time()
        self.cmd.v = min(100, max(self.cmd.v, -100))
        self.cmd.theta = min(360, max(self.cmd.theta, 0)) # todo: correct for wrap

    def color_cb(self, color_request):
        rospy.loginfo("Setting color to "+str(color_request))
        self.user_io.set_led_matrix_one_color(color=Color(red=int(hex(color_request.r)), green=int(hex(color_request.g)), blue=int(hex(color_request.b))))

    def sensor_bt_cb(self, data:dict):
        # print(data)
        for param in Accelerometer:
            if param in data:
                print(f"{param} : {data.get(param):1.2f}")
        
        for param in Attitude:
            if param in data:
                print(f"{param} : {data.get(param):1.2f}")
        
        for param in Gyroscope:
            if param in data:
                print(f"{param} : {data.get(param):1.2f}")
        
        for param in Quaternion:
            if param in data:
                print(f"{param} : {data.get(param):1.2f}")

        print("=" * 60)

    def step(self):
        if not self.sensors_setup: self.init_sensor_read()

        # cap inputs
        t, v, theta = self.cmd.t, self.cmd.v, self.cmd.theta
        if (time() - t > 3.0 and v > 0):
            print("forcing 0 velocity due to stale command.")
            self.cmd = HeadingStamped()

        self.light_pub.publish(self.sensor.get_ambient_light_sensor_value())

        self.prev_cmd_pub.publish(self.cmd)
        self.driving.drive_with_heading(int(v), int(theta), Direction.forward)

def main():
    print("Starting sphero interface node")
    rospy.init_node("interface")

    '''
    This node has to maintain connections to all spheros and handle send and receive.
    '''
    # Wake up all the spheros 
    for ma in spheros.keys():
        sphero = WrappedSphero(mac_address=ma)
        spheros[ma] = sphero

    while not rospy.is_shutdown():
        for mac_address, sphero in spheros.items():
            if (heartbeat_period % 10 == 0):
                pass
            try:
                if (sphero.is_connected):
                    sphero.step()
                else:
                    sphero.reconnect()
            except Exception as e:
                traceback.print_exc()
                print(f"{sphero.name} error: {e}")
        rospy.sleep(0.01)


    # Close out the blueooth adapters
    for sphero in spheros.values():
        if sphero.ble_adapter: sphero.ble_adapter.close()

    # client = roslibpy.Ros(host="localhost", port=9090)
    # client.run()
    # talker = roslibpy.Topic(client, '/ambient_light', 'std_msgs/Float32')

    # pub = rospy.Publisher("blah/test", Float32)

    # with Sphero(mac_address=mac_address) as sphero:
    #     sphero.power.wake()
    #     # sphero.user_io.set_all_leds_8_bit_mask(back_color=Color(red=0xff))
    #     # sphero.user_io.set_led_matrix_one_color(color=Color(red=0xff))
    #     sphero.user_io.set_led_matrix_text_scrolling(string="CATCH", color=Color(red=0xff))
    #     for _ in range(10):
    #         speed = random.randint(50, 100)
    #         heading = random.randint(0, 360)
    #         print(f"Send drive with speed {speed} and heading {heading}")
    #         sphero.driving.drive_with_heading(speed, heading, Direction.forward)
            
    #         print("Ambient light sensor value:" + str(sphero.sensor.get_ambient_light_sensor_value()))
    #         # talker.publish(roslibpy.Message({'data':sphero.sensor.get_ambient_light_sensor_value()}))
    #         pub.publish(sphero.sensor.get_ambient_light_sensor_value())

    #     sphero.power.enter_soft_sleep()
 

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass