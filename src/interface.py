#!/usr/bin/env python3
from time import sleep, time
import math
import os
import random
from turtle import update

from pysphero.device_api.user_io import Color, Pixel
from pysphero.core import Sphero
from pysphero.driving import Direction, StabilizationIndex
from pysphero.device_api.sensor import CoreTime, Accelerometer, Quaternion, Attitude, Gyroscope

from sphero_interface.msg import HeadingStamped, ColorRequest, SpheroNames
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion as RosQuaternion
from tf.transformations import euler_from_quaternion

import rospy
import traceback
import random
from IPython import embed


IN_LAB = False if os.environ.get("IN_LAB") is None else True
print(f"IN_LAB: {IN_LAB}")

spheros = {
    # "D9:81:9E:B8:AD:DB": None,
    # "F8:48:B1:E1:1E:2D": None,
    # "E9:84:4B:AD:58:EF": None,
    # "F6:24:6F:6D:B1:2D": None,
    # "DC:6A:87:40:AA:AD": None,
    "EC:73:F2:19:0E:CA": None,
    "CA:64:39:FC:74:FB": None,
    # "FD:B5:2E:2B:2A:3C": None,
    # "FB:E7:20:44:74:E4": None,
    # "D7:98:82:CD:1F:EA": None,
    # "D1:FC:A0:92:D5:19": None,
    # "C8:2E:9A:E9:37:16": None,
    # "D1:7E:07:ED:D1:37": None,
    # "CD:7D:FA:67:54:AB": None,
    # "F0:35:04:88:07:76": None,
    # "C9:B4:EF:32:EC:28": None,
} if IN_LAB else {
    "D9:81:9E:B8:AD:DB": None,
    "F8:48:B1:E1:1E:2D": None,
}

ACTIVE_SENSORS = [CoreTime, Quaternion] #Accelerometer, Attitude , Gyroscope]

SENSOR_READ = True
STABILIZE_SPHEROS = False

from TrackerParams import GREEN_RGB, Sphero_RGB_Color
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
        self.yaw_reset_sub = rospy.Subscriber(self.name+"/reset_north", Bool, self.reset_north, queue_size=1)

        self.light_pub = rospy.Publisher(self.name+"/light", Float32, queue_size=1)
        self.ekf_orientation_pub = rospy.Publisher(self.name+"/imu_data", Imu, queue_size=1) # publish for the ekf node
        
        self.setup()

    def setup(self):
        t0 = time()
        self.sensors_setup = False
        self.is_connected = False # TODO: This should be calling a ble_adapter method to check
        while (not self.is_connected and time() - t0 < 1.): # Try to connect for a little bit (one second)
            try:
                self.ble_adapter = self._ble_adapter_cls(self.mac_address)
                self.is_connected = True
                rospy.loginfo(f"{self.name} connected.")
            except Exception as e:
                # traceback.print_exc()
                rospy.loginfo(f"{self.name} failed to connect.")
                rospy.sleep(0.1)
        
        if (self.is_connected):
            self.power.wake()
            if not STABILIZE_SPHEROS: 
                # This is silently broken, not sure why TODO
                rospy.loginfo("Disabling sphero control system.")
                self.driving.set_stabilization(StabilizationIndex.no_control_system)

            # self.user_io.set_led_matrix_text_scrolling(string=self.name, color=Color(red=0xff))
            r,g,b = Sphero_RGB_Color[self.name]
            # for x in range(8):
            #     for y in range(3):
            #         self.user_io.set_led_matrix_pixel(Pixel(x,y), Color(red=r, green=g, blue=b))
            # self.user_io.set_led_matrix_one_color(color=Color(red=r, green=g, blue=b))
            rospy.sleep(0.05)
            # self.user_io.set_all_leds_8_bit_mask(front_color=Color(green=50), back_color=Color())
            self.user_io.set_all_leds_8_bit_mask(front_color=Color(green=100), back_color=Color(r,g,b))
            # self.driving.reset_yaw()
        else:
            print(f"WARN: {self.name} could not connect to bluetooth adapter.")
            self.is_connected = False
    
    def init_sensor_read(self):
        if not (SENSOR_READ): return # don't care about sensors 
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
            return True
        except Exception as e:
            print(f"{self.name} failed to reconnect.")
            return False

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

    def reset_north(self, msg):
        self.sensor.magnetometer_calibrate_to_north()

    def sensor_bt_cb(self, data:dict):
        #NOTE: Example of what the data structures look like. For more info see the pysphero package.
        # for param in Gyroscope:
        #     if param in data:
        #         print(f"{param} : {data.get(param):1.2f}")
        
        # Convert quaternion to IMU
        orientation = RosQuaternion()
        orientation.x = data.get(Quaternion.x)
        orientation.y = data.get(Quaternion.y)
        orientation.z = data.get(Quaternion.z)
        orientation.w = data.get(Quaternion.w)
        
        imu = Imu()
        imu.header.stamp = rospy.Time.now() # NOTE: This maybe should be CoreTime
        imu.header.frame_id = "base_footprint" # NOTE: this measurement is in the sphero's frame. Need to make it relative to starting orientation to use here
        imu.orientation = orientation
        imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.ekf_orientation_pub.publish(imu)
        r,p,y = [math.degrees(entry) for entry in euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])]
        print(f"{self.name} {r:1.1f} {p:1.1f} {y:1.1f}")

    def step(self):
        if not self.sensors_setup: self.init_sensor_read()
        if not self.is_connected: 
            print(f"Not connected to {self.name}.")
            return

        # cap inputs
        t, v, theta = self.cmd.t, self.cmd.v, self.cmd.theta
        if (time() - t > 3.0 and v > 0):
            print("forcing 0 velocity due to stale command.")
            self.cmd = HeadingStamped()

        self.light_pub.publish(self.sensor.get_ambient_light_sensor_value())
        self.driving.drive_with_heading(int(v), int(theta), Direction.forward)

def main():
    print("Starting sphero interface node")
    rospy.init_node("interface")
    connected_names_pub = rospy.Publisher("/sphero_names", SpheroNames, queue_size=1, latch=True) # publish a list of the names of connected spheros

    '''
    This node has to maintain connections to all spheros and handle send and receive.
    '''
    # Wake up all the spheros 
    connected_sphero_names = set()
    for ma in spheros.keys():
        rospy.loginfo("Waking up sphero "+ma)
        try:
            sphero = WrappedSphero(mac_address=ma.lower())
            spheros[ma] = sphero
            connected_sphero_names.add(sphero.name)
        except Exception as e:
            print(f"WARN: Could not connect to sphero {ma}: {e}")
            traceback.print_exc()


    update_names = True
    while not rospy.is_shutdown():
        for mac_address, sphero in spheros.items():
            if not sphero: continue
            try:
                if (sphero.is_connected):
                    sphero.step()
                else:
                    print(f"Attempting to reconnect from disconnected sphero {sphero.name}...")
                    if not sphero.reconnect():
                        print(f"Could not reconnect to {sphero.name}...")
                        connected_sphero_names.remove(sphero.name)
                        update_names = True
                    
            except Exception as e:
                traceback.print_exc()
                print(f"{mac_address} error: {e}")
        
        if (update_names):
            connected_names_pub.publish(SpheroNames(connected_sphero_names))
            update_names = False

        # print(f"looping...")
        rospy.sleep(0.05)

    # Close out the blueooth adapters
    for sphero in spheros.values():
        if sphero and sphero.ble_adapter: sphero.ble_adapter.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass