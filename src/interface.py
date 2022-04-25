#!/usr/bin/env python3
from time import sleep, time
import math
import os
import random
from turtle import update

from pysphero.device_api.user_io import Color, Pixel
from pysphero.core import Sphero
from pysphero.driving import Direction
from pysphero.device_api.sensor import CoreTime, Accelerometer, Quaternion, Attitude, Gyroscope

from sphero_interface.msg import ColorRequest, SpheroNames, HeadingStamped
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion as RosQuaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
import rosnode
import traceback
import random
from IPython import embed

'''
Wrapped Sphero wraps the subscribers and publishers for one sphero.
Also publishes the last issued command for state tracking.
Some statefullness around the connection.
'''
import signal
from contextlib import contextmanager

class TimeoutException(Exception): pass

@contextmanager
def time_limit(seconds):
    def signal_handler(signum, frame):
        raise TimeoutException("Timed out!")
    signal.signal(signal.SIGALRM, signal_handler)
    signal.alarm(seconds)
    try:
        yield
    finally:
        signal.alarm(0)


IN_LAB = False if os.environ.get("IN_LAB") is None else True
print(f"IN_LAB: {IN_LAB}")

from TrackerParams import GREEN_RGB, WHITE_RGB, Sphero_RGB_Color

spheros = {
    "EC:73:F2:19:0E:CA": None,
    # "CA:64:39:FC:74:FB": None,
    "D1:FC:A0:92:D5:19": None,
    # "D9:81:9E:B8:AD:DB": None,
    # "F8:48:B1:E1:1E:2D": None,
    # "E9:84:4B:AD:58:EF": None,
    # "F6:24:6F:6D:B1:2D": None,
    # "DC:6A:87:40:AA:AD": None,
    # "FD:B5:2E:2B:2A:3C": None,
    # "FB:E7:20:44:74:E4": None,
    # "D7:98:82:CD:1F:EA": None,
    "C8:2E:9A:E9:37:16": None,
    # "D1:7E:07:ED:D1:37": None,
    # "CD:7D:FA:67:54:AB": None,
    # "F0:35:04:88:07:76": None,
    # "C9:B4:EF:32:EC:28": None,
} if IN_LAB else {
    "F8:48:B1:E1:1E:2D": None,
    # "D9:81:9E:B8:AD:DB": None,
}

MAX_CONNECTION_TRIES = 3
# ACTIVE_SENSORS = [CoreTime, Quaternion] #Accelerometer, Attitude , Gyroscope]
SENSOR_READ = True

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
        self.cmd_echo_pub = rospy.Publisher(self.name+"/echo_cmd", HeadingStamped, queue_size=1)
        
        self.n_tries = 0

    def setup(self):
        t0 = time()
        self.sensors_setup = False
        self.is_connected = False # TODO: This should be calling a ble_adapter method to check
        while (not self.is_connected) and (self.n_tries < 2):
            try:
                self.ble_adapter = self._ble_adapter_cls(self.mac_address)
                with time_limit(5):
                    self.power.wake()
                    r,g,b = Sphero_RGB_Color[self.name]
                    rospy.sleep(0.25)
                    # self.user_io.set_all_leds_8_bit_mask(front_color=Color(r//10, g//10, b//10), back_color=Color(0,0,0)) #Color(r,g,b))
                    self.user_io.set_all_leds_8_bit_mask(front_color=Color(WHITE_RGB[0]//20, WHITE_RGB[1]//20, WHITE_RGB[2]//20), back_color=Color(0,0,0)) #Color(r,g,b))
                    print(f"{self.name} Setting matrix to: {r},{g},{b}")
                    # self.user_io.set_led_matrix_one_color(Color(*WHITE_RGB))
                    rospy.sleep(0.25)
                    self.user_io.set_led_matrix_one_color(Color(r, g, b))

                rospy.loginfo(f"{self.name} connected.")
                self.is_connected = True
                rospy.sleep(0.25)
                self.init_sensor_read()
                rospy.sleep(0.25)
            except Exception as e:
                if (e is TimeoutException):
                    rospy.loginfo(f"{self.name} connect attempt {self.n_tries} timed out during setup.")
                else:
                    # traceback.print_exc()
                    rospy.loginfo(f"{self.name} connect attempt {self.n_tries} failed to connect.")
                self.n_tries += 1
                self.ble_adapter = None
                # raise e
        return self.is_connected
        
    
    def init_sensor_read(self):
        if not (SENSOR_READ): return # don't care about sensors 
        with time_limit(1):
            try:
                self.sensor.set_notify(self.sensor_bt_cb, CoreTime, Quaternion, interval=100)
                self.sensors_setup = True 
                rospy.loginfo(f"Sensors initialized for {self.name}") 
            except TimeoutException as e:
                self.sensors_setup = False
                rospy.loginfo(f"{self.name} sensor init timed out.")

    def cmd_cb(self, heading_magnitude_cmd):
        '''
        Cap and send the command
        '''
        self.cmd = heading_magnitude_cmd
        self.cmd.t = time()
        self.cmd.v = min(100, max(self.cmd.v, -100))
        self.cmd.theta = min(360, max(self.cmd.theta, 0)) # todo: correct for wrap

        # self.cmd_echo_pub.publish(self.cmd)
        try:
            self.driving.drive_with_heading(int(self.cmd.v), int(self.cmd.theta), Direction.forward)
            rospy.loginfo(f"{self.name} sent command: {self.cmd.v:1.2f} {self.cmd.theta:1.2f}")
        except Exception as e:
            rospy.loginfo(f"Failed drive_with_heading for {self.name}")

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
        imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        r,p,y = euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z])
        # print(f"{self.name} {r:1.2f} {p:1.2f} {y:1.2f}")
        # The default 0 is actually +/- pi (WHY!?). We correct for that here.
        # offset_yaw = y + math.pi
        imu.orientation = orientation #quaternion_from_euler(r, p, offset_yaw)

        # print(f"{self.name} {r:1.2f} {p:1.2f} {y:1.2f}")
        self.ekf_orientation_pub.publish(imu)

    def step(self):
        if not self.is_connected: 
            print(f"Not connected to {self.name}.")
            return

        if not self.sensors_setup: self.init_sensor_read()

        # cap inputs
        t, v, theta = self.cmd.t, self.cmd.v, self.cmd.theta
        if (time() - t > 3.0): 
            self.cmd = HeadingStamped()
            if (v > 0):
                self.cmd.v = 0
                print("forcing 0 velocity due to stale command.")
            if (theta > 0 and "/tracker" not in rosnode.get_node_names()):
                self.cmd.theta = 0
                rospy.loginfo("No tracker so forcing theta to 0")

        try:
            with time_limit(1):
                # self.light_pub.publish(self.sensor.get_ambient_light_sensor_value())
                if (self.cmd.v == 0): # make sure 0s get through
                    self.cmd_echo_pub.publish(HeadingStamped(rospy.get_time(), v, theta))
                    self.driving.drive_with_heading(int(v), int(theta), Direction.forward)
        except TimeoutException as e:
            rospy.loginfo(f"{self.name} timed out on step.")
        # except Exception as e:
        #     rospy.logerr(f"{self.name} exception on step: {e.with_traceback()}")

def main():
    rospy.loginfo("Starting sphero interface node")
    rospy.init_node("interface")
    connected_names_pub = rospy.Publisher("/sphero_names", SpheroNames, queue_size=1, latch=True) # publish a list of the names of connected spheros

    '''
    This node has to maintain connections to all spheros and handle send and receive.
    '''
    # Wake up all the spheros 
    connected_sphero_names = set()
    for ma in spheros.keys():
        rospy.loginfo("Waking up sphero "+ma)
        sphero = WrappedSphero(mac_address=ma.lower())
        if (sphero.setup()):
            spheros[ma] = sphero
            connected_sphero_names.add(sphero.name)
        else:
            rospy.loginfo(f"Gave up on sphero {ma}")

    update_names = True
    while not rospy.is_shutdown():
        rospy.sleep(0.05)
        for mac_address, sphero in spheros.items():
            if not sphero: continue
            try:
                if (sphero.is_connected):
                    sphero.step()
                elif (sphero.n_tries < MAX_CONNECTION_TRIES):
                    sphero.setup()
            except Exception as e:
                traceback.print_exc()
                print(f"{mac_address} error: {e}")

        if (update_names):
            connected_names_pub.publish(SpheroNames(connected_sphero_names))
            update_names = False


    # Close out the blueooth adapters
    for sphero in spheros.values():
        if sphero and sphero._ble_adapter:
            rospy.loginfo(f"Closing out bluetooth adapter for {sphero.name}")
            try:
                with time_limit(1):
                    sphero.power.enter_soft_sleep()
            except TimeoutException as e:
                rospy.loginfo(f"{sphero.name} timed out on enter_soft_sleep.")

            try:
                with time_limit(1):
                    sphero.ble_adapter.close()
            except TimeoutException as e:
                rospy.loginfo(f"{sphero.name} timed out ble_adapter.close")
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass