# from pysphero.bluetooth.ble_adapter import AbstractBleAdapter
#!/usr/bin/env python3
from time import sleep, time
import os
from pysphero.device_api.user_io import Color

import random
from pysphero.core import Sphero
from pysphero.driving import Direction
from pysphero.device_api.sensor import CoreTime, Velocity, Accelerometer, Quaternion

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseStamped # hijack the Pose2D message. x = timestep. theta = heading. y = velocity.
import rospy

ACTIVE_SENSORS = [CoreTime, Quaternion]
heartbeat_period = 0
# Sphero set
spheros = {
    # "d9:81:9e:b8:ad": None,
    # "f0:35:04:88:07:76": None,
    "c9:b4:ef:32:eC:28": None,
}

'''
Wrapped Sphero wraps the subscribers and publishers 
'''
class WrappedSphero(Sphero):
    def __init__(self, mac_address: str):
        super().__init__(mac_address)
        self.name = "s"+mac_address[0:2]
        
        self.pose = Pose2D(); 
        self.vector = Pose2D()
        
        self.cmd_sub = rospy.Subscriber(self.name+"/goal", Pose2D, self.cmd_cb, queue_size=1)

        self.light_pub = rospy.Publisher(self.name+"/light", Float32, queue_size=1)
        self.velocity_pub = rospy.Publisher(self.name+"/velocity", Float32, queue_size=1)
        
        self.sensors_setup = False
        self.adaptor_setup = False
        self.setup()

    def setup(self):
        try:
            self.ble_adapter = self._ble_adapter_cls(self.mac_address)
            self.adaptor_setup = True
        except:
            print("WARN: Could not connect to bluetooth adapter for "+self.name)
    
    def init_sensor_read(self):
        try:
            self.sensor.set_notify(self.sensor_bt_cb, *ACTIVE_SENSORS) #) #
            self.sensors_setup = True
        except:
            print("WARN: Could not connect to sensors for "+self.name)

    def cmd_cb(self, pose2d):
        self.vector = pose2d
        self.vector.x = time()

        self.vector.y = min(100, max(self.vector.y, -100))
        self.vector.theta = min(360, max(self.vector.theta, 0)) # todo: correct for wrap

    def sensor_bt_cb(self, data:dict):
        # info = ", ".join("{:1.2f}".format(data.get(param)) for param in Quaternion)
        # print(f"[{data.get(CoreTime.core_time):1.2f}] Quaternion (x, y, z, w): {info}")
        # print("=" * 60)
        sensed_vector = PoseStamped()
        # Send along data from sphero to ros
        sensed_vector.header.stamp = data.get(CoreTime.core_time)
        # sensed_vector.pose.position.x = data.get(Velocity.x)
        # sensed_vector.pose.position.y = data.get(Velocity.y)
        # sensed_vector.pose.position.z = 0
        sensed_vector.pose.orientation.x = data.get(Quaternion.x)
        sensed_vector.pose.orientation.y = data.get(Quaternion.y)
        sensed_vector.pose.orientation.z = data.get(Quaternion.z)
        sensed_vector.pose.orientation.w = data.get(Quaternion.w)

        print(sensed_vector)
        # self.velocity_pub.publish(sensed_vector)

    def step(self):
        if not self.sensors_setup: self.init_sensor_read()


        # cap inputs
        t, v, theta = self.vector.x, self.vector.y, self.vector.theta
        if (time() - t > 3.0 and v > 0):
            print("forcing 0 velocity due to stale command.")
            self.vector = Pose2D()
        # print(self.name+"(v,theta) : {} {}".format(v, theta))

        self.light_pub.publish(self.sensor.get_ambient_light_sensor_value())

        self.driving.drive_with_heading(int(v), int(theta), Direction.forward)

def main():
    print("Starting sphero interface node")
    rospy.init_node("sphero_interface")

    '''
    This node has to maintain connections to all spheros and receive data from each one.
    '''
    # Wake up all the spheros 
    for ma in spheros.keys():
        sphero = WrappedSphero(mac_address=ma)
        sphero.power.wake()
        sphero.user_io.set_led_matrix_text_scrolling(string=sphero.name, color=Color(red=0xff))
        spheros[ma] = sphero

    while not rospy.is_shutdown():
        for mac_address, sphero in spheros.items():
            if (heartbeat_period % 10 == 0):
                pass
            try:
                sphero.step()
            except pysphero.exceptions.PySpheroTimeoutError:
                print("Timeout "+sphero.name)
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