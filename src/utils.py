
import math
import cv2
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

def posewithcovariancestamped_to_pose2d(msg):
    r,p,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    return Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

def pose2d_distance(a, b):
    return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 )

def speed_to_mps(speed):
    '''
    Convert a 0-255 sphero speed command to meters-per-second
    '''
    return (speed/255) * 2.01168 # max speed (4.5 miles per hour)

def cap_0_to_2pi(angle):
    while (angle > 2*math.pi): angle -= 2*math.pi
    while (angle < 0): angle += 2*math.pi
    return angle

def cap_0_to_360(angle):
    while (angle > 360): angle -= 360
    while (angle < 0): angle += 360
    return angle

def list_vide_ports():
    """
    Test the ports and returns a tuple with the available ports and the ones that are working.
    """
    is_working = True
    dev_port = 0
    working_ports = []
    available_ports = []
    for dev_port in range(4):
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            is_working = False
            print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
    return available_ports,working_ports

# def init_videocapture(channel=0,width=1280, height=720):
def init_videocapture(channel=0,width=2048, height=1080, scale=2):
# def init_videocapture(channel=0,width=4096, height=2160, scale=2):
    camera = cv2.VideoCapture(channel, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width//2)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height//2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    camera.set(cv2.CAP_PROP_FPS, 30)
    # camera.set(28, 100)
    return camera
