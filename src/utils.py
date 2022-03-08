
import math

from geometry_msgs import Pose2D
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