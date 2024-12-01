#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
import math

def pose_callback(msg):
    # Extract quaternion from PoseStamped message
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )

    # Convert quaternion to roll, pitch, yaw in radians
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    # Prepare the arrays for publishing in radians and degrees
    rpy_radians = Float32MultiArray()
    rpy_radians.data = [roll, pitch, yaw]

    rpy_degrees = Float32MultiArray()
    rpy_degrees.data = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

    # Publish the roll, pitch, yaw in radians and degrees
    rpy_radians_publisher.publish(rpy_radians)
    rpy_degrees_publisher.publish(rpy_degrees)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('quaternion_to_rpy')

    # Subscribe to the /mavros/local_position/pose topic
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)

    # Create publishers for the roll, pitch, yaw in radians and degrees
    rpy_radians_publisher = rospy.Publisher('/drone/rpy/radians', Float32MultiArray, queue_size=10)
    rpy_degrees_publisher = rospy.Publisher('/drone/rpy/degrees', Float32MultiArray, queue_size=10)

    # Keep the node running
    rospy.spin()
