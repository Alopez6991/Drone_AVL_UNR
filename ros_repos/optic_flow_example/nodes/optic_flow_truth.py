#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy

# opencv imports
import cv2

# numpy imports - basic math and matrix manipulation
import numpy as np

# imports for ROS image handling
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# message imports specific to this package
from optic_flow_example.msg import OpticFlowMsg
from optic_flow_example.msg import MeanOpticFlowMsg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
################################################################################

class Optic_Flow_Truth:
    def __init__(self):
        self.pub_truth_x=rospy.Publisher('optic_flow_truth_x', Float32)
        self.pub_truth_y=rospy.Publisher('optic_flow_truth_y', Float32)
        self.sub_z = rospy.Subscriber('/et/ned/pose_stamped', TwistWithCovarianceStamped, self.z_callback)
        self.sub_xy_vel = rospy.Subscriber('velocity/body_level_frame', TwistWithCovarianceStamped, self.xy_vel_callback)
        self.fx=615.03
        self.fy=615.21
        self.z=-.1
    def z_callback(self, msg):
        self.z = msg.pose.position.z
    def xy_vel_callback(self, msg):
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.pub_truth_x.publish(self.fx*self.vx/self.z)
        self.pub_truth_y.publish(self.fy*self.vy/self.z)
################################################################################
# Main function
    def main(self):
            try:
                rospy.spin()
            except KeyboardInterrupt:
                print ("Shutting down")
                cv2.destroyAllWindows()

################################################################################
if __name__ == '__main__':
    parser = OptionParser()
    (options, args) = parser.parse_args()
    rospy.init_node('optic_flow_truth', anonymous=True)
    of_truth = Optic_Flow_Truth()
    of_truth.main()