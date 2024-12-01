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

################################################################################
# Ros subscribers for topic optic_flow
class Mean_Optic_Flow_Calculator:
    def __init__(self, topic):
        self.OF_pub=rospy.Publisher('mean_optic_flow', MeanOpticFlowMsg)

        self.OF_sub = rospy.Subscriber(topic, OpticFlowMsg, self.OF_callback)
        self.fx=615.03
        self.fy=615.21

    def OF_callback(self, msg):
        # Calculate the mean optic flow
        
        mean_optic_flow = MeanOpticFlowMsg()
        mean_optic_flow.header = msg.header
        mean_optic_flow.dt = msg.dt
        mean_optic_flow.OFx = np.mean(msg.vx)/msg.dt
        mean_optic_flow.OFy = np.mean(msg.vy)/msg.dt
        self.OF_pub.publish(mean_optic_flow)

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
    parser.add_option("-t", "--topic", dest="topic", default='/optic_flow',
                      help="input topic for optic flow data", metavar="topic")
    (options, args) = parser.parse_args()

    rospy.init_node('optic_flow_calculator', anonymous=True)
    MOFC = Mean_Optic_Flow_Calculator(options.topic)
    MOFC.main()
