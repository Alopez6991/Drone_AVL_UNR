#!/usr/bin/env python

import serial
import numpy as np
import rospy
import math
from optparse import OptionParser
from trisonica_ros.msg import trisonica_msg
from geometry_msgs.msg import PoseStamped

import copy

class Trisonica(object):
    def __init__(self, port="/dev/ttyUSB1", topic='/trisonica', baud=115200, rate=400):
        baud = baud
        self.rate = rate # rate to check/publish new data, Hz

        self.nodename = rospy.get_name().rstrip('/')
        self.params = rospy.get_param(self.nodename, {})
        self.Vx=0
        self.Vy=0
        self.Vz=0
        self.R=0
        self.P=0
        self.Y=0
        self.C=np.matrix([[0 , 0, 0]
                        ,[0, 0 , 0]
                        ,[0 , 0 , 0]])
        # Set serial port name
        if rospy.has_param(self.nodename + '/port'):
            self.port = self.params['port']
        else:
            self.port = port

        # Set topic name
        if rospy.has_param(self.nodename + '/topic'):
            self.topic = self.nodename + '/' + self.params['topic']
        else:
            self.topic = topic

        # initilize msg
        self.msg = trisonica_msg()
        self.msg_body_level = trisonica_msg()
        self.msg_global = trisonica_msg()

        rospy.logwarn('Trisonica topic: ' + self.topic)

        # Connect to serial port
        rospy.logwarn('Connecting to: ' + self.port)
        self.connection = serial.Serial(self.port, baud, timeout=0.01)
        self.connection.flush()
        rospy.logwarn('Connected.')

        # Define publisher
        self.publisher = rospy.Publisher(self.topic, trisonica_msg, queue_size=10)
        self.publisher_body_level = rospy.Publisher(self.topic+"_body_level", trisonica_msg, queue_size=10)
        self.publisher_global = rospy.Publisher(self.topic+"_global", trisonica_msg, queue_size=10)

        # Define subscriber
        self._mocap_sub = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.mocap_cb)

    def mocap_cb(self,mocap_state):
        self.R,self.P,self.Y=self.euler_from_quaternion(mocap_state.pose.orientation.x,mocap_state.pose.orientation.y,mocap_state.pose.orientation.z,mocap_state.pose.orientation.w)
        self.Rd=np.rad2deg(self.R)
        self.Pd=np.rad2deg(self.P)
        self.Yd=np.rad2deg(self.Y)
        self.C=self.unroll_unpitch(self.R,self.P)
        # print(self.C)
        # print(self.Rd,self.Pd,self.Yd)


    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def unroll_unpitch(self,R,P):
        C=np.matrix([[np.cos(P) , np.sin(R)*np.sin(P), np.cos(R)*np.sin(P)]
                        ,[0 , np.cos(R) , -1*np.sin(R)]
                        ,[-1*np.sin(P) , np.sin(R)*np.cos(P) , np.cos(R)*np.sin(P)]])
        return C

    def get_Cyaw(self,Y):
        Cyaw=np.matrix([[np.cos(Y) , 1*np.sin(Y), 0]
                        ,[-1*np.sin(Y) , np.cos(Y) , 0]
                        ,[0 , 0 , 1]])
        return Cyaw

    def get_Cpitch(self,P):
        Cpitch=np.matrix([[np.cos(P) , 0, -1*np.sin(P)]
                        ,[0, 1 , 0]
                        ,[np.sin(P) , 0 , np.cos(P)]])

        return Cpitch

    def get_Croll(self,R):
        Croll=np.matrix([[1 , 0, 0]
                        ,[0, np.cos(R) , np.sin(R)]
                        ,[0 , -1*np.sin(R) , np.cos(R)]])

        return Croll

    def main(self):
        msg = self.msg
        msg_body_level = self.msg_body_level
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            data = self.connection.readline()
            #rospy.logwarn(data)
            data = data.decode()
            if data is not None and len(data) > 10:
                if 1: #data[0] == 'S':
                    msg.header.stamp.secs = rospy.Time.now().secs
                    msg.header.stamp.nsecs = rospy.Time.now().nsecs
                    try:
                        msg.speed       = float( data.split('S ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.speed = np.nan
                        pass

                    try:
                        msg.speed2d       = float( data.split('S2 ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.speed2d = np.nan
                        pass

                    try:
                        msg.direction   = float( data.split(' D ')[1].lstrip().split(' ')[0])
                    except:
                        #msg.direction = np.nan
                        pass

                    try:
                        msg.vertdirection   = float( data.split('DV ')[1].lstrip().split(' ')[0])
                    except:
                        #msg.vertdirection = np.nan
                        pass

                    try:
                        msg.northsouth  = float( data.split('U ')[1].lstrip().split(' ')[0] )
                        self.Vx= msg.northsouth
                        # print(self.Vx)
                    except:
                        #msg.northsouth = np.nan
                        pass

                    try:
                        msg.westeast    = float( data.split(' V ')[1].lstrip().split(' ')[0] )
                        self.Vy = msg.westeast
                    except:
                        #msg.westeast = np.nan
                        pass

                    try:
                        msg.updown      = float( data.split('W ')[1].lstrip().split(' ')[0] )
                        self.Vz = msg.updown
                    except:
                        #msg.updown = np.nan
                        pass

                    try:
                        msg.temperature = float( data.split('T ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.temperature = np.nan
                        pass

                    try:
                        msg.pressure = float( data.split('P ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.pressure = np.nan
                        pass

                    try:
                        msg.humidity = float( data.split('H ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.humidity = np.nan
                        pass

                    try:
                        msg.pitch = float( data.split('P ')[2].lstrip().split(' ')[0] )
                    except:
                        try:
                            msg.pitch = float( data.split('PI ')[1].lstrip().split(' ')[0] )
                        except:
                            #msg.pitch = np.nan
                            pass

                    try:
                        msg.roll = float( data.split('RO ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.roll = np.nan
                        pass

                    try:
                        msg.heading = float( data.split('MD ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.heading = np.nan
                        pass

                    try:
                        msg.levelx = float( data.split('AX ')[1].lstrip().split(' ')[0] )
                    except:
                        msg.levelx = np.nan
                        pass

                    try:
                        msg.levely = float( data.split('AY ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levely = np.nan
                        pass

                    try:
                        msg.levelz = float( data.split('AZ ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levelz = np.nan
                        pass

                    msg_body_level=copy.deepcopy(msg)
                    msg_global=copy.deepcopy(msg)
                    # print(np.matrix([[Vx],[Vy],[Vz]]))
                    # print(self.C@np.matrix([[Vx],[Vy],[Vz]]))
                    Vsensor_SF=np.matrix([[self.Vx],[self.Vy],[self.Vz]])
                    C_SF_2_BF=self.get_Croll(np.pi)
                    Vsensor_BF=C_SF_2_BF@Vsensor_SF
                    # print(self.C)
                    Vsensor_BL=self.C@Vsensor_BF
                    C_BL_2_G=self.get_Cyaw(self.Y)
                    Vsensor_G=C_BL_2_G@Vsensor_BL
                    # print(float(V[0]),float(V[1]),float(V[2]))
                    msg_body_level.westeast=float(Vsensor_BL[0])
                    msg_body_level.northsouth=float(Vsensor_BL[1])
                    msg_body_level.updown=float(Vsensor_BL[2])

                    msg_global.westeast=float(Vsensor_G[0])
                    msg_global.northsouth=float(Vsensor_G[1])
                    msg_global.updown=float(Vsensor_G[2])
                    # print(msg)
                    self.publisher_body_level.publish(msg_body_level)
                    self.publisher_global.publish(msg_global)
                    self.publisher.publish(msg)

            rate.sleep()

        self.connection.close()

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--port", type="str", dest="port", default='/dev/ttyUSB1',
                        help="port to which trisonica is connected")
    parser.add_option("--topic", type="str", dest="topic", default='/trisonica',
                        help="rostopic to publish to")
    parser.add_option("--rate", type="int", dest="rate", default=80,
                        help="ROS rate to check for data and publish")
    parser.add_option("--baud", type="int", dest="baud", default=115200,
                        help="baudrate")
    parser.add_option("--nodename", type="str", dest="nodename", default='trisonica',
                        help="name of the node")

    (options, args) = parser.parse_args()

    rospy.init_node(options.nodename, anonymous=True)

    trisonica = Trisonica(port=options.port, topic=options.topic, baud=options.baud, rate=options.rate)
    trisonica.main()