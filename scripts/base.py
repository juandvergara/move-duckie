#!/usr/bin/env python3

import numpy as np
import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class Duckiebot(DTROS):

    def __init__(self, node_name):
        super(Duckiebot, self).__init__(node_name=node_name,
node_type=NodeType.GENERIC)
        self.node_name = rospy.get_name()
        self.omega = 0
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        rospy.logwarn("Creating the duckiebot")

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)

        # Publications
        self.car_cmd = rospy.Publisher("/duckiebot3/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

        # Subscriptions

        # Move duckie
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    def printVel(self):
        self.car_cmd_msg= Twist2DStamped()
        self.car_cmd_msg.v = 0.5
        self.car_cmd_msg.omega = 0.0
        while(rospy.Time.now() < rospy.Time.now() + rospy.Time(10)):
            self.car_cmd.publish(self.car_cmd_msg)
        rospy.loginfo(self.car_cmd_msg)

if __name__ == "__main__":
    duckie = Duckiebot(node_name="duckie_move")
    duckie.printVel()
    rospy.spin()
