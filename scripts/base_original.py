#!/usr/bin/env python3

import numpy as np
import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class Duckiebot(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.omega = 0
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        rospy.logwarn("Creating the duckiebot")

        # Setup Parameters
        self.bridge_object = CvBridge()
        self.v_gain = self.setupParam("/duckiebot3/joy_mapper_node/speed_gain", 0.41)
        self.omega_gain = self.setupParam("/duckiebot3/joy_mapper_node/steer_gain", 8.3)

        # Publications
        self.car_cmd = rospy.Publisher("/duckiebot3/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.image_sub = rospy.Subscriber("/duckiebot3/camera_node/image/compressed", CompressedImage, self.cam_duckie)

        # Move duckie
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = 0.3
        car_cmd_msg.omega = 0
        self.car_cmd.publish(car_cmd_msg)
        rospy.loginfo(car_cmd_msg)

    def cam_duckie(self, image):
        rospy.logwarn("printing image")
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            #print(e)

if (__name__ == "__main__"):
    rospy.init_node("duckie_move", anonymous=False)
    duckiebot=Duckiebot()
    rospy.spin()
