#!/usr/bin/env python3

import numpy as np
import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

# from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class Duckiebot(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.logwarn("Creating the duckiebot")

    # Setup Parameters
        self.bridge_object = CvBridge()
        self.car_cmd_msg = Twist2DStamped()
        self.v_gain = self.setupParam("/duckiebot3/joy_mapper_node/speed_gain", 0.41)
        self.omega_gain = self.setupParam("/duckiebot3/joy_mapper_node/steer_gain", 8.3)

    # Publications
        self.pub_car_cmd = rospy.Publisher("/duckiebot3/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)

    # Subscriptions
        self.image_sub = rospy.Subscriber("/duckiebot3/camera_node/image/compressed", CompressedImage, self.cam_duckie)

    # Setup param
    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name, value))

    def printVel(self, vel, omega):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = vel
        car_cmd_msg.omega = omega
        while(car_cmd_msg.header.seq == 0):
            self.pub_car_cmd.publish(car_cmd_msg)
        rospy.loginfo(car_cmd_msg)

    def cam_duckie(self, image):
        # rospy.logwarn("printing image")
        np_arr = np.frombuffer(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        height, width, channels = image_np.shape
        image_circle = cv2.circle(image_np, (int(height/2), int(width/2)), 10, (0, 0, 255), -1)
        # Create a compressed msg
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        # cv2.imshow('Output',image_np)

if(__name__ == "__main__"):
    rospy.init_node("duckie_move", anonymous=False)
    duckiebot = Duckiebot()
    duckiebot.printVel(0.0, 0.0)

    def shutdownhook():
      duckiebot.printVel(0.0, 0.0)
      rospy.loginfo("Shutdown time!")

    rospy.on_shutdown(shutdownhook)
    rospy.spin()
