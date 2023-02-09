#!/usr/bin/env python3

import os
import rospy
import numpy as np
import math
from typing import Tuple
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from duckietown_msgs.msg import WheelEncoderStamped
from as_msgs.msg import WheelOdometry

# ~~~~~~~~~~~~~ Geometric Definitions ~~~~~~~~~~~~~~~~~~~~~~
TICKS_PER_REVOLUTION = 135
WHEEL_RADIUS = rospy.get_param(f"/{os.environ['VEHICLE_NAME']}/kinematics_node/radius", 100)
WHEEL_CIRCUMFERENCE =  2 * np.pi * WHEEL_RADIUS
#ROBOT_L = 0.1085 #meters

# ~~~~~~~~~~~~~~~~~~ Helpers ~~~~~~~~~~~~~~~~~~~~
def are_messages_different(message_one, message_two):
    """
    Checks if the timestamp in the message headers are the same
    """

    return message_one.header.stamp != message_two.header.stamp

# ~~~~~~~~~~~~~~~~~~ ROS Node ~~~~~~~~~~~~~~~~~~~

class WheelOdometryNode(DTROS):
    def __init__(self, node_name) -> None:
        super(WheelOdometryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.heading = 0
        self.x = 0
        self.y = 0

        self.prev_left_enc_msg = None
        self.prev_right_enc_msg = None
        self.left_enc_msg = None
        self.right_enc_msg = None

        #sub to left encoder
        self.left_tick_sub = rospy.Subscriber(
            f"~left_wheel_encoder_node/tick", 
            WheelEncoderStamped,
            self.left_encoder_cb,
        )

        #sub to right encoder
        self.right_tick_sub = rospy.Subscriber(
            f"~right_wheel_encoder_node/tick", 
            WheelEncoderStamped,
            self.right_encoder_cb,
        )

        self.odom_pub = rospy.Publisher(
            f"~odometry",
            WheelOdometry,
            queue_size=1
        )

        self.log("Initalized!")

    def left_encoder_cb(self, data):
        self.left_enc_msg = data
        if self.prev_left_enc_msg is None:
            self.prev_left_enc_msg = self.left_enc_msg
        self.update()

    def right_encoder_cb(self, data):
        self.right_enc_msg = data
        if self.prev_right_enc_msg is None:
            self.prev_right_enc_msg = self.right_enc_msg
        self.update()

    def update(self):
        
        #null checks
        if self.left_enc_msg is None or self.right_enc_msg is None:
            return

        #do not update if both encoder messages haven't been received yet
        if not are_messages_different(self.left_enc_msg, self.prev_left_enc_msg):
            return
        
        if not are_messages_different(self.right_enc_msg, self.prev_right_enc_msg):
            return

        #calculate encoder differences
        dtick_left = self.left_enc_msg.data - self.prev_left_enc_msg.data
        dtick_right = self.right_enc_msg.data - self.prev_right_enc_msg.data
        
        #calculate dt.
        dt_left = self.left_enc_msg.header.stamp - \
                  self.prev_left_enc_msg.header.stamp
    
        dt_right = self.right_enc_msg.header.stamp - \
                   self.prev_right_enc_msg.header.stamp 

        dt = max(dt_left, dt_right)

        #dt /= 1e9 #dt is in nano seconds, convert it to seconds
        dt = dt.to_nsec() / 1000000000.0

        #calculate speeds
            # 1) ticks per second -> 
            # 2) revolutions per second ->
            # 3) meters per second
        left_tps = dtick_left / dt
        right_tps = dtick_right / dt

        left_rps = left_tps / TICKS_PER_REVOLUTION
        right_rps = right_tps / TICKS_PER_REVOLUTION

        left_mps = left_rps * WHEEL_CIRCUMFERENCE
        right_mps = right_rps * WHEEL_CIRCUMFERENCE

        #set previous 
        self.prev_left_enc_msg = self.left_enc_msg
        self.prev_right_enc_msg = self.right_enc_msg

        #publish to topic
        odometry_msg = WheelOdometry()
        header = Header()
        header.stamp = rospy.Time.now()
        odometry_msg.header = header
        odometry_msg.left_wheel_velocity = left_mps
        odometry_msg.right_wheel_velocity = right_mps
        self.odom_pub.publish(odometry_msg)
    
if __name__ ==  "__main__":
    node = WheelOdometryNode(node_name="wheel_odometry")
    rospy.spin()