#!/usr/bin/env python3

import os
import rospy
import numpy as np
import math
from typing import Tuple
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelEncoderStamped

# ~~~~~~~~~~~~~ Geometric Definitions ~~~~~~~~~~~~~~~~~~~~~~
TICKS_PER_REVOLUTION = 135
WHEEL_RADIUS = rospy.get_param(f"/{os.environ['VEHICLE_NAME']}/kinematics_node/radius", 100)
WHEEL_CIRCUMFERENCE =  2 * np.pi * WHEEL_RADIUS
ROBOT_L = 0.1085 #meters
sigma = 0.00001 #close enough to zero check

# ~~~~~~~~~~~~~ Kinematic Helper Functions ~~~~~~~~~~~~~~~~~
ratio_sigma = 0.01

def calculate_radius(length: float, 
                     vel_left: float, 
                     vel_right: float) -> float:
    """
    Calculates the radius of the turn

    Arguments:
        length: the distance between the wheels
        vel_left: the velocity of the left wheel
        vel_right: the velocity of the right wheel

    Returns:
        The radius of the turn
    """
    ratio = vel_left / vel_right

    #as the robot gets closer to moving straight the radius approaches 
    #infinity and rest of the math has a divide by 0 error. 
    if np.abs(ratio - 1) < ratio_sigma:
        return math.inf

    R = (0.5 * length * (1 + ratio)) / (1 - ratio)
    return R

def calculate_angular_veloctiy(length: float, 
                               radius: float, 
                               vel_right: float) -> float:
    """
    Calculate the angular velocity of a turn

    Arguments:
        length: the distance between the wheels
        raidus: the radius of the turn
        vel_right: the velocity of the right wheel

    Returns:
        The angular velocity of the turn in radians/sec
    """
    return vel_right / (radius + 0.5 * length)

def calculate_ICC(radius: float, 
                  heading: float, 
                  x: float, 
                  y: float) -> Tuple[float, float]:
    """
    Calculates the Instantaneous Center of Curvature 

    Arguments:
        radius: the radius of the turn
        heading: the heading of the robot in radians
        x: the x coordinate of the robot
        y: the y coordinate of the robot

    Returns:
        A tuple in the form of (ICCx, ICCy)
    """
    ICCx = x - radius * np.sin(heading)
    ICCy = y + radius * np.cos(heading)
    return ICCx, ICCy

# ~~~~~~~~~~~~~~~~~~ Helpers ~~~~~~~~~~~~~~~~~~~~
def are_messages_different(message_one, message_two):
    """
    Checks if the timestamp in the message headers are the same
    """

    return message_one.header.stamp != message_two.header.stamp

def EWMA(previous, current, a):
    return (1-a)*previous + a * current

# ~~~~~~~~~~~~~~~~~~ ROS Node ~~~~~~~~~~~~~~~~~~~

class OdometryEncoder(DTROS):
    def __init__(self, node_name) -> None:
        super(OdometryEncoder, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.heading = 0
        self.x = 0
        self.y = 0

        self.prev_left_enc_msg = None
        self.prev_right_enc_msg = None
        self.left_enc_msg = None
        self.right_enc_msg = None

        self.prev_left_tick = 0
        self.prev_right_tick = 0
        
        #sub to left encoder
        self.left_tick_sub = rospy.Subscriber(
            f"/{os.environ['VEHICLE_NAME']}/left_wheel_encoder_node/tick", 
            WheelEncoderStamped,
            self.left_encoder_cb,
        )

        #sub to right encoder
        self.right_tick_sub = rospy.Subscriber(
            f"/{os.environ['VEHICLE_NAME']}/right_wheel_encoder_node/tick", 
            WheelEncoderStamped,
            self.right_encoder_cb,
        )

        #TODO: Publish on a topic


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

        #filter ticks
        dtick_left = EWMA(self.prev_left_tick, dtick_left, 0.2)
        dtick_right = EWMA(self.prev_right_tick, dtick_right, 0.2)
        self.prev_left_tick = dtick_left
        self.prev_right_tick = dtick_right
        
        #calculate dt.
        #   dt is hard because we are working with two disjoint message streams.
        #   They are most likely not going to ever line up perfectly, so if
        #   we calculate dt using just the left or right messages one of them
        #   will be shorter. What we really want is the longer of the two times,
        #   since that will be total time elapsed between updates.

        #   time comes back in nano seconds
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

        #update position
        dr = right_mps * dt
        dl = left_mps * dt

        #not moving check
        if np.abs(dr) < sigma or np.abs(dl) < sigma:
            return

        radius = calculate_radius(ROBOT_L, left_mps, right_mps)  

        #moving in an arc          
        if radius != math.inf:
            dtheta = (dr - dl) / (ROBOT_L)
            ds = (dr + dl) / 2
            self.x += ds * np.cos(self.heading + dtheta / 2)
            self.y += ds * np.sin(self.heading + dtheta / 2)
            self.heading += dtheta
        else: #moving forward
            # they should almost be the same, but avg for further accuracy
            ds = (dl + dr) / 2
            self.x += dl * np.cos(self.heading)
            self.y += dl * np.sin(self.heading)
        
        self.log(
            f"x: {self.x} y: {self.y} heading: {np.rad2deg(self.heading)} \n\n")

        #set previous 
        self.prev_left_enc_msg = self.left_enc_msg
        self.prev_right_enc_msg = self.right_enc_msg
    
if __name__ ==  "__main__":
    node = OdometryEncoder(node_name="odom_encoder")
    rospy.spin()