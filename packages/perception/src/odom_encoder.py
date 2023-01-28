#!/usr/bin/env python3

import os
import rospy
import numpy as np
import math
from typing import Tuple
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelEncoderStamped


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

# ~~~~~~~~~~~~~~~~~~ ROS Node ~~~~~~~~~~~~~~~~~~~

class OdometryEncoder(DTROS):
    def __init__(self, node_name) -> None:
        super(OdometryEncoder, self).__init__(node_name=node_name, NodeType=NodeType.PUBLISHER)
        
        self.heading = 0
        self.x = 0
        self.y = 0

        self.prev_left_enc_msg = None
        self.prev_right_enc_msg = None
        self.left_enc_msg = None
        self.right_enc_msg = None
        #sub to left encoder
        #sub to right encoder

        #TODO: Publish on a topic

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
        

        pass

    def calculate_new_position(self, vel_left, vel_right, dt):
        """
        Updates the estimated positions

        Arguments:
            vel_left: the velocity of the left wheel in meters/sec
            vel_right: the velocity of the right wheel in meters/sec
            dt: the time interval
        """
        length = 0 #TODO Get this somehow, load in from file that duckietown provides
        radius = calculate_radius(length, vel_left, vel_right)

        #straight line case
        if raidus == math.inf:
            magnitude = vel_left * dt
            dx = magnitude * np.cos(self.heading)
            dy = magnitude * np.sin(self.heading)
            self.x += dx
            self.y += dy
        else:
            w = calculate_angular_veloctiy(length, radius, vel_right)
            ICCx, ICCy = calculate_ICC(radius, heading, self.x, self.y)

            rotation = np.array([[np.cos(wdt), -np.sin(wdt), 0],
                            [np.sin(wdt), np.cos(wdt), 0],
                            [0, 0, 1]])

            frame = np.array([[self.x - ICCx],
                              [self.y - ICCy],
                              [self.heading]])

            translation = np.array([[ICCx],
                                    [ICCy],
                                    [w*dt]])

            result = np.add(np.matmul(rotation, frame), translation)
            result = result.T

            self.x = result[0][0]
            self.y = result[0][1]
            self.heading = result[0][2]
        
if __name__ ==  "__main__":
    node = OdometryEncoder(node_name="OdometryEncoder")
    rospy.spin()