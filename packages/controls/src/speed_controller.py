#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from duckietown_msgs.msg import WheelsCmdStamped
from include.pid import PID

class SpeedControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(SpeedControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.wheel_pub = rospy.Publisher(f"/{os.environ['VEHICLE_NAME']}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        self.log("Initialized")

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            self.log("sending command")

            
            wheelsCmd = WheelsCmdStamped()
            header = Header()
            wheelsCmd.header = header
            wheelsCmd.vel_right = 0.5
            wheelsCmd.vel_left = 0.5
            self.wheel_pub.publish(wheelsCmd)

            rate.sleep()

    def on_shutdown(self):

        #send a zero velocity wheel command

            wheelsCmd = WheelsCmdStamped()
            header = Header()
            wheelsCmd.header = header
            wheelsCmd.vel_right = 0
            wheelsCmd.vel_left = 0
            self.wheel_pub.publish(wheelsCmd)


if __name__ == '__main__':
    # create the node
    node = SpeedControlNode(node_name='speed_control_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
