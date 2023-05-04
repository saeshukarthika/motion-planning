#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from duckietown_msgs.msg import WheelsCmdStamped
from include.pid import PID
from as_msgs.msg import WheelOdometry

class SpeedControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(SpeedControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.wheel_pub = rospy.Publisher(f"/{os.environ['VEHICLE_NAME']}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.left_speed = 0
        self.right_speed = 0
        self.left_goal = 0
        self.right_goal = 0
        self.left_pid = PID(0,0,0,0)
        self.right_pid = PID(0,0,0,0)

        self.odometery_sub = rospy.Subscriber(f"~wheel_odometry/odometry",WheelOdometry,self.odometry_cb)
        self.goal_speed_sub = rospy.Subscriber(f"~speed_control/goal",WheelOdometry,self.goal_speed_cb)
        
        self.log("Initialized")

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(20) # 1Hz
        while not rospy.is_shutdown():
            self.log("sending command")


            wheelsCmd = WheelsCmdStamped()
            header = Header()
            wheelsCmd.header = header
            wheelsCmd.vel_right = self.right_pid.update(self.right_speed,0.05)
            wheelsCmd.vel_left = self.left_pid.update(self.left_speed,0.05)
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

    def odometry_cb(self,data):
        self.left_speed = data.left_wheel_velocity
        self.right_speed = data.right_wheel_velocity
        self.log(data)

    def goal_speed_cb(self,data):
        self.left_goal = data.left_wheel_velocity
        self.right_goal = data.right_wheel_velocity
        #self.left_goal = 0.5
        #self.right_goal = 0.5
        self.left_pid.set(self.left_goal)
        self.right_pid.set(self.right_goal)
        self.log(data)


if __name__ == '__main__':
    # create the node
    node = SpeedControlNode(node_name='speed_control_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
