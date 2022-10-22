#!/usr/bin/env python
# license removed for brevity

class Coord:
    def __init__(self,x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

list = []
list.append(Coord(0.5,0,0))
list.append(Coord(0.3,0,1.0))
list.append(Coord(0.1,0.7,1.0))

import rospy
from std_msgs.msg import String, Float32

def talker():
    pub = rospy.Publisher('chatter', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz


    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        for obj in list:
            hello_str = obj.x
            y_y = obj.y
            theta_t = obj.theta
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            #pub.publish("yoyo")
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
