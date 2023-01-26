
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class OdometryEncoder(DTROS):
    def __init__(self, node_name) -> None:
        super(OdometryEncoder, self).__init__(node_name=node_name, NodeType=NodeType.PUBLISHER)
        
        self.encoder_publisher = rospy.Publisher
        
    