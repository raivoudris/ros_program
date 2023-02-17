#!/usr/bin/env python3

import os
import rospy 
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import Range


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        #self.sub = rospy.Subscriber('/bestestduckiebot/front_center_tof_driver_node/range', Range, self.callback)

    #def callback(self, data):
        #rospy.loginfo("I heard %s", data.range)


if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()