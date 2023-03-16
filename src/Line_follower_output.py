#!/usr/bin/env python3

import os
import rospy
from smbus2 import SMBus
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from std_msgs.msg import Int32

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('line_follower', String, queue_size = 0)
        #self.pub2 = rospy.Publisher('line_values', Int32, queue_size = 1)
        

    def line_follower(self, data):
        self.linefollower = data.data

    def values(self, data):
        self.array = data.data
        

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(30) # 1Hz  
        while not rospy.is_shutdown():
            bus = SMBus(1)
            temp = bus.read_byte_data(62, 17)
            theta_ref = bin(temp)[2:].zfill(8)

            '''
            self.line_values = []
            for i, value in enumerate(theta_ref):
                if value =='1':
                    self.line_values.append(i + 1)
            
            self.pub2.publish(self.line_values)
            '''

            self.pub.publish(theta_ref)
            
            
            
            rate.sleep()

                
if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='Line_follower_output')
    # run node
    node.run()
    # keep spinning
    
    rospy.spin()