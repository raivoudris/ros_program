#!/usr/bin/env python3

import os
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        
        self.pub = rospy.Publisher('posintdelta', Float32, queue_size = 0)
        
        self.sub2 = rospy.Subscriber('left_wheel_travel', Pose, self.left_odom)
        self.sub4 = rospy.Subscriber('line_follower', String, self.line_follower)

        self.In_al = 0
        self.delta_t = 1
        self.prev_e = 0 

    def line_follower(self, data):
        self.theta_ref = data.data

    def left_odom (self, data):
        self.delta_t = data.position.z
        print("delta_t from PID ", self.delta_t)

    def run(self):
        
        # publish message every 1 second
        rate = rospy.Rate(30) # 1Hz  
        while not rospy.is_shutdown():
            
            #Joone järgimine
            Kp = 0.045
            Ki = 0.02 
            Kd = 1.2    #Vähendab roboti ujumist 

            #P - proportional

            try:
                rospy.wait_for_message('line_follower', String, None)
            except:
                print("Ei saa infot teistest failidest.")
            self.line_values = []
            for i, value in enumerate(self.theta_ref):
                if value =='1':
                    self.line_values.append(i + 1)
            if len(self.line_values) != 0:
                self.position = sum(self.line_values) / len(self.line_values)
                error = 4.5 - self.position
                
                #I - integral
                self.In_al = self.In_al + (self.delta_t * error) #in_al = integral
                #print("integral: ", self.In_al)
                #anti-windup - väldib integraalvea liigset suurenemist
                self.In_al = max(min(self.In_al, 1.8), -1.8)
                #print("integral + anti-windup: ", self.In_al)
                #D - derivative
                err_der = (error - self.prev_e) / self.delta_t
                Kpe = Kp * error
                Kie = Ki * self.In_al
                Kde = Kd * err_der
                #print("Kpe: ", Kpe)
                self.PID = Kpe + Kie + Kde
                self.prev_e = error
            else: 
                self.PID = 0

                #print("PID FAILIS: ", self.PID)
            
            self.pub.publish(self.PID)
            rate.sleep()

                
if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='posintdelta')
    # run node
    node.run()
    # keep spinning
    
    rospy.spin()