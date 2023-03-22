#!/usr/bin/env python3

import os
import rospy
import time
from smbus2 import SMBus





def line_follower():
    curr_time = 0
    prev_time = 0
    delta_t = 0
    slow_data = False
    # publish message every 1 second
    

    bus = SMBus(1)
    try:
        temp = bus.read_byte_data(62, 17)
        theta_ref = bin(temp)[2:].zfill(8)
    except:
        print("Ei saanud joonelugerilt infot.")

    curr_time = time.time()
    
    delta_t = curr_time - prev_time

    prev_time = curr_time
    #print("delta_t : ", self.delta_t)

    if delta_t >= 0.1:
        slow_data = True
        
    else:
        slow_data = False

    '''
    self.line_values = []
    for i, value in enumerate(theta_ref):
        if value =='1':
            self.line_values.append(i + 1)
    
    self.pub2.publish(self.line_values)
    '''

    return theta_ref
