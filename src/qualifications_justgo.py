#!/usr/bin/env python
# license removed for brevity

import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time

state_data = 0
start_tri_flag = 0
reset_flag = 0
finish_flag = 0

def voltage_cb(data):
    global state_data, start_tri_flag, reset_flag, finish_flag
    data = data.data
    if data < 2.5:
        reset_flag = 1
        state_data = 0
        finish_flag = 0
        pub1.publish(state_data)
    elif data > 10 and start_tri_flag == 0 and state_data == 0 and state_data != 64 and finish_flag == 0:
        print(data, start_tri_flag, state_data)
        state_data = 64
        reset_flag = 0
        print('qq')
        start_tri_flag = 1

def depth_cb(data):
    global depth_data
    depth_data = data.data

rospy.init_node('state',anonymous=True)
pub1 = rospy.Publisher('/state',Int32,queue_size=10)

rospy.Subscriber('/voltage', Float32, voltage_cb)
rospy.Subscriber('/depth', Float32, depth_cb)

while not rospy.is_shutdown():
    if start_tri_flag == 1:
        pub1.publish(64)
        time.sleep(1)
        pub1.publish(0)
        time.sleep(1)
        pub1.publish(64)
        time.sleep(1)
        pub1.publish(128)
        time.sleep(1)
        pub1.publish(2)
        time.sleep(1)
        state_data = 3
        pub1.publish(3)
        start_tri_flag = 0
    elif state_data == 3 and depth_data > 0.55:
        state_data = 15
        pub1.publish(15)
        for i in range(40):
            time.sleep(1)
            if reset_flag == 1:
                reset_flag = 0
                break
        finish_flag = 1
        state_data = 0
        pub1.publish(0)
        
