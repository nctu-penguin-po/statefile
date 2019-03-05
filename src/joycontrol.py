#!/usr/bin/env python
# license removed for brevity

import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
import time

joy_button_data = [0, 0, 0]
state_data = 0

def joyB_cb(data):
    global joy_button_data, state_data
    joy_button_data = data.data
    right_sig = joy_button_data[0]
    front_sig = joy_button_data[2]
    if (right_sig >> 3)%2 == 1:
        state_data = 0
        pub1.publish(state_data)
    elif (right_sig >> 1)%2 == 1:
        state_data = state_data ^ 1
        pub1.publish(state_data)
    elif right_sig%2 == 1:
        state_data = state_data ^ 2
        pub1.publish(state_data)

rospy.init_node('state',anonymous=True)

rospy.Subscriber('/joy/button', Int32MultiArray, joyB_cb)

pub1 = rospy.Publisher('/state', Int32, queue_size=10)
while not rospy.is_shutdown():
    pass
