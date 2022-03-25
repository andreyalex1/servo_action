#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#Subscribes to sensor_msgs.JointState and sends received spped
#to CAN bus for stepper control
#Software designed by Andrey Smirnov, Moscow State Unversity, 2022

import os
import time
import numpy as np
import rospy
from std_msgs.msg import UInt16MultiArray
from trajectory_msgs.msg import JointTrajectory
import struct
import sys

servo_CAN_IDs = [15,]
t = time.time()
sub_dict = {}

rospy.init_node('Manip_Control_Encoder')
pub = rospy.Publisher('CAN_Tx_Buffer', UInt16MultiArray, queue_size=1)

class JointStateSubscriber:
    name = None
    sub = None
    ids = None
    def __init__(self, id):
        print("Subcriber Started!")
        self.id = id
        self.name = "Servo_Control"
        self.sub = rospy.Subscriber(self.name, JointTrajectory, self.Subscriber_callback)


    def Subscriber_callback(self, JointData):
        global pub
        TxBuffer_ROS = UInt16MultiArray()
        for point in JointData:
            velocity = point.velocities[0]
            position = point.positions[0]
            data = arr1 = list(struct.pack("<f", velocity))
            data = arr2 = list(struct.pack("<f", position))
            TxBuffer_ROS.data.append(self.ids[0])
            for i in arr2:
                TxBuffer_ROS.data.append(int(i))
            for i in arr1:
                TxBuffer_ROS.data.append(int(i))
            pub.publish (TxBuffer_ROS)
            time.sleep(0.005)
        print("Published!")


    
def Publisher():
    global pub
    print("Publisher Started!")
    pub = rospy.Publisher("/CAN_Tx_Buffer", UInt16MultiArray, queue_size=1000)

try:
    Publisher()
    obj = JointStateSubscriber(servo_CAN_IDs)
    while True:
        None
except ValueError:
    print("Not a number")
except KeyboardInterrupt:
    sys.exit()
