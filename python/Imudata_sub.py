#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

rospy.init_node('Imudata_sub')

def callback(data):
    print(data)
    
Imu_sub=rospy.Subscriber("/steer_bot/imu_data", Imu, callback)
rospy.spin()