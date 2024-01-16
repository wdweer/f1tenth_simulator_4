#!/usr/bin/env python 

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import time

rospy.init_node('Stop')

stop_msg1=AckermannDriveStamped()
stop_msg1.drive.speed=0
stop_msg1.drive.steering_angle=0
stop_msg1.drive.acceleration=0
stop_msg1.drive.jerk=0


pub=rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=5)
r=rospy.Rate(100000)
start_time=time.time()
while not rospy.is_shutdown():
    pub.publish(stop_msg1)

    r.sleep()