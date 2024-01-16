#!/usr/bin/env python

import rospy
import time
from ackermann_msgs.msg import AckermannDriveStamped

def publish_drive_commands():
    # Initialize the ROS Node
    rospy.init_node('ackermann_drive_publisher')

    # Create a publisher object
    pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=5)

    # Set the rate of publishing
    rate = rospy.Rate(10) # 10 Hz


        # Create a new AckermannDriveStamped message
    ackermann_forward = AckermannDriveStamped()

        # Fill in the fields of the message
    ackermann_forward.drive.speed = 1.0  # Speed value
    ackermann_forward.drive.steering_angle = 0.5  # Steering angle
    ackermann_forward.drive.acceleration = 1.0
    ackermann_forward.drive.jerk = 0.5
    
    ackermann_acceleration = AckermannDriveStamped()
    ackermann_acceleration.drive.speed = 2.0
    ackermann_acceleration.drive.acceleration = 3.0
    ackermann_acceleration.drive.steering_angle = 0.5
    

    ackermann_stop=AckermannDriveStamped()
    ackermann_stop.drive.speed=0
    ackermann_stop.drive.steering_angle = 0
    start_time=time.time()

    while not rospy.is_shutdown():
        if time.time() - start_time < 10.0:
            pub.publish(ackermann_forward)
        elif time.time()-start_time < 20.0:
            pub.publish(ackermann_acceleration)
        else:
            rospy.logwarn(" 20 seconds left, Stop!! ")
            pub.publish(ackermann_stop)
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_drive_commands()
    except rospy.ROSInterruptException:
        pass
