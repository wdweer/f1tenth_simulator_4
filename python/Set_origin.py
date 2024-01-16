#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import time
# Initialize the ROS node
rospy.init_node('set_model_state')

# Publisher setup
state_pub = rospy.Publisher('/steer_bot/gazebo/set_model_state', ModelState, queue_size=1)

# Rate of publishing
rate = rospy.Rate(20)  # 20 Hz
start_time=time.time()
while not rospy.is_shutdown():
    # Prepare the ModelState message
    if time.time()-start_time < 1:
        
    
        state_msg = ModelState()
        state_msg.model_name = "steer_bot"
        state_msg.reference_frame = "world"
        state_msg.pose = Pose(
          position=Point(x=0, y=0, z=1),
          orientation=Quaternion(x=0, y=0, z=0, w=1)
    )
        state_msg.twist = Twist(
        linear=Point(x=0, y=0, z=0),
        angular=Point(x=0, y=0, z=0)
    )

    # Publish the message
        state_pub.publish(state_msg)

    # Sleep to maintain the loop rate
        rate.sleep()
    else:
        break