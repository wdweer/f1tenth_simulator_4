#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import time
class Transformer():
    def __init__(self):
        rospy.init_node("Transformer")
        self.Ackermann_sub=rospy.Subscriber('/vesc/ackermann_cmd',AckermannDriveStamped,self.Ackermann_to_cmd_vel_callback)
        self.cmd_vel_pub=rospy.Publisher("/steer_bot/ackermann_steering_controller/cmd_vel",Twist, queue_size=5)
        self.model_state_sub=rospy.Subscriber("/steer_bot/gazebo/model_states", ModelStates, self.model_states_to_vesc_odom_callback)
        self.vesc_odom_pub=rospy.Publisher("/vesc/odom", Odometry, queue_size=5)
        self.laser_scan_sub=rospy.Subscriber("/steer_bot/steer_bot/laser_scan", LaserScan, self.scan_callback)
        self.scan_pub=rospy.Publisher("/scan" ,LaserScan, queue_size=5)
        rate = rospy.Rate(1000)
        self.velocity=0
        self.acceleration=0
        self.steering_angle=0
        self.jerk=0
        self.scan=LaserScan()
        self.start_time=time.time()
        self.cmd_vel_data = Twist()
        self.Odom_data = Odometry()
        self.cmd_vel_data.linear.x=0
        self.cmd_vel_data.angular.z=0
        while not rospy.is_shutdown():
            self.Ackermann_to_cmd_vel()
        #     Basic_cmd_vel=Twist()
        #     Basic_cmd_vel.linear.x=0
        #     Basic_cmd_vel.angular.z=0
        #     self.cmd_vel_pub.publish(Basic_cmd_vel)
            rate.sleep()
            
    def Ackermann_to_cmd_vel_callback(self,data):
        self.start_time=time.time()
        # rospy.loginfo("Received Ackermann data: %s", data)
        # No need to create a new AckermannDriveStamped object, as data is already in the correct format
        self.velocity = data.drive.speed
        self.steering_angle = data.drive.steering_angle
        self.acceleration=data.drive.acceleration
        self.jerk=data.drive.jerk
        self.cmd_vel_data.linear.x = self.velocity
        
    def Ackermann_to_cmd_vel(self):
        # rospy.loginfo("Received Ackermann data: %s", self.cmd_vel_data)
        # Create a new Twist message and populate it
        self.acceleration =self.acceleration+(time.time()-self.start_time)*self.jerk
        self.cmd_vel_data.linear.x =self.velocity+(time.time()-self.start_time)*self.acceleration
        self.cmd_vel_data.angular.z = self.steering_angle

        # Publish the converted message
        self.cmd_vel_pub.publish(self.cmd_vel_data)
        
    def model_states_to_vesc_odom_callback(self,data):
        robot_name="steer_bot"
        robot_index = data.name.index(robot_name)
        self.Odom_data.pose.pose=data.pose[robot_index]
        self.Odom_data.twist.twist=data.twist[robot_index]
        self.vesc_odom_pub.publish(self.Odom_data)
        
    def scan_callback(self,data):
        self.scan_pub.publish(data)
        # print(data)
        
if __name__=='__main__':
    try:
        transformer=Transformer()
    except rospy.ROSInterruptException:
        pass