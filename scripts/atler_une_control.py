#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from math import radians, copysign, sqrt, pow, pi
# from std_msgs.msg import Empty

class GetOdom:
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.posi_x = 0.0
        self.posi_y = 0.0
        self.ori_z = 0.0

    def odom_callback(self, data):
        self.posi_x = data.pose.pose.position.x
        self.posi_y = data.pose.pose.position.y
        self.pri_z = data.pose.pose.orientation.z

class AtlerControl:
    def __init__(self):
        self.get_odom = GetOdom()
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('/atler_pixel_detect_data', Int64MultiArray, self.detect_data_callback)
        rospy.Subscriber('/target_trajectory', Int64MultiArray, self.target_callback)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        ROS_TATE = 30
        self.R = rospy.Rate(ROS_TATE)
        self.LINEAR_SPEED = 1.7
        self.AGULAR_SPEED = 0.95
        self.SWING_RADIUS = 300
        self.Kp = 3.0
        self.Ki = 1.0
        self.tmp_E = 0.0
        self.move_cmd = Twist()

    def detect_data_callback(self, detect_data):
        if detect_data:
            self.atler_data = (detect_data.data[0], detect_data.data[1])
        else:
            self.atler_data = (0,0)
        # rospy.logerr(self.atler_data)

    def target_callback(self, target_data):
        self.atler_target_data = (target_data.data[0], target_data.data[1])
        # rospy.loginfo(self.atler_target_data)

    def main_control(self):
        self.control_no1()
        self.une_control_no1()
        self.control_no2()

    def control_no1(self):
        rospy.loginfo('start_atler_control_no.1')
        rospy.sleep(5)
        self.current_position_x = self.get_odom.posi_x
        self.current_position_y = self.get_odom.posi_y

        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
        self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
        self.target_travel_distance_save = self.target_travel_distance
        self.atler_data = (0, 0)
        
        while self.atler_data[0] == 0 and not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
            self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
            self.lateral_distance = rospy.get_param('/pylon_detection/param_x', 0)
            self.target_travel_distance_save = self.target_travel_distance
            self.angular_z_speed = self.angle_PID(-32.0, self.lateral_distance, self.Kp, self.Ki)
            self.move_cmd.angular.z = self.angular_z_speed
            self.target_travel_distance = self.target_travel_distance_save
            print(self.atler_data[0])
            self.R.sleep()

        while self.atler_data[1] < 180 and not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
            self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
            self.lateral_distance = rospy.get_param('/pylon_detection/param_x', 0)
            self.target_travel_distance_save = self.target_travel_distance
            self.angular_z_speed = self.angle_PID(-32.0, self.lateral_distance, self.Kp, self.Ki)
            self.move_cmd.angular.z = self.angular_z_speed
            self.target_travel_distance = self.target_travel_distance_save
            print(self.atler_data[1])
            self.R.sleep()

        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(3)

    def control_no2(self):
        rospy.loginfo('start_atler_control_no.1')
        rospy.sleep(5)
        self.current_position_x = self.get_odom.posi_x
        self.current_position_y = self.get_odom.posi_y

        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
        self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
        self.target_travel_distance_save = self.target_travel_distance

        while self.atler_data[1] > 50 and not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
            self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
            self.lateral_distance = rospy.get_param('/pylon_detection/param_x', 0)
            self.target_travel_distance_save = self.target_travel_distance
            self.angular_z_speed = self.angle_PID(-32.0, self.lateral_distance, self.Kp, self.Ki)
            self.move_cmd.angular.z = self.angular_z_speed
            self.target_travel_distance = self.target_travel_distance_save
            print(self.atler_data[1])
            self.R.sleep()
            
        while self.target_pylon > 380 and not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
            self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
            self.lateral_distance = rospy.get_param('/pylon_detection/param_x', 0)
            self.target_travel_distance_save = self.target_travel_distance
            self.angular_z_speed = self.angle_PID(-32.0, self.lateral_distance, self.Kp, self.Ki)
            self.move_cmd.angular.z = self.angular_z_speed
            self.target_travel_distance = self.target_travel_distance_save
            self.R.sleep()

        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(3)

    def une_control_no1(self):
        rospy.loginfo('start_atler_une_control_no.1')
        rospy.sleep(5)

        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.move_cmd.angular.z = self.AGULAR_SPEED
        self.cmd_vel.publish(self.move_cmd)

        while self.atler_data[0] < 320 and not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            print(self.atler_data[0])
            self.R.sleep()

        while self.atler_data[1] > 180 and not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            print(self.atler_data[1])
            self.R.sleep()

        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(3)

    def angle_PID(self, R, Y, Kp, Ki):
        E = R - Y
        self.angular_z_speed = 1.0 * (Kp * E + Ki * (E - self.tmp_E))
        if self.angular_z_speed > 0.4:
            self.angular_z_speed = 0.4
        elif self.angular_z_speed < -0.4:
            self.angular_z_speed = -0.4
        self.tmp_E = E
        return self.angular_z_speed
    
    def mathematics_model(self):
        pass

    def shutdown(self):
        rospy.loginfo('Stopping the robot...')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

def atler_une_control_main():
    rospy.init_node('Atler_Control')
    robot_control = AtlerControl()
    # while not rospy.is_shutdown():
    #     robot_control.main_control()

    for i in range(1):
        robot_control.main_control()
        
if __name__ == '__main__':
    atler_une_control_main()
