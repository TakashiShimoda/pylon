#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
2019-12-2 yamada
ロボットのオドメトリを利用しない場合(旋回時)
    実際の畑など、摩擦を考慮する
    
・control_no1()
見えているパイロンまでの距離を計測し、そのパイロンまである距離になるまで移動する。

・control_no2()
移動した地点から右に90°旋回する
次のパイロンが見えるまで旋回し、停止。

'''

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from std_msgs.msg import String
from math import radians, copysign, sqrt, pow, pi
import math
from nav_msgs.msg import Odometry

class GetOdom():
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.posi_x = 0.0
        self.posi_y = 0.0
        self.ori_z = 0.0
        
    def odom_callback(self, data):
        self.posi_x = data.pose.pose.position.x
        self.posi_y = data.pose.pose.position.y
        self.ori_z = data.pose.pose.orientation.z
        
class RobotControl():
    def __init__(self):
        self.get_odom = GetOdom()
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        ROS_RATE = 30
        self.R = rospy.Rate(ROS_RATE)
        self.LINEAR_SPEED = 1.8
        self.ANGULAR_SPEED = 0.8
        self.SWING_RADIUS = 300
        self.ROTATION_RADIUS = 0.2
        self.Kp = 3.0
        self.Ki = 1.0
        self.tmp_E = 0.0
        self.move_cmd = Twist()
            
    def main_control(self):
        rospy.logerr("rosbag_start")
        # rospy.sleep(10)
        self.control_no1()
        rospy.sleep(1)
        self.control_no2()
        rospy.sleep(1)
        self.control_no1()
        rospy.sleep(1)
        self.control_no3()
        rospy.sleep(1)
        self.control_no1()
        rospy.sleep(1)
        self.control_no4()
        rospy.sleep(1)
        self.control_no1()
        rospy.sleep(1)
        self.control_no5()
        rospy.sleep(1)
        self.control_no1()
        
        
    def control_no1(self):
        rospy.loginfo('start_robot_control_no.1')
        rospy.sleep(5)
        
        self.current_position_x = self.get_odom.posi_x
        self.current_position_y = self.get_odom.posi_y
        
        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
        self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
        self.target_travel_distance_save = self.target_travel_distance
        print(self.target_travel_distance)
        
        while self.target_travel_distance > 160.0 and not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            self.target_pylon = rospy.get_param('/pylon_detection/param_z', 0)
            self.target_travel_distance = self.target_pylon - self.SWING_RADIUS
            self.lateral_distance = rospy.get_param('/pylon_detection/param_x', 0)
            self.target_travel_distance_save = self.target_travel_distance
            # red_area = rospy.get_param('/color_extract/b_area', 0)
            if (self.target_travel_distance > 1500):
                self.move_cmd.angular.z = 0.0
                self.current_position_x_1 = self.get_odom.posi_x
                self.current_position_y_1 = self.get_odom.posi_y
                self.target_travel_distance = sqrt(pow(self.current_position_x_1 - self.current_position_x, 2) + pow(self.current_position_y_1 - self.current_position_y, 2))
                self.target_travel_distance = self.target_pylon - self.target_travel_distance
                
            else:
                self.angular_z_speed = self.angle_PID(-32.0, self.lateral_distance, self.Kp, self.Ki)
                self.move_cmd.angular.z = self.angular_z_speed
                self.target_travel_distance = self.target_travel_distance_save
                
            # rospy.loginfo('move_to_x_axes_distance=%f' % self.target_travel_distance)
            self.R.sleep()
            # rospy.loginfo('pylon_y_distance=%f' % self.lateral_distance)
            print(self.get_odom.ori_z)
            
        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(3)
        
    def control_no2(self):
        rospy.loginfo('start_robot_control_no.2')
        
        self.current_position_x = self.get_odom.posi_x
        self.current_position_y = self.get_odom.posi_y
        self.current_orientation_z = self.get_odom.ori_z
        print(self.current_orientation_z)
        rospy.sleep(5)

        
        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.move_cmd.angular.z = - self.ANGULAR_SPEED
        
        # self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        self.target_orientation_z = -0.5
        self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        if self.current_orientation_z > 0.0:
            while self.current_orientation_z > 0.0 and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                print(("1", self.current_orientation_z))
                self.R.sleep()
                
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print(("2", self.current_orientation_z))
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
            
                
        else:
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print(("3", self.current_orientation_z))
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
        # while self.base_orientation_z < 0.5 and not rospy.is_shutdown():
        #     self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        #     rospy.loginfo(self.base_orientation_z)
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     print(self.target_pylon_x)
        #     self.R.sleep()
            
        # while self.target_pylon_x != -32.0 and not rospy.is_shutdown():
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     self.R.sleep()
    
        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        self.cmd_vel.publish(Twist())
        rospy.sleep(3)

        
    def control_no3(self):
        rospy.loginfo('start_robot_control_no.3')
        
        self.current_position_x = self.get_odom.posi_x
        self.current_position_y = self.get_odom.posi_y
        self.current_orientation_z = self.get_odom.ori_z
        print(self.current_orientation_z)
        rospy.sleep(5)

        
        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.move_cmd.angular.z = - self.ANGULAR_SPEED
        
        # self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        self.target_orientation_z = -0.9
        self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        if self.current_orientation_z > -0.7:
            while self.current_orientation_z > -0.7 and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                print("1", self.current_orientation_z)
                self.R.sleep()
                
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print("2", self.current_orientation_z)
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
            
                
        else:
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print("3", self.current_orientation_z)
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
        # while self.base_orientation_z < 0.5 and not rospy.is_shutdown():
        #     self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        #     rospy.loginfo(self.base_orientation_z)
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     print self.target_pylon_x
        #     self.R.sleep()
            
        # while self.target_pylon_x != -32.0 and not rospy.is_shutdown():
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     self.R.sleep()
    
        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        self.cmd_vel.publish(Twist())
        rospy.sleep(3)

    def control_no4(self):
        rospy.loginfo('start_robot_control_no.4')
        
        self.current_position_x = self.get_odom.posi_x
        self.current_position_y = self.get_odom.posi_y
        self.current_orientation_z = self.get_odom.ori_z
        print(self.current_orientation_z)
        rospy.sleep(5)

        
        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.move_cmd.angular.z = - self.ANGULAR_SPEED
        
        # self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        self.target_orientation_z = 0.4
        self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        if self.current_orientation_z > 0.9:
            while self.current_orientation_z > 0.9 and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                print("1", self.current_orientation_z)
                self.R.sleep()
                
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print("2", self.current_orientation_z)
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
            
                
        else:
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print("3", self.current_orientation_z)
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
        # while self.base_orientation_z < 0.5 and not rospy.is_shutdown():
        #     self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        #     rospy.loginfo(self.base_orientation_z)
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     print self.target_pylon_x
        #     self.R.sleep()
            
        # while self.target_pylon_x != -32.0 and not rospy.is_shutdown():
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     self.R.sleep()
    
        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        self.cmd_vel.publish(Twist())
        rospy.sleep(3)

    def control_no5(self):
        rospy.loginfo('start_robot_control_no.5')
        
        self.current_position_x = self.get_odom.posi_x
        self.current_position_y = self.get_odom.posi_y
        self.current_orientation_z = self.get_odom.ori_z
        print(self.current_orientation_z)
        rospy.sleep(5)

        
        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.move_cmd.angular.z = - self.ANGULAR_SPEED
        
        # self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        self.target_orientation_z = -0.2
        self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        if self.current_orientation_z > 0.2:
            while self.current_orientation_z > 0.2 and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                print("1", self.current_orientation_z)
                self.R.sleep()
                
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print ("2", self.current_orientation_z)
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
            
                
        else:
            while self.current_orientation_z > self.target_orientation_z and not rospy.is_shutdown():
                self.current_orientation_z = self.get_odom.ori_z
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                print("3", self.current_orientation_z)
                self.R.sleep()

            while self.target_pylon_x < 0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
            while self.target_pylon_x > -32.0 and not rospy.is_shutdown():
                self.move_cmd.angular.z = - self.ANGULAR_SPEED
                self.cmd_vel.publish(self.move_cmd)
                self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
                self.R.sleep()
                
        # while self.base_orientation_z < 0.5 and not rospy.is_shutdown():
        #     self.base_orientation_z = self.current_orientation_z - self.get_odom.ori_z
        #     rospy.loginfo(self.base_orientation_z)
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     print self.target_pylon_x
        #     self.R.sleep()
            
        # while self.target_pylon_x != -32.0 and not rospy.is_shutdown():
        #     self.move_cmd.angular.z = - self.ANGULAR_SPEED
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.target_pylon_x = rospy.get_param('/pylon_detection/param_x', 0)
        #     self.R.sleep()
    
        rospy.loginfo('end_robot_control')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        self.cmd_vel.publish(Twist())
        rospy.sleep(3)


    def a(self, a):
        if self.current_orientation_z > a and not rospy.is_shutdown():
            print(a)

    #self.a(0.9)
    #self.a(0.2)
    #self.a(self.target_orientation_z)
        
    def angle_PID(self, R, Y, Kp, Ki):
        # rospy.loginfo('Y = %f' % Y)
        E = R - Y
        # rospy.loginfo('E = %f' % E)
        self.angular_z_speed = 1.0 * (Kp * E + Ki * (E - self.tmp_E))
        if self.angular_z_speed > 0.5:
            self.angular_z_speed = 0.5
        elif self.angular_z_speed < -0.5:
            self.angular_z_speed = -0.5
        self.tmp_E = E
        # rospy.loginfo('ang_z = %f' % self.angular_z_speed)
        return self.angular_z_speed
    
    def shutdown(self):
        rospy.loginfo('Stopping the robot...')
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        
def atler_control_main():
    rospy.init_node('Atler_Control')
    robot_control = RobotControl()
    robot_control.main_control()

        
if __name__ == '__main__':
    atler_control_main()
