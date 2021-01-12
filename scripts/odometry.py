#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from math import pi
import numpy as np
from math import sqrt, cos, sin
import tf,tf2_ros
from geometry_msgs.msg import Twist, Pose, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# 車輪の半径[m]
D_RIGHT=0.07
D_LEFT=0.07
# 車輪間の距離[m]
TREAD=0.315

def calc_input(omega_r, omega_l):
    """
    パラメータ
    ----------
    omega_r, omega_l : float
        車輪の角速度

    戻り値
    ----------
    v : float
        ドリーの線速度
        
    yawrate : float
        ドリーのyaw角速度
    """
    
    # 両車輪の速度
    vr=D_RIGHT*omega_r
    vl=D_LEFT*omega_l
    # ドリーの速度
    v=(vr+vl)/2.0
    # ドリーのyaw角速度
    yawrate=(vr-vl)/(TREAD/2.0)
    # 配列生成
    u = np.array([v,yawrate])
    return u
    
def MotionModel(x, u, dt):
    """
    パラメータ
    ----------
    x : array
        最新のタイムスタンプ時のドリーの姿勢
        ドリーの位置と方向(loc_x,loc_y,theta)
    u : array
        ドリーの速度
        ドリーの線、yaw角速度
    dt : float
        現在と最新のタイムスタンプの差
        
    戻り値
    ----------
    x : array
        現在のタイムスタンプにおける推定されたドリーの姿勢
        ドリーの位置と方向(loc_x,loc_y,theta)
    """
    loc_x = x[0]
    loc_y = x[1]
    theta = x[2]
    v = u[0]
    omega = u[1]
    loc_x = loc_x + np.cos(theta+np.pi/2)*v*dt
    loc_y = loc_y + np.sin(theta+np.pi/2)*v*dt
    theta = theta + omega*dt
    theta = PItoPI(theta)
    x=np.array([loc_x,loc_y,theta])
    return x

def PItoPI(angle):
    while angle>=np.pi:
        angle=angle-2*np.pi
    while angle<=-np.pi:
        angle=angle+2*np.pi
    return angle

class ParamConverter:
    def __init__(self):
        rospy.Subscriber("/left_angle", Float32, self.left_callback)
        rospy.Subscriber("/right_angle", Float32, self.right_callback)
        self.left = 0.0
        self.right = 0.0
        self.left_angle_rad = 0.0
        self.right_angle_rad = 0.0
        
    def left_callback(self, data):
        self.left = np.array((data.data), dtype = 'float32')
        self.left_angle_rad = self.left*pi/180
        
    def right_callback(self, data):
        self.right = np.array((data.data), dtype = 'float32')
        self.right_angle_rad = self.right*pi/180
        
class Measurement:
    def __init__(self):
        self.param_converter = ParamConverter()
        self.left_speed_pub = rospy.Publisher("/left_speed", Float32, queue_size=1)
        self.left_pos_pub = rospy.Publisher("/left_position", Float32, queue_size=1)
        self.right_speed_pub = rospy.Publisher("/right_speed", Float32, queue_size=1)
        self.right_pos_pub = rospy.Publisher("/right_position", Float32, queue_size=1)
        self.time_after = 0.0
        self.time_before = 0.0
        self.time_width = 0.0
        self.left_angle_before_rad = 0.0
        self.left_angle_width_rad = 0.0
        self.left_speed_rads = 0.0
        self.right_angle_width_rad = 0.0
        self.right_angle_before_rad = 0.0
        self.right_speed_rads = 0.0
        self.left_angle_width = 0.0
        self.left_angle =0.0
        self.left_angle_before = 0.0
        self.left_position = 0.0
        self.p_l = 0.0
        self.right_angle_width = 0.0
        self.right_angle = 0.0
        self.right_angle_before = 0.0
        self.right_position = 0.0
        self.p_r = 0.0
            
        
    def measurement_main_no1(self):
        self.timer()
        self.left_speed()
        self.right_speed()
        self.left_posi()
        self.right_posi()
        self.pub_param()
        
    def timer(self):
        self.time_after = time.time()
        self.time_width = self.time_after - self.time_before
        self.time_before = self.time_after
        
    def left_speed(self):
        self.left_angle_width_rad = self.left_angle_rad - self.left_angle_before_rad
        self.left_angle_before_rad = self.left_angle_rad
        self.left_speed_rads = self.left_angle_width_rad / self.time_width
        if self.left_speed_rads < 0.1:
            self.left_speed_rads = 0.0
            
    def right_speed(self):
        self.right_angle_width_rad = self.right_angle_rad - self.right_angle_before_rad
        self.right_angle_before_rad = self.right_angle_rad
        self.right_speed_rads = self.right_angle_width_rad / self.time_width
        if self.right_speed_rads < 0.1:
            self.right_speed_rads = 0.0
            
    def left_posi(self):
        self.left_angle_width = self.left_angle - self.left_angle_before
        # rospy.sleep(0.01)
        self.left_angle_before = self.left_angle
        self.left_position = self.left_angle_width
        if self.left_position > 0.50:
            self.p_l += self.left_position
            
    def right_posi(self):
        self.right_angle_width = self.right_angle - self.right_angle_before
        # rospy.sleep(0.01)
        self.right_angle_before = self.right_angle
        self.right_position = self.right_angle_width
        if self.right_position > 0.50:
            self.p_r += self.right_position
            
    def paramreset(self):
        self.p_l = self.p_l - self.p_l
        self.p_r = self.p_r - self.p_r
        rospy.sleep(0.01)
        
    def pub_param(self):
        self.left_speed_pub.publish(self.left_speed_rads)
        self.right_speed_pub.publish(self.right_speed_rads)
        self.left_pos_pub.publish(self.p_l)
        self.right_pos_pub.publish(self.p_r)
        print(self.p_l)
        print(self.p_r)

class ATLEROdom:
    def __init__(self):
        # self.measurement = Measurement()
        self.odo = Odometry()
        self.odo.header.frame_id = 'odom'
        self.odo.child_frame_id = 'base_link'
        self.odocount = 0
        self.pub_odo = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.x = np.array([0.0, 0.0, 0.0])
        self.current_time = time.time()
        self.last_time = time.time()
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.odom_trans = TransformStamped()
        self.odom_trans.header.stamp = rospy.Time.now()
        self.odom_trans.header.frame_id = "odom"
        self.odom_trans.child_frame_id = "base_link"
        
        self.right_velocity = 0
        self.left_velocity = 0
        
        self.right_angle_trans = TransformStamped()
        self.right_angle_trans.header.stamp =rospy.Time.now()
        # self.right_angle_trans.header.frame_id = "right_motor"
        # self.right_angle_trans.child_frame_id = "right_wheel"
        self.right_angle_trans.header.frame_id = "odom"
        self.right_angle_trans.child_frame_id = "right_wheel"
        
        self.left_angle_trans = TransformStamped()
        self.left_angle_trans.header.stamp = rospy.Time.now()
        # self.left_angle_trans.header.frame_id = "left_motor"
        # self.left_angle_trans.child_frame_id = "left_wheel"
        self.left_angle_trans.header.frame_id = "base_link"
        self.left_angle_trans.child_frame_id = "left_wheel"
        
    def pubodo(self):
        try:
            self.odocount += 1
            self.current_time = time.time()
            dt = self.current_time - self.last_time
            
            left_velocity = self.left_speed
            left_position = self.left_pos
            
            right_velocity = self.right_speed
            right_position = self.right_pos
            
            u = calc_input(right_velocity, left_velocity)
            old_theta = self.x[2]
            self.x = MotionModel(self.x, u, dt)
            self.odo.header.seq = self.odocount
            self.odo.header.stamp = rospy.Time.now()
            self.odo.pose.pose.position.x = self.x[0]
            self.odo.pose.pose.position.y = self.x[1]
            
            q = tf.transformations.quaternion_from_euler(0, 0, self.x[2])
            self.odo.pose.pose.orientation = Quaternion(*q)
            self.odo.twist.twist.linear.x = u[0]
            self.odo.twist.twist.angular.z = u[1]
            
            self.pub_odo.publish(self.odo)
            
            self.odom_trans.header.stamp = rospy.Time.now()
            self.odom_trans.transform.translation.x = self.x[0]
            self.odom_trans.transform.translation.y = self.x[1]
            self.odom_trans.transform.translation.z = 0
            self.odom_trans.transform.rotation = Quaternion(*q)
            self.tf_broadcaster.sendTransform(self.odom_trans)
            
            self.left_angle_trans.header.stamp = rospy.Time.now()
            self.left_angle_trans.transform.translation.x = 0
            self.left_angle_trans.transform.translation.y = 0
            self.left_angle_trans.transform.translation.z = 0
            q = tf.transformations.quaternion_from_euler(-left_position, 0, 0)
            self.left_angle_trans.transform.rotation = Quaternion(*q)
            self.tf_broadcaster.sendTransform(self.left_angle_trans)
            
            self.right_angle_trans.header.stamp = rospy.Time.now()
            self.right_angle_trans.transform.translation.x = 0
            self.right_angle_trans.transform.translation.y = 0
            self.right_angle_trans.transform.translation.z = 0
            q = tf.transformations.quaternion_from_euler(right_position, 0, 0)
            self.right_angle_trans.transform.rotation = Quaternion(*q)
            self.tf_broadcaster.sendTransform(self.right_angle_trans)
            
            self.last_time = time.time()
            
        except:
            import traceback
            traceback.print_exc()

def atler_odom_main():
    rospy.init_node("atler_odom")
    measurement = Measurement()
    atler_odom = ATLEROdom()
    rate = rospy.Rate(30)
    for i in range(100):
        measurement.right_angle = measurement.param_converter.right
        measurement.right_angle_rad = measurement.param_converter.right_angle_rad
        measurement.left_angle = measurement.param_converter.left
        measurement.left_angle_rad = measurement.param_converter.left_angle_rad
        measurement.measurement_main_no1()
        measurement.paramreset()
        atler_odom.right_speed = measurement.right_speed_rads
        atler_odom.right_pos = measurement.p_r
        atler_odom.left_speed = measurement.left_speed_rads
        atler_odom.left_pos = measurement.p_l
        # atler_odom.pubodo()
        # measurement.measurement_main_no1()
        # measurement.paramreset()
        # rate.sleep()
        rospy.sleep(0.01)
        
    while not rospy.is_shutdown():
        measurement.right_angle = measurement.param_converter.right
        measurement.right_angle_rad = measurement.param_converter.right_angle_rad
        measurement.left_angle = measurement.param_converter.left
        measurement.left_angle_rad = measurement.param_converter.left_angle_rad
        measurement.measurement_main_no1()
        atler_odom.right_speed = measurement.right_speed_rads
        atler_odom.right_pos = measurement.p_r
        atler_odom.left_speed = measurement.left_speed_rads
        atler_odom.left_pos = measurement.p_l
        atler_odom.pubodo()
        # measurement.measurement_main_no1()
        # rate.sleep()
        rospy.sleep(0.01)
        
if __name__ == '__main__':
    atler_odom_main()
    
