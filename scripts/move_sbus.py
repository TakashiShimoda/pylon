#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
import numpy as np
from std_msgs.msg import Int32MultiArray
"""
SBUSについて
ステアリングの値の範囲を以下に示す

左                N                 右 
1680             1024                368
 |----------------|------------------|
 　　　 656                 656
 
スロットルについても同様(1024で停止，それ以上は前進，以下は後進)
"""
class Main: 
    def __init__(self):
        self.datainput = Datainput()
        rospy.init_node('move_robot')
        self.pub_vel = rospy.Publisher('/sbus_cmd',Int32MultiArray, queue_size=10)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(15)
        self.steering = Int32MultiArray()
        self.throttle_fast = 1155 
        self.throttle_turn = 1024 #ターンするときの直進方向のスピード
        self.until_turn = 1200 #青パイロンの前に停止するまでにどこまで近づくか
        self.steering_rate = 0.055 #フルスロットルは1, だが危険なので使用禁止!!
        self.is_wind_up = False
        self.e = 0
        self.pid = PID(kp = 0.08, ki = 0.0465, kd = 0) #PID制御のパラメータ D制御は扱いが難しいので0が良い
        #わりと優秀だった値(usbカメラにて) kp = 0.156, ki = 0.0565, kd = 0
        #steering_rate = 0.025, kp = 0.15, ki = 0.03, kd = 0 (GoPro)
        self.main_loop()
        rospy.spin()

    def main_loop(self):
        while not rospy.is_shutdown():
            self.move_straight()
            rospy.sleep(1)
            #self.turn()
            #self.datainput.data_turn[1] = 10000
            rospy.sleep(1)

    def move_straight(self):
        while not rospy.is_shutdown() and self.datainput.data_turn[1] > self.until_turn:
            angular = self.angular_pid()
            #print(self.datainput.data_turn[1]) 
            #print(self.datainput.data_signal) 
            self.steering.data = [angular, self.throttle_fast]
            self.pub_vel.publish(self.steering)
            self.rate.sleep()

    def turn(self):
        if self.datainput.data_turn[0] < 0: #パイロンが車体中心より左にあるため右旋回
            print("Turning Right")
            while self.datainput.data_p < 3:
                self.steering.data = [974, self.throttle_turn]
                self.pub_vel.publish(self.steering)
                self.rate.sleep()
        elif self.datainput.data_turn[0] > 0:
            print("Turning Left")
            while self.datainput.data_p < 3:
                self.steering.data = [1074, self.throttle_turn]
                self.pub_vel.publish(self.steering)
                self.rate.sleep()
        elif self.datainput.data_turn[0] == 0:
            self.move_straight()

    """
    #直進行軍(PID制御バージョン)
    def move_straight(self):
        while not rospy.is_shutdown():
            diff = math.atan2(self.datainput.data_x, self.datainput.data_z)
            #print(diff)
            m = self.pid.calc(diff, 0, self.is_wind_up)
            #print(self.pid.ei)
            angular = 1024 + int(m * 656 * self.steering_rate)
            #リミッター
            if angular > 1500:
                if np.sign(self.pid.e) == np.sign(m):
                   self.is_wind_up = True
                   
                else:
                   self.is_wind_up = False
            elif angular < 548:
                if np.sign(self.pid.e) == np.sign(m):
                    self.is_wind_up = True
                else:
                    self.is_wind_up = False
            else:
                self.is_wind_up = False
            
            if angular >1500:
                angular = 1500

            if angular < 548:
                angular = 548

            print(angular)
            self.steering.data = [angular, self.throttle_fast]
            self.pub_vel.publish(self.steering)
            rate.sleep()
   """

    def angular_pid(self):
        self.e = self.datainput.data_x
        if self.e > 100:
            angular = 1024 - int(656 * self.steering_rate) 
            if self.datainput.data_signal == 1:
                angular = angular + 50
            if self.datainput.data_signal == 2:
                angular = angular - 50

        if self.e < -100:
            angular = 1024 + int(656 * self.steering_rate) 
            if self.datainput.data_signal == 2:
                angular = angular - 50
            if self.datainput.data_signal == 1:
                angular = angular + 50

        if self.e <= 100 and self.e >= -100:
            angular = 1024 

        print(angular)
        return angular

    def move_roll_right(self):
        data = self.center_calculate()
        while  data[0] < 150 and not rospy.is_shutdown():
                data = self.center_calculate()
                #twist.linear.x = 1.6
                #twist.angular.z = -1.6
                self.steering.data = [1024 - int(656 * self.steering_rate), self.throttle_fast]
                rate.sleep()
        while  data[0] > 70 and not  rospy.is_shutdown():
                data = self.center_calculate()
                #twist.linear.x = 1.5
                #twist.angular.z = -2.3
                self.steering.data = [1024 - int(656 * self.steering_rate * 1.2), self.throttle_fast]
                self.pub_vel.publish(self.steering)
                rate.sleep()
            
        self.steering.data = [1024, 1024]
        self.pub_vel.publish(self.steering)

    def move_roll_left(self):
        data = self.center_calculate()
        while  data[0] > -150  and not rospy.is_shutdown():
                data = self.center_calculate()
                #twist.linear.x = 1.6
                #twist.angular.z = -1.6
                self.steering.data = [1024 + int(656 * self.steering_rate), self.throttle_fast]
                rate.sleep()
        while  data[0] < -70 and not  rospy.is_shutdown():
                data = self.center_calculate()
                self.steering.data = [1024 + int(656 * self.steering_rate * 1.2), self.throttle_fast]
                self.pub_vel.publish(self.steering)
                rate.sleep()

        self.steering.data = [1024, 1024]
        self.pub_vel.publish(self.steering)

    def center_calculate(self):
        main.x_data = main.datainput.data_x
        main.y_data = main.datainput.data_y
        center_x = self.x_data
        center_y = self.y_data   
        print(center_x)
        return center_x, center_y 
 

    def shutdown(self):
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_vel.publish(twist)
        """
        self.steering.data = [1024, 1024]
        self.pub_vel.publish(self.steering)
        print("shutdown")


class Datainput:
    def __init__(self):
        self.sub_x = rospy.Subscriber('/data_x', Float32, self.callback1)
        self.sub_y = rospy.Subscriber('/data_y', Float32, self.callback2)
        self.sub_z = rospy.Subscriber('/data_z', Float32, self.callback3)
        self.sub_h = rospy.Subscriber('/data_h', Float32, self.callback4)
        self.sub_w = rospy.Subscriber('/data_witdh', Float32, self.callback6)
        self.sub_c = rospy.Subscriber('/data_color', Int8, self.callback7)
        self.sub_signal = rospy.Subscriber('signal', Int32, self.callback8)
        self.sub_turn = rospy.Subscriber('/turning', Float32MultiArray, self.callback9)
        self.data_x = 0
        self.data_y = 0
        self.data_z = 0
        self.data_h = 0
        self.data_w = 0
        self.data_c = 0
        self.data_signal = 0
        self.data_turn = [0,10000] 
        self.data_p= 0
        self.sub_signal = rospy.Subscriber('p_num', Int8, self.callback10)

    def callback1(self,messeage):
        self.data_x =  messeage.data
    def callback2(self,messeage):
        self.data_y =  messeage.data
    def callback3(self,messeage):
        self.data_z =  messeage.data
    def callback4(self,messeage):
        self.data_h =  messeage.data
    def callback6(self,messeage):
        self.data_w = messeage.data
    def callback7(self, message):
        self.data_c = message.data
    def callback8(self, message):
        self.data_signal = message.data
    def callback9(self, message):
        self.data_turn[0] = message.data[0]
        self.data_turn[1] = message.data[1]
    def callback10(self, message):
        self.data_p = message.data

class PID:
    def __init__(self, kp, ki, kd):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.m = 0
        self.e = 0
        self.ei = 0 #積分器用偏差 
        self.e1 = 0
        self.e2 = 0


    def calc(self, y, r, is_wind_up): #(制御対象の出力，目標量, 制御量がwind upしたか)
        self.e2 = self.e1 #値の更新
        self.e1 = self.e
        self.e = r - y
        self.ei = r - y 
        if is_wind_up:
            self.ei = 0
        d_m = self.kp * (self.e - self.e1) + self.ei * self.ki + self.kd * (self.e - 2 * self.e1 + self.e2)
        self.m = self.m + d_m
        
        return self.m

if __name__ == '__main__': 
    main = Main()
