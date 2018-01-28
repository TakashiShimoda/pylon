#!usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import time

class Main: 
    
    def __init__(self):
        self.datainput = Datainput()
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def mainsystem(self):
        rospy.sleep(3)
        """self.marking_point_setting()
        rospy.sleep(0.01)"""
        self.marking()                            
        self.determining_pylon()
        rospy.sleep(1)
        """self.marking_point_setting()
        rospy.sleep(0.01)
        print("setting_OK") """
        self.marking()                 
        self.determining_pylon()
        rospy.sleep(1)
        """self.marking_point_setting()
        rospy.sleep(0.01)"""
        self.marking()                  
        self.determining_pylon()
        rospy.sleep(1)
        """self.marking_point_setting()
        rospy.sleep(0.01)   """
        self.marking()                 
        self.determining_pylon()
        rospy.sleep(1)
        """self.marking_point_setting()
        rospy.sleep(0.01) """
        self.marking()          
        self.determining_pylon()
        rospy.sleep(1)

    def determining_pylon(self):
        self.z_data = 0
        main.z_data = main.datainput.data_z
        self.flag = rospy.get_param("/pylon_color", None)
        if self.flag == True : 
            print("pylon2")
            self.move_straight()
            print("straight")
            rospy.sleep(0.01)
            self.move_roll_left()
            print("roll_left")
        if self.flag == False :
            print("pylon1")
            self.move_straight()
            print("straight")
            rospy.sleep(0.01)
            self.move_roll_right()
            print("roll_right")
        if self.flag == None:
            print(None)
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.pub_vel.publish(twist)
    

    #def marking_point_setting(self):
        #twist = Twist()
        #while not data[0] = 0 and not rospy.is_shutdown():
        #    data = self.center_calculate()
        #    if data[0] < 0:
        #       self.angular = -3
        #    if data[0] > 0:
        #        self.angular = 3
        #    twist.angular.z = self.angular
        #    self.pub_vel.publish(twist)
        #    rate.sleep()
        #self.angular = 0
        #twist.angular.z = self.angular
        #self.pub_vel.publish(twist)
 




    def move_straight(self):
        main.z_data = main.datainput.data_z
        twist = Twist()
        while not  self.z_data < 350 and not rospy.is_shutdown():
            main.z_data = main.datainput.data_z
            print(self.z_data)
            twist.linear.x = 1.6
            #if self.z_data > 500 :
            #    angular = self.angular_1()
            #else:  
            angular = self.angular_pid()
            twist.angular.z = angular
            self.pub_vel.publish(twist)
            rate.sleep()
            

    def angular_1(self):
        data = self.center_calculate()
        self.e = 0 - data[0]
        if self.e > 60 :
            angular = 1.5
            return angular
        if self.e < -60 :
            angular = -1.5
            return angular
        else:
            angular = 0
            return angular

     
    def angular_pid(self):
        data = self.center_calculate()
        self.e = 0 - data[0]
        if self.e > 1 :
            angular = 1.5
            return angular
        if self.e < -1 :
            angular = -1.5
            return angular
        if self.e <= 1 and self.e >= -1 :
            angular = 0
            return angular


    def move_roll_right(self):
        twist = Twist()
        data = self.center_calculate()
        while  data[0] < 150 and not rospy.is_shutdown():
                data = self.center_calculate()
                twist.linear.x = 1.6
                twist.angular.z = -1.6
                self.pub_vel.publish(twist)
                rate.sleep()
        while  data[0] > 20 and not  rospy.is_shutdown():
                data = self.center_calculate()
                twist.linear.x = 1.5
                twist.angular.z = -2.3
                self.pub_vel.publish(twist)
                rate.sleep()
            
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_vel.publish(twist)

    def move_roll_left(self):
        twist = Twist()
        data = self.center_calculate()
        while  data[0] > -150  and not rospy.is_shutdown():
                data = self.center_calculate()
                twist.linear.x = 1.8
                twist.angular.z = 1.7
                self.pub_vel.publish(twist)
                rate.sleep()
        while  data[0] < -20 and not  rospy.is_shutdown():
                data = self.center_calculate()
                twist.linear.x = 1.5
                twist.angular.z = 2.5
                self.pub_vel.publish(twist)
                rate.sleep()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_vel.publish(twist)

    def marking(self):
        self.marking_flag = True
        rospy.set_param("/move_robot_ver1/marking_flag", self.marking_flag)


    def center_calculate(self):
        main.x_data = main.datainput.data_x
        main.y_data = main.datainput.data_y
        main.h_data = main.datainput.data_h
        # main.depth_data = main.datainput.data_depth
        main.w_data = main.datainput.data_w

        center_x = self.x_data + self.w_data/2
        center_y = self.y_data - self.h_data/2   
        print(center_x)
        return center_x, center_y 

 

    def shutdown(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_vel.publish(twist)
        print("shutdown")

class Datainput:
    def __init__(self):
        self.sub_x = rospy.Subscriber('/data_x', Float32, self.callback1)
        self.sub_y = rospy.Subscriber('/data_y', Float32, self.callback2)
        self.sub_z = rospy.Subscriber('/data_z', Float32, self.callback3)
        self.sub_h = rospy.Subscriber('/data_h', Float32, self.callback4)
        self.sub_w = rospy.Subscriber('/data_witdh', Float32, self.callback6)
        #self.sub_depth = rospy.Subscriber('/data_depth',Float32,self.callback5)


        self.data_x = 0
        self.data_y = 0
        self.data_z = 0
        self.data_h = 0
        #self.data_depth = 0
        self.data_w = 0
     

    def callback1(self,messeage):
        self.data_x =  messeage.data
    def callback2(self,messeage):
        self.data_y =  messeage.data
    def callback3(self,messeage):
        self.data_z =  messeage.data
    def callback4(self,messeage):
        self.data_h =  messeage.data
    #def callback5(self,messeage):
    #    self.data_depth =  messeage.data
    def callback6(self,messeage):
        self.data_w = messeage.data

if __name__ == '__main__': 
    rospy.init_node('move_robot')
    main = Main()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        main.x_data = main.datainput.data_x
        main.y_data = main.datainput.data_y
        main.z_data = main.datainput.data_z
        main.h_data = main.datainput.data_h
        # main.depth_data = main.datainput.data_depth
        main.w_data = main.datainput.data_w

        main.mainsystem()
        rospy.sleep(0.1)
        rate.sleep()
