#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Made by Shimoda
#OpenCV 4.1.0 で動作確認

import rospy
import numpy as np
import sys
import cv2
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageInput:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/gopro_image", Image, self.image_callback)
        self.bgr_image = np.arange(27).reshape(3, 3, 3)
        self.callbacked = False
        
    def image_callback(self, image_data):
        self.callbacked = True #画像が届いたかどうか判定
        try:
            self.bgr_image = self.bridge.imgmsg_to_cv2(image_data)
        except CvBridgeError as e:
            rospy.logerr(e)

class PylonDetector:
    RED_PYLON_COLOR = (0, 100, 40), (5, 255, 225), (170, 100, 40), (179, 255, 225)
    # ガウシアンカーネルサイズ
    GAUSSIAN_KERNEL_SIZE = (3, 3)
    # モルフォロジー演算カーネルサイズ
    MORPHOLOGY_KERNEL_SIZE = np.ones((5,5), np.uint8)
    # パイロン〜カメラ間の実際の測定距離[mm]
    BASE_DISTANCE = 600
    # BASE_DISTANCEでのパイロンの高さ[pixel]
    BASE_HEIGHT = 264
    # パイロンの高さ[mm]
    PYLON_HEIGHT = 165
    # 画面の中央(横方向)[pixel]
    IMAGE_CENTER_X = 640
    # 画面の中央(高さ方向)[pixel]
    IMAGE_CENTER_Y = 360
    # パイロンの最小高さ
    MIN_HEIGHT = 50
    # 600mm先のz位置[mm]
    Z = 600


    def __init__(self):
        rospy.init_node('pylon_detector')
        self.image = ImageInput()
        self.bridge = CvBridge()
        self.is_pylon = False 
        self.convert_x_publish = rospy.Publisher("/data_x", Float32, queue_size=1)
        self.convert_y_publish = rospy.Publisher("/data_y", Float32, queue_size=1)
        self.z_publish = rospy.Publisher("/data_z", Float32, queue_size=1)
        self.camera_publish = rospy.Publisher("/camera_image", Image, queue_size=10)
        self.min_height = 50
        self.main()
        rospy.spin()


    def outline_detection(self):
        #print(self.image.bgr_image.shape)
        hsv_image = cv2.cvtColor(self.image.bgr_image.astype(np.uint8), cv2.COLOR_BGR2HSV)
        # ガウシアンフィルタをかける
        gaussian_filter_image = cv2.GaussianBlur(hsv_image, self.GAUSSIAN_KERNEL_SIZE, 0)
        # 二値化
        binarization_image_1 = cv2.inRange(gaussian_filter_image, self.RED_PYLON_COLOR[0], self.RED_PYLON_COLOR[1])
        binarization_image_1 = cv2.inRange(gaussian_filter_image, self.RED_PYLON_COLOR[0], self.RED_PYLON_COLOR[1])
        binarization_image_2 = cv2.inRange(gaussian_filter_image, self.RED_PYLON_COLOR[2], self.RED_PYLON_COLOR[3])
        binarization_image = cv2.add(binarization_image_1, binarization_image_2)
        # クロージング(膨張、収縮の順に行う演算)
        morphology_close_image = cv2.morphologyEx(binarization_image, cv2.MORPH_CLOSE, self.MORPHOLOGY_KERNEL_SIZE)
        #cv2.imshow('mask', morphology_close_image)
        bi = cv2.cvtColor(morphology_close_image, cv2.COLOR_GRAY2BGR)
        #self.bi_publish.publish(self.bridge.cv2_to_imgmsg(bi,'bgr8' ))
        # 輪郭検出
        self.countours, hierarchy = cv2.findContours(morphology_close_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)     

    def pylon_infomation(self):
        if len(self.countours) > 1 and self.image.callbacked:
            self.countours.sort(key=cv2.contourArea, reverse=True)
            cnt = self.countours[:2]
            top_center = [[],[]]
            pylon_data = [[],[]]

            for i in range(2):
                x,y,width,height = cv2.boundingRect(cnt[i])
                top_center[i] = [int(x) + round(0.5 * width), int(y) + round(0.5 * height), width, height]
                height = float(height)
                aspect_ratio = float(width / height)
                
                if (aspect_ratio > 0.95) and (aspect_ratio < 1.8): #ハーフ・パイロン(青枠)
                    if height <= self.min_height:
                        self.is_pylon = False
                        pylon_data[i] = [0, 0, 0, 2]

                    else:
                        convert_x, convert_y, z, depth = self.calculate_depth(aspect_ratio, height, top_center[i][:2])

                        if (x > 20) and (x + width < 1260): #画面端のパイロンは計測の邪魔なので排除
                            #cv2.rectangle(self.image.bgr_image, (x, y-int(height*1.1)), (x+int(width), y+int(height)), (255, 0, 0), 2)
                            #cv2.putText(self.image.bgr_image, "z="+str(round(z[i]*0.485))+"[mm]", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (189,104, 30), thickness=2)
                            pylon_data[i] = [convert_x, convert_y, z, 1] #パイロンの情報(x, y, z, 種類判別 (0: フルパイロン, 1: ハーフパイロン, 2: パイロンではない))

                        else:
                            self.is_pylon = False
                            pylon_data[i] = [0, 0, 0, 2]


                elif (aspect_ratio > 0.5) and (aspect_ratio < 0.95): #フル・パイロン(緑枠)
                    if height <= self.min_height:
                        self.is_pylon = False
                        pylon_data[i] = [0, 0, 0, 2]

                    else: 
                        convert_x, convert_y, z, depth = self.calculate_depth(aspect_ratio, height, top_center[i][:2])

                        if (x > 20) and (x + width < 1260): #画面端のパイロンは(ry
                            #cv2.rectangle(self.image.bgr_image, (x, y), (x+int(width), y+int(height)), (0, 255, 0), 2)
                            #cv2.putText(self.image.bgr_image, "z="+str(z[i])+"[mm]", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (189,104, 30), thickness=2) 
                            #cv2.putText(self.image.bgr_image, "pylonheight="+str(height)+"[pixel]", (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (252, 104, 189), thickness=2)        
                            pylon_data[i] = [convert_x, convert_y, z, 0]

                        else:
                            self.is_pylon = False
                            pylon_data[i] = [0, 0, 0, 2]

                else:
                    self.is_pylon = False
                    pylon_data[i] = [0, 0, 0, 2]


            if pylon_data[0][3] is not 2 and pylon_data[1][3] is not 2: #画面上にパイロンが2つ存在する場合
                pylon_data.sort(key=lambda x: x[0]) #パイロンの位置関係に応じて並べ替え(左:[0], 右:[1])
                top_center.sort(key=lambda x: x[0]) 
                print(pylon_data)

                x_ave = (pylon_data[0][0] + pylon_data[1][0]) / 2 #2つのパイロンの中央のx座標の値
                y_ave = (pylon_data[0][1] + pylon_data[1][1]) / 2 #いちおうy座標の中央の値
                z_ave = (pylon_data[0][2] + pylon_data[1][2]) / 2 #z座標の中央の値

                for i in range(2):
                    if pylon_data[i][3] == 0: #緑枠描画
                        cv2.rectangle(self.image.bgr_image, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (0, 255, 0), 2)
                    else: #青枠描画
                        cv2.rectangle(self.image.bgr_image, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (255, 0, 0), 2)

                cv2.putText(self.image.bgr_image, "z="+str(z_ave)+"[pixel]", (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (252, 104, 189), thickness=2)
                cv2.drawMarker(self.image.bgr_image, (int((top_center[0][0] + top_center[1][0])/2), int((top_center[0][1] + top_center[1][1])/2)),\
                        (0, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=20)

                #print(x_mean)
                self.pub_param(x_ave, y_ave, z_ave)    


            elif pylon_data[0][3] is not 2 and  pylon_data[1][3] is 2: #画面上にパイロンが1つしかない場合
                print(pylon_data)
                self.pub_param(pylon_data[0][0], pylon_data[0][1], pylon_data[0][2])
                if pylon_data[0][3] == 0: #緑枠
                    cv2.rectangle(self.image.bgr_image, (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), \
                            (top_center[0][0] + int(top_center[0][2] / 2), top_center[0][1] + int(top_center[0][3] / 2)), (0, 255, 0), 2)

                else: #青枠 
                    cv2.rectangle(self.image.bgr_image, (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), \
                            (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), (255, 0, 0), 2)

                cv2.drawMarker(self.image.bgr_image, (top_center[0][0], top_center[0][1]),\
                    (0, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=20)
                print("There is a pylon A")


            elif pylon_data[0][3] is 2 and  pylon_data[1][3] is not 2: #画面上にパイロンが1つしかない場合
                pylon_data.sort(key = lambda x:x[3])
                print(pylon_data)
                self.pub_param(pylon_data[0][0], pylon_data[0][1], pylon_data[0][2])
                if pylon_data[0][3] == 0: #緑枠
                    cv2.rectangle(self.image.bgr_image, (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), \
                            (top_center[0][0] + int(top_center[0][2] / 2), top_center[0][1] + int(top_center[0][3] / 2)), (0, 255, 0), 2)

                else: #青枠 
                    cv2.rectangle(self.image.bgr_image, (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), \
                            (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), (255, 0, 0), 2)

                cv2.drawMarker(self.image.bgr_image, (top_center[0][0], top_center[0][1]),\
                    (0, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=20)
                print("There is a pylon B")


            elif pylon_data[0][3] == 2 and pylon_data[1][3] == 2:
                self.pub_param(0, 0, 0)
                cv2.drawMarker(self.image.bgr_image, (int(self.image.bgr_image.shape[1]/2), int(self.image.bgr_image.shape[0]/2)),\
                    (0, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=20)
                print("nothing")
                                
        else:
            self.is_pylon = False
            self.pub_param(0, 0, 0)


    def calculate_depth(self, aspect_ratio, height, top_center):
        # 奥行き方向の計算
        depth = (self.BASE_HEIGHT / float(height)) * self.BASE_DISTANCE
        # 1ピクセル当たりの大きさを計算
        one_pixel_size = self.PYLON_HEIGHT / float(height)
        # 三次元位置(X, Y, Z)の計算
        convert_x = (top_center[0] - self.IMAGE_CENTER_X) * one_pixel_size
        convert_y = (self.IMAGE_CENTER_Y - top_center[1]) * one_pixel_size
        if depth > convert_x:
            Z = math.sqrt(depth * depth - convert_x * convert_x)
        convert_x, convert_y, Z, depth = round(convert_x), round(convert_y), round(Z), round(depth)
        return convert_x, convert_y, Z, depth


    def pub_param(self, convert_x, convert_y, Z):
        self.convert_x_publish.publish(convert_x)
        self.convert_y_publish.publish(convert_y)
        self.z_publish.publish(Z)

    
    def pub_image(self):
        self.camera_publish.publish(self.bridge.cv2_to_imgmsg(self.image.bgr_image.astype(np.uint8), 'bgr8'))


    def main(self): #メインルーチン
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            #輪郭抽出
            self.outline_detection()
            #パイロンの情報を処理
            self.pylon_infomation()
            self.pub_image()
            rate.sleep()


if __name__ == '__main__':
    pylon_detection = PylonDetector()
