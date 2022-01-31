#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Made by Shimoda
#OpenCV 4.1.0 で動作確認

import rospy
import ros_numpy
import copy
import numpy as np
import sys
import cv2
import time
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from sensor_msgs.msg import Image

class PylonDetector:
    RED_PYLON_COLOR = (0, 150, 60), (5, 255, 225), (170, 100, 40), (179, 255, 225)
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
    MIN_HEIGHT = 30
    # 600mm先のz位置[mm]
    Z = 600

    def __init__(self, cam):
        rospy.init_node('pylon_detector')
        if cam == "g":
            self.image = GoProInput()
        elif cam == "s":
            self.image = ImageInput()
        else:
            print("Please specify the camera type as a command line argument \n GoPro :g, Sony RX0II: s")
        self.src = np.arange(27).reshape(3, 3, 3)
        self.is_pylon = False 
        self.half_ratio = 0.75
        self.half_ratio_p = 0.75
        self.convert_x_publish = rospy.Publisher("/data_x", Float32, queue_size=1)
        self.convert_y_publish = rospy.Publisher("/data_y", Float32, queue_size=1)
        self.z_publish = rospy.Publisher("/data_z", Float32, queue_size=1)
        self.signal_publish = rospy.Publisher("/signal", Int32, queue_size = 1)
        self.camera_publish = rospy.Publisher("/camera_image", Image, queue_size=10)
        self.camera_publish = rospy.Publisher("/camera_image", Image, queue_size=10)
        self.p_num_publish = rospy.Publisher("/p_num", Int8, queue_size=1)
        self.turning_publish = rospy.Publisher("/turning", Float32MultiArray, queue_size = 1)
        self.bi_publish = rospy.Publisher("/bi", Image, queue_size = 10)
        self.min_height = 25 
        self.cutoff = 100 #認識させない範囲（画面下部からのピクセル数）
        self.out = 0 #画面外のパイロンカウンタ
        self.main() #メインルーチン
        rospy.spin()

    def main(self): #メインルーチン
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            #輪郭抽出
            self.src = copy.copy(self.image.bgr_image) 
            self.outline_detection()
            #パイロンの情報を処理
            self.pylon_infomation()
            self.pub_image()
            rate.sleep()

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
        cv2.line(self.src, (0, 720-self.cutoff), (1280, 720 - self.cutoff), (0, 0, 255), thickness=1, lineType=cv2.LINE_8, shift=0)
        if len(self.countours) >= 1: 
            c_num = len(self.countours)
            #print(self.out, c_num)
            if c_num > 4:
                c_num = 4
            if self.out == 1 and c_num == 0:
                self.out == 0
            c_num += self.out
            #print(c_num)
            self.countours.sort(key=cv2.contourArea, reverse=True)
            cnt = self.countours[:c_num]
            if len(cnt) != c_num:
                c_num = len(cnt)
            print(len(cnt), c_num)
            top_center = [[] for i in range(c_num)] #GUIのためのデータ(矩形描写のためのxyz座標のピクセル値)を格納するリスト
            pylon_data = [[] for i in range(c_num)] #ロボット動作のためのデータ(パイロンの実環境下のxyz座標[mm]を格納するリスト)
            #輪郭のデータから物体がパイロンか否かを判別する
            for i in range(c_num):
                x,y,width,height = cv2.boundingRect(cnt[i])
                top_center[i] = [int(x) + round(0.5 * width), int(y) + round(0.5 * height), width, height]
                height = float(height)
                aspect_ratio = float(width / height)
                
                if (aspect_ratio > 0.9) and (aspect_ratio < 1.8): #ハーフ・パイロン(青枠)
                    if height <= self.min_height:
                        self.is_pylon = False
                        pylon_data[i] = [0, 0, 0, 2]
                        top_center[i].append(2)

                    else:
                        convert_x, convert_y, z, depth = self.calculate_depth(aspect_ratio, height, top_center[i][:2])

                        if x > 20 and x + width < 1260 and y + height < 720 - self.cutoff: #画面端のパイロンは計測の邪魔なので排除
                            pylon_data[i] = [convert_x, convert_y, z * self.half_ratio, 1]
                            #パイロンの情報(x, y, z, 種類判別ラベル (0: フルパイロン, 1: ハーフパイロン, 2: パイロンではない, 3:パイロンではあるが画面外))
                            top_center[i].append(1)

                        else:
                            self.is_pylon = False
                            pylon_data[i] = [0, 0, 0, 3]
                            top_center[i].append(3)


                elif (aspect_ratio > 0.5) and (aspect_ratio < 0.8): #フル・パイロン(緑枠)
                    if height <= self.min_height:
                        self.is_pylon = False
                        pylon_data[i] = [0, 0, 0, 2]
                        top_center[i].append(2)

                    else: 
                        convert_x, convert_y, z, depth = self.calculate_depth(aspect_ratio, height, top_center[i][:2])

                        if x > 20  and x + width < 1260 and y + height < 720 - self.cutoff: #画面端のパイロンは(ry     
                            pylon_data[i] = [convert_x, convert_y, z, 0]
                            top_center[i].append(0)

                        else:
                            self.is_pylon = False
                            pylon_data[i] = [0, 0, 0, 3]
                            top_center[i].append(3)

                else:
                    self.is_pylon = False
                    pylon_data[i] = [0, 0, 0, 2]
                    top_center[i].append(2)

            self.out = len([x for x in pylon_data if x[3] is 3])
            pylon_data = [x for x in pylon_data if x[3] is not 2 and x[3] is not 3] #パイロンではないもの(ラベルが2である物体)を排除

            for i in range(len(pylon_data)):
                pylon_data[i].append(i)    

            for i in range(len(top_center)):
                top_center[i].append(i)

            #top_center.sort(key = lambda x: x[2])
            top_center = [x for x in top_center if x[4] is not 2 and x[4] is not 3]
            #print(top_center)

            cross_color = (0, 0, 255) #画面に表示するバッテンの色
            if len(pylon_data) == 4: #画面上にパイロンが4つ存在する場合
                pylon_data.sort(key=lambda x: x[2]) #z方向の距離によって並べ替え
                pair_ave = [[] for i in range(3)]
                pair_ave_pix = [[] for i in range(3)]

                for i in range(2):
                    pair_ave[i] = [(pylon_data[i * 2][j] + pylon_data[i * 2 + 1][j]) / 2 for j in range(3)]
                    pair_ave_pix[i] = [(top_center[i * 2][j] + top_center[i * 2 + 1][j]) / 2 for j in range(3)]

                    cv2.drawMarker(self.src, (int((top_center[i*2][0] + top_center[i*2 + 1][0])/2), \
                        int((top_center[i * 2][1] + top_center[i * 2 + 1][1])/2)),\
                        cross_color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=40, thickness=3)
                
                pylon_data.sort(key=lambda x: x[4])
                top_center.sort(key=lambda x: x[5])
                for i in range(4):
                    if pylon_data[i][3] == 0: #緑枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (0, 109, 242), 2)
                    else: #青枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / self.half_ratio_p)), \
                            (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (255, 0, 0), 2)               
                        self.pub_turning(pylon_data[i][0], pylon_data[i][2]) #青枠パイロンのx, z方向距離をパブリッシュ

                self.pub_param(*pair_ave[0]) #手前のパイロンの組の情報をパブリッシュ
                self.p_num(4)

                

            elif len(pylon_data) == 3: #画面上にパイロンが3つ存在する場合
                pair_ave = []
                pair_ave_pix =[]
                d_0to1 = abs(pylon_data[0][2] - pylon_data[1][2])
                d_0to2 = abs(pylon_data[0][2] - pylon_data[2][2])
                d_1to2 = abs(pylon_data[1][2] - pylon_data[2][2])
                #３つのパイロンうち，そのZ軸方向の位置が近いものをペアとする．しきい値は1000[mm]．
                #そのためパイロンはZ軸方向に1000mm以上離して設置する必要あり．
                if d_0to1 < 700 and d_0to2 > 700 and d_1to2 > 700:
                    pair_ave =[(pylon_data[0][i] + pylon_data[1][i])/ 2 for i in range(3)]
                    pair_ave_pix =[(top_center[0][i] + top_center[1][i])/ 2 for i in range(3)]
                    
                elif d_0to2 < 700 and d_0to1 > 700 and d_1to2 > 700:
                    pair_ave =[(pylon_data[0][i] + pylon_data[2][i])/ 2 for i in range(3)]
                    pair_ave_pix =[(top_center[0][i] + top_center[2][i])/ 2 for i in range(3)]

                elif d_1to2 < 700 and d_0to1 > 700 and d_0to2 > 700:
                    pair_ave =[(pylon_data[1][i] + pylon_data[2][i])/ 2 for i in range(3)]
                    pair_ave_pix =[(top_center[1][i] + top_center[2][i])/ 2 for i in range(3)]
                
                else:
                    pair_ave_pix = [0,0,0]
                    pair_ave = [0,0,0]
                #print(pair_ave)
                for i in range(3):
                    if pylon_data[i][3] == 0: #緑枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (229, 77, 164), 2)
                    elif pylon_data[i][3] == 1: #青枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / self.half_ratio_p)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (255, 0, 0), 2)
                        print(pylon_data[i])
                        self.pub_turning(pylon_data[i][0], pylon_data[i][2])

                cv2.drawMarker(self.src, (int(pair_ave_pix[0]), int(pair_ave_pix[1])), cross_color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=40, thickness=3)
                self.pub_param(*pair_ave) #手前のパイロンの組の情報をパブリッシュ
                self.p_num(3)
                

            elif len(pylon_data) == 2: #画面上にパイロンが2つ存在する場合
                pylon_data.sort(key=lambda x: x[2]) #パイロンの位置関係に応じて並べ替え(手前:[0], 奥:[1])
                pair_ave =[(pylon_data[0][i] + pylon_data[1][i])/ 2 for i in range(3)]
                pylon_data.sort(key=lambda x: x[4])
                top_center.sort(key=lambda x: x[5])
                for i in range(2):
                    if pylon_data[i][3] == 0: #緑枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (0, 255, 0), 2)
                    else: #青枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / self.half_ratio_p)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (255, 0, 0), 2)
                        self.pub_turning(pylon_data[i][0], pylon_data[i][2])

                cv2.putText(self.src, "z="+str(pair_ave[2])+"[pixel]", (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (252, 104, 189), thickness=2)
                cv2.drawMarker(self.src, (int((top_center[0][0] + top_center[1][0])/2), int((top_center[0][1] + top_center[1][1])/2)),\
                        cross_color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=40, thickness=3)

                #print(x_mean)
                distance = abs(pylon_data[0][0] - pylon_data[1][0]) #パイロン間の距離によってロボットが左右どちらかに行き過ぎてないか判別
                if distance < 1000 and pylon_data[0][0] < pylon_data[1][0]: #ロボットが左を向きすぎている
                    pair_ave.append(1) #signal = 1 (左行き過ぎのシグナル)

                if distance < 1000 and pylon_data[0][0] > pylon_data[1][0]: #ロボットが右を向きすている
                    pair_ave.append(2) #signal = 2 (右行き過ぎのシグナル)

                self.pub_param(*pair_ave)    
                self.p_num(2)


            elif len(pylon_data) == 1: #画面上にパイロンが1つしかない場合
                if pylon_data[0][3] == 0: #緑枠
                    cv2.rectangle(self.src, (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), \
                            (top_center[0][0] + int(top_center[0][2] / 2), top_center[0][1] + int(top_center[0][3] / 2)), (0, 255, 0), 2)

                else: #青枠 
                    cv2.rectangle(self.src, (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / self.half_ratio_p)), \
                            (top_center[0][0] + int(top_center[0][2] / 2), top_center[0][1] + int(top_center[0][3] / 2)), (255, 0, 0), 2)
                    self.pub_turning(pylon_data[0][0], pylon_data[0][2])
                cv2.drawMarker(self.src, (top_center[0][0], top_center[0][1]),\
                    cross_color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=40, thickness=3)
                
                if pylon_data[0][0] < -100: #行き過ぎシグナル(調整中)
                    pylon_data[0][3] = 1 

                if pylon_data[0][0] > 100:
                    pylon_data[0][3] = 2

                self.pub_param(*pylon_data[0][:3])
                self.p_num(1)

            elif len(pylon_data) == 0:
                self.pub_param(0, 0, 0)
                cv2.drawMarker(self.src, (int(self.image.bgr_image.shape[1]/2), int(self.image.bgr_image.shape[0]/2)),\
                    cross_color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=40, thickness = 3)
                print("nothing")
                self.p_num(0)
                                
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

    def pub_param(self, convert_x, convert_y, Z, signal = 0):
        self.convert_x_publish.publish(convert_x)
        self.convert_y_publish.publish(convert_y)
        self.z_publish.publish(Z)
        self.signal_publish.publish(signal)
    
    def pub_turning(self,x,z):
        array = Float32MultiArray(data=[x,z])
        self.turning_publish.publish(array)
    
    def pub_image(self):
        self.camera_publish.publish(ros_numpy.msgify(Image, self.src, encoding='rgb8'))

    def p_num(self, n):
        self.p_num_publish.publish(n)

class GoProInput:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/gopro_image", Image, self.image_callback)
        self.bgr_image = np.arange(27).reshape(3, 3, 3)
        self.callbacked = False
        
    def image_callback(self, msg):
        self.callbacked = True #画像が届いたかどうか判定
        self.bgr_image = ros_numpy.numpify(msg)

class ImageInput:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.bgr_image = np.arange(27).reshape(3, 3, 3)
        self.callbacked = False
    def image_callback(self, image_msg):
        self.bgr_image = ros_numpy.numpify(msg) 


if __name__ == '__main__':
    main = PylonDetector(sys.argv[1])
