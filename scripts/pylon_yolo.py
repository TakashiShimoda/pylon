#!/usr/bin/env python3 
# -*- coding: utf-8 -*- 
#Made by Shimoda in January, 2022
#OpenCV 4.1.0 で動作確認
#GoProからの映像を用いる場合, gopropub2.pyを用いること

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
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

class PylonDetector:
    RED_PYLON_COLOR = (0, 150, 60), (5, 255, 225), (170, 100, 40), (179, 255, 225)
    # ガウシアンカーネルサイズ
    GAUSSIAN_KERNEL_SIZE = (3, 3)
    # モルフォロジー演算カーネルサイズ
    MORPHOLOGY_KERNEL_SIZE = np.ones((5,5), np.uint8)
    # パイロン〜カメラ間の実際の測定距離[mm]
    BASE_DISTANCE = 600
    # BASE_DISTANCEでのパイロンの高さ[pixel]
    BASE_HEIGHT = 264/3
    # パイロンの高さ[mm]
    PYLON_HEIGHT = 165
    # 画面の中央(横方向)[pixel] 
    IMAGE_CENTER_X = 214 # 画面の中央(高さ方向)[pixel
    IMAGE_CENTER_Y = 120
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
        self.reso = (428, 240) # 画面の解像度
        self.convert_x_publish = rospy.Publisher("/data_x", Float32, queue_size=1)
        self.convert_y_publish = rospy.Publisher("/data_y", Float32, queue_size=1)
        self.z_publish = rospy.Publisher("/data_z", Float32, queue_size=1)
        self.signal_publish = rospy.Publisher("/signal", Int32, queue_size = 1)
        self.camera_publish = rospy.Publisher("/camera_image", Image, queue_size=10)
        self.p_num_publish = rospy.Publisher("/p_num", Int8, queue_size=1)
        self.turning_publish = rospy.Publisher("/turning", Float32MultiArray, queue_size = 1)
        self.bi_publish = rospy.Publisher("/bi", Image, queue_size = 10)
        self.min_height = 25 
        self.bdboxes = DataInput()
        self.out = 0 #画面外のパイロンカウンタ
        self.main() #メインルーチン
        rospy.spin()

    def main(self): #メインルーチン
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.src = copy.copy(self.image.bgr_image) 
            self.pylon_infomation()
            self.pub_image()
            rate.sleep()

    def pylon_infomation(self, boxes=[], rects=[]):
        cutoff = round(self.reso[1]*0.14)
        cv2.line(self.src, (0, self.reso[1]-cutoff), (self.reso[0], self.reso[1] - cutoff), \
                (0, 0, 255), thickness=1, lineType=cv2.LINE_8, shift=0)
        boxes = copy.deepcopy(self.bdboxes.pylons)
        rects = copy.deepcopy(self.bdboxes.rect)
        print(self.bdboxes.pylons)
        if len(boxes) >= 1 and self.bdboxes.is_callbacked:
            #print(self.boundbx.pylons,"self.boundbx.pylons")
            c_num = len(boxes)
            #c_num = len(self.countours)
            #print(self.out, c_num)
            if c_num > 4:
                c_num = 4
            if self.out == 1 and c_num == 0:
                self.out == 0
            c_num += self.out
            #print(c_num)
            #バウンディングボックスの大きさによって並べ替え
            boxes.sort(key=lambda x: x[2]*x[3], reverse=True)
            rects.sort(key=lambda x:x[2]*x[3], reverse=True)

            cnt = boxes[:c_num]
            if len(cnt) != c_num:
                c_num = len(cnt)

            pylon_data = [[] for i in range(c_num)] #ロボット動作のためのデータ(パイロンの実環境下のxyz座標[mm]を格納するリスト)
            top_center = [[] for i in range(c_num)]
            print(top_center, "top")

            for i in range(c_num):
                x, y, width, height = rects[i][:4]
                top_center[i] = boxes[i]
                width = top_center[i][2]
                height = top_center[i][3]
                aspect_ratio = float(width / height)
                
                if (aspect_ratio > 0.8) and (aspect_ratio < 1.8): #ハーフ・パイロン(青枠)
                            #パイロンの情報(x, y, z, 種類判別ラベル (0: フルパイロン, 1: ハーフパイロン,\
                            # 2: パイロンではない, 3:パイロンではあるが画面外))
                        if x < 20: #画面端のパイロンは計測の邪魔なので排除
                            self.is_pylon = False
                            pylon_data[i] = [0,0,0,3]
                            top_center[i].append(3)

                        elif x + width > self.reso[0] -20:
                            self.is_pylon = False
                            pylon_data[i] = [0,0,0,3]
                            top_center[i].append(3)
                       
                        elif y + height > self.reso[1] - cutoff: 
                            self.is_pylon = False
                            pylon_data[i] = [0, 0, 0, 3]
                            top_center[i].append(3)
 
                        else:
                            convert_x, convert_y, z, depth = self.calculate_depth(aspect_ratio, height, top_center[i][:2])
                            pylon_data[i] = [convert_x, convert_y, z, 1]
                            top_center[i].append("1023")


                elif (aspect_ratio > 0.5) and (aspect_ratio < 0.8): #フル・パイロン(緑枠)
                        if x < 20: #画面端のパイロンは計測の邪魔なので排除
                            self.is_pylon = False
                            pylon_data[i] = [0,0,0,3]
                            top_center[i].append(3)

                        elif x + width > self.reso[0] -20:
                            self.is_pylon = False
                            pylon_data[i] = [0,0,0,3]
                            top_center[i].append(3)

                        elif y + height > self.reso[1] - cutoff: 
                            self.is_pylon = False
                            pylon_data[i] = [0, 0, 0, 3]
                            top_center[i].append(3)
                        
                        else:
                            convert_x, convert_y, z, depth = self.calculate_depth(aspect_ratio, height, top_center[i][:2])
                            pylon_data[i] = [convert_x, convert_y, z, 0]
                            top_center[i].append(1024)

                else:
                    self.is_pylon = False
                    pylon_data[i] = [0, 0, 0, 2]
                    top_center[i].append(2)
            
            if len(top_center[i]) > 5:
                top_center[i] = top_center[i][:4]

            #print(top_center, "top_center")
            #print(pylon_data, "pylon_data")
            self.out = len([x for x in pylon_data if x[3] == 3])
            pylon_data = [x for x in pylon_data if x[3] != 3] #パイロンではないもの(ラベルが2である物体)を排除

            for i in range(len(pylon_data)):
                pylon_data[i].append(i)    

            for i in range(len(top_center)):
                top_center[i].append(i)

            #top_center.sort(key = lambda x: x[2])
            top_center = [x for x in top_center if x[4] != 3]

            if len(pylon_data) == 4: #画面上にパイロンが4つ存在する場合
                pylon_data.sort(key=lambda x: x[2]) #z方向の距離によって並べ替え
                pair_ave = [[] for i in range(3)]
                pair_ave_pix = [[] for i in range(3)]

                for i in range(2):
                    pair_ave[i] = [(pylon_data[i * 2][j] + pylon_data[i * 2 + 1][j]) / 2 for j in range(3)]
                    pair_ave_pix[i] = [(top_center[i * 2][j] + top_center[i * 2 + 1][j]) / 2 for j in range(3)]
                    self.marker(int((top_center[i*2][0] + top_center[i*2 + 1][0])/2), int((top_center[i * 2][1] + top_center[i * 2 + 1][1])/2))
                
                pylon_data.sort(key=lambda x: x[4])
                top_center.sort(key=lambda x: x[5])
                for i in range(4):
                    if pylon_data[i][3] == 0 or 1: #緑枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (0, 109, 242), 2)
                        self.pub_turning(pylon_data[i][0], pylon_data[i][2]) #青枠パイロンのx, z方向距離をパブリッシュ

                text ="z="+str(pair_ave[0][2])+"[mm]"
                self.text("z="+str(pair_ave[2])+"[mm]","x="+str(pair_ave[0])+"[mm]" )
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
                    pair_ave_pix = [round(self.reso[0]/2),round(self.reso[1]/2),0]
                    pair_ave = [0,0,0]
                #print(pair_ave)
                for i in range(3):
                    if pylon_data[i][3] == 0 or 1: #緑枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (229, 77, 164), 2)
                        self.pub_turning(pylon_data[i][0], pylon_data[i][2])

                self.marker(int(pair_ave_pix[0]), int(pair_ave_pix[1]))
                self.text("z="+str(pair_ave[2])+"[mm]","x="+str(pair_ave[0])+"[mm]" )
                self.pub_param(*pair_ave) #手前のパイロンの組の情報をパブリッシュ
                self.p_num(3)
                

            elif len(pylon_data) == 2: #画面上にパイロンが2つ存在する場合
                pylon_data.sort(key=lambda x: x[2]) #パイロンの位置関係に応じて並べ替え(手前:[0], 奥:[1])
                pair_ave =[(pylon_data[0][i] + pylon_data[1][i])/ 2 for i in range(3)]
                pylon_data.sort(key=lambda x: x[4])
                top_center.sort(key=lambda x: x[5])
                for i in range(2):
                    if pylon_data[i][3] == 0 or 1: #緑枠描画
                        cv2.rectangle(self.src, (top_center[i][0] - int(top_center[i][2] / 2), top_center[i][1] - int(top_center[i][3] / 2)), \
                                (top_center[i][0] + int(top_center[i][2] / 2), top_center[i][1] + int(top_center[i][3] / 2)), (0, 255, 0), 2)
                        self.pub_turning(pylon_data[i][0], pylon_data[i][2])

                self.text("z="+str(pair_ave[2])+"[mm]","x="+str(pair_ave[0])+"[mm]" )
                self.marker(int((top_center[0][0] + top_center[1][0])/2), int((top_center[0][1] + top_center[1][1])/2))

                #print(x_mean)
                distance = abs(pylon_data[0][0] - pylon_data[1][0]) #パイロン間の距離によってロボットが左右どちらかに行き過ぎてないか判別
                if distance < 1000 and pylon_data[0][0] < pylon_data[1][0]: #ロボットが左を向きすぎている
                    pair_ave.append(1) #signal = 1 (左行き過ぎのシグナル)

                if distance < 1000 and pylon_data[0][0] > pylon_data[1][0]: #ロボットが右を向きすている
                    pair_ave.append(2) #signal = 2 (右行き過ぎのシグナル)

                self.pub_param(*pair_ave)    
                self.p_num(2)


            elif len(pylon_data) == 1: #画面上にパイロンが1つしかない場合
                if pylon_data[0][3] == 0 or 1: #緑枠
                    cv2.rectangle(self.src, (top_center[0][0] - int(top_center[0][2] / 2), top_center[0][1] - int(top_center[0][3] / 2)), \
                            (top_center[0][0] + int(top_center[0][2] / 2), top_center[0][1] + int(top_center[0][3] / 2)), (0, 255, 0), 2)

                    self.pub_turning(pylon_data[0][0], pylon_data[0][2])
                self.marker(top_center[0][0], top_center[0][1])
                
                if pylon_data[0][0] < -100: #行き過ぎシグナル(調整中)
                    pylon_data[0][3] = 1 

                if pylon_data[0][0] > 100:
                    pylon_data[0][3] = 2

                self.pub_param(*pylon_data[0][:3])
                self.text("z="+str(pylon_data[0][2])+"[mm]","x="+str(pylon_data[0][0])+"[mm]")
                self.p_num(1)



            elif len(pylon_data) == 0:
                self.pub_param(0, 0, 0)
                self.marker(int(self.image.bgr_image.shape[1]/2), int(self.image.bgr_image.shape[0]/2))
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
        self.camera_publish.publish(ros_numpy.msgify(Image, self.src, encoding='bgr8'))

    def p_num(self, n):
        self.p_num_publish.publish(n)

    def marker(self,x,y):
        cv2.drawMarker(self.src, (x, y), (0,255,0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=10, thickness = 2)

    def text(self, *statement):
        count = 0
        for val in statement:
            cv2.putText(self.src, val, (5, 15 + count*20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color= (163,224,13), thickness=1)
            count += 1

class DataInput:
    def __init__(self):
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback )
        self.object_num = 0
        self.pylons = [] 
        self.rect = []
        self.is_callbacked = False

    def callback(self, msg):
        self.is_callbacked = True
        pyl = []
        rect = []
        for i in range(len(msg.bounding_boxes)):
            if msg.bounding_boxes[i].Class == "pylon":
                width = msg.bounding_boxes[i].xmax - msg.bounding_boxes[i].xmin
                height = msg.bounding_boxes[i].ymax - msg.bounding_boxes[i].ymin 
                pyl.append([msg.bounding_boxes[i].xmin + round(width/2), msg.bounding_boxes[i].ymin + round(height/2), width, height])
                rect.append([msg.bounding_boxes[i].xmin, msg.bounding_boxes[i].ymin, width, height])
                print(len(rect), "bdlen")
        self.pylons = pyl
        self.rect = rect
        print(self.rect, "test01")

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
    def image_callback(self, msg):
        self.bgr_image = ros_numpy.numpify(msg) 


if __name__ == '__main__':
    main = PylonDetector(sys.argv[1])
