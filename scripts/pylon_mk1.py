#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
2019-7-23 yamada
パイロン認識
usb_cam_node と併用で利用可能

"""

import rospy
import numpy as np
import sys
import cv2
import time
import math
import socket
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from goprocam import GoProCamera, constants

#以下, GoProで画像取り込み(未完成)
class ImageImput:
    def __init__(self):
        self.bgr_image = None

    def image_callback():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gopro = GoProCamera.GoPro(ip_address=GoProCamera.GoPro.getWebcamIP("enp0s20f0u3"), camera=constants.gpcontrol, webcam_device="enp0s20f0u3")
        gopro.webcamFOV(constants.Webcam.FOV.Wide)
        gopro.startWebcam(resolution="480")
        cap = cv2.VideoCapture("udp://172.21.173.54:8554", cv2.CAP_FFMPEG)
        t = time.time()
        
        while True:
            try:
                self.bgr_image  = cap.read()
                # frame = cv2.resize(frame, (1696, 960))

            except CV_Error as e:
                rospy.logerr(e)


"""
class ImageImput:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.bgr_image = None

    def image_callback(self, image_data):
        try:
            self.bgr_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
"""

class PylonDetector:
    # パイロンナンバー
    PYLON_NUMBER = 1
    # 赤パイロンのHSV閾値
    RED_PYLON_COLOR = (0, 40, 40), (2, 255, 255), (170, 40, 40), (179, 255, 255)
    # ガウシアンカーネルサイズ
    GAUSSIAN_KERNEL_SIZE = (3, 3)
    # モルフォロジー演算カーネルサイズ
    MORPHOLOGY_KERNEL_SIZE = np.ones((5,5), np.uint8)
    # パイロン〜カメラ間の実際の測定距離[mm]
    BASE_DISTANCE = 600
    # BASE_DISTANCEでのパイロンの高さ[pixel]
    BASE_HEIGHT = 90
    # パイロンの高さ[mm]
    PYLON_HEIGHT = 76
    # 画面の中央(横方向)[pixel]
    IMAGE_CENTER_X = 320
    # 画面の中央(高さ方向)[pixel]
    IMAGE_CENTER_Y = 240
    # パイロンの最小高さ
    MIN_HEIGHT = 30
    # 600mm先のz位置[mm]
    Z = 600

    def __init__(self):
        self.is_pylon_data = False
        self.image_imput = ImageImput()
        self.camera_publish = rospy.Publisher("/camera_image", Image, queue_size=10)
        self.convert_x_publish = rospy.Publisher("/data_x", Float32, queue_size=1)
        self.convert_y_publish = rospy.Publisher("/data_y", Float32, queue_size=1)
        self.z_publish = rospy.Publisher("/data_z", Float32, queue_size=1)
        self.h_publish = rospy.Publisher("/data_h", Float32, queue_size=1)
        self.depth_publish = rospy.Publisher("/data_depth", Float32, queue_size=1)
        # 画像配列を生成
        self.image = np.arange(27).reshape(3, 3, 3)
        # データを生成
        self.z_datas = [0] * self.PYLON_NUMBER
        self.h_datas = [0] * self.PYLON_NUMBER
        self.convert_x_datas = [0] * self.PYLON_NUMBER

    # 検出のメイン(流れ)
    def detection_main(self):
        image_width = self.image.shape[0]
        if image_width > 3:
            # BGRからHSVに変換
            self.hsv_convention()
            # パイロンの輪郭検出
            countours = self.outline_detection()
            # パイロンの情報
            self.is_pylon_data = self.pylon_infomations(countours, 0)
            # パラメータのセット
            self.set_param()
            # イメージをパブリッシュ
            self.pub_image()
            # cv2.imshow('KUKEI', self.image)

    # BGRからHSVに変換
    def hsv_convention(self):
        self.hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

    # パイロンの輪郭検出
    def outline_detection(self):
        # ガウシアンフィルタをかける
        gaussian_filter_image = cv2.GaussianBlur(self.hsv_image, self.GAUSSIAN_KERNEL_SIZE, 0)
        # 二値化
        binarization_image_1 = cv2.inRange(gaussian_filter_image, self.RED_PYLON_COLOR[0], self.RED_PYLON_COLOR[1])
        binarization_image_2 = cv2.inRange(gaussian_filter_image, self.RED_PYLON_COLOR[2], self.RED_PYLON_COLOR[3])
        binarization_image = cv2.add(binarization_image_1, binarization_image_2)
        # クロージング(膨張、収縮の順に行う演算)
        morphology_close_image = cv2.morphologyEx(binarization_image, cv2.MORPH_CLOSE, self.MORPHOLOGY_KERNEL_SIZE)
        # cv2.imshow('mask', morphology_close_image)
        # 輪郭検出
        image, countours, hierarchy = cv2.findContours(morphology_close_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print countours
        return countours

    # パイロンの情報
    def pylon_infomations(self, countours, index_number):
        if len(countours) > index_number:
            # 面積が大きい順に並べ替える
            countours.sort(key=cv2.contourArea, reverse=True)
            cnt = countours[index_number]
            
            ## 回転を考慮しない外接円
            # 左上の座標(x, y)、横と縦のサイズ(width, height)を取得
            x,y,width,height = cv2.boundingRect(cnt)
            # 矩形の左上の座標取得
            top_left = (int(x), int(y))
            # 矩形の高さ(縦)[pixel]
            height = float(height)
            if height == 0:
                return False
            # 矩形の幅(横)[pixel]
            width = float(width)
            # パイロンの縦横比
            aspect_ratio = float(width / height)
            # print aspect_ratio
            # aspect_ratioの判定 >> パイロンの判別
            if (aspect_ratio > 1.4) and (aspect_ratio < 1.8):
                if height <= self.MIN_HEIGHT:
                    return False
                # 矩形描画
                cv2.rectangle(self.image, (x, y-int(height*0.8)), (x+int(width), y+int(height)), (255, 0, 0), 2)
                
            # ## 回転を考慮した外接円
            # # Box2D(左上の座標(x, y)、横と縦のサイズ(width, height)、回転角)を取得
            # rect = cv2.minAreaRect(cnt)
            # # 4点座標に変換
            # box = np.int0(cv2.boxPoints(rect))
            # # 矩形の左上の座標取得
            # top_left = (int(rect[0][0]), int(rect[0][1]))
            # # 矩形の高さ(縦)[pixel]
            # height = float(rect[1][1])
            # if height == 0:
            #     return False
            # # 矩形の幅(横)[pixel]
            # width = float(rect[1][0])
            # # パイロンの縦横比
            # aspect_ratio = float(width / height)
            # print aspect_ratio
            # # aspect_ratioの判定 >> パイロンの判別
            # if (aspect_ratio > 1.6) and (aspect_ratio < 1.8):
            #     if height <= self.MIN_HEIGHT:
            #         return False
            #     # 矩形描画
            #     cv2.drawContours(self.image, [box], 0, (255, 0, 0), 2)
            
                convert_x, convert_y, Z, depth = self.calculate_depth(aspect_ratio, height, top_left)
                
                self.z_datas[index_number] = Z*0.58
                self.h_datas[index_number] = height
                self.convert_x_datas[index_number] = convert_x
                self.pub_param(convert_x, convert_y, Z*0.58, height, depth)
                # print convert_x
                # print Z*0.58
                return True
            
            if (aspect_ratio > 0.7) and (aspect_ratio < 1.4):
                if height <= self.MIN_HEIGHT:
                    return False
                # 矩形描画(回転あり)
                # cv2.drawContours(self.image, [box], 0, (0, 255, 0), 2)
                # 矩形描画(回転なし)
                cv2.rectangle(self.image, (x, y), (x+int(width), y+int(height)), (0, 255, 0), 2)
                
                convert_x, convert_y, Z, depth = self.calculate_depth(aspect_ratio, height, top_left)
                
                self.z_datas[index_number] = Z
                self.h_datas[index_number] = height
                self.convert_x_datas[index_number] = convert_x
                self.pub_param(convert_x, convert_y, Z, height, depth)
                #print Z
                return True
        else:
            return False

    # 三次元位置(X, Y, Z)の計算(回転なし)
    def calculate_depth(self, aspect_ratio, height, top_left):
        # 奥行き方向の計算
        depth = (self.BASE_HEIGHT / float(height)) * self.BASE_DISTANCE
        # 1ピクセル当たりの大きさを計算
        one_pixel_size = self.PYLON_HEIGHT / float(height)
        # 三次元位置(X, Y, Z)の計算
        convert_x = (top_left[0] - self.IMAGE_CENTER_X) * one_pixel_size
        convert_y = (self.IMAGE_CENTER_Y - top_left[1]) * one_pixel_size
        if depth > convert_x:
            Z = math.sqrt(depth * depth - convert_x * convert_x)
        convert_x, convert_y, Z, depth = round(convert_x), round(convert_y), round(Z), round(depth)
        return convert_x, convert_y, Z, depth
        
    # # 三次元位置(X, Y, Z)の計算(回転あり)
    # def calculate_depth(self, aspect_ratio, height, top_left):
    #     # 奥行き方向の計算
    #     depth = (self.BASE_HEIGHT / float(height)) * self.BASE_DISTANCE
    #     # 1ピクセル当たりの大きさを計算
    #     one_pixel_size = self.PYLON_HEIGHT / float(height)
    #     # 三次元位置(X, Y, Z)の計算
    #     convert_x = (top_left[0] - self.IMAGE_CENTER_X) * one_pixel_size
    #     convert_y = (self.IMAGE_CENTER_Y - top_left[1]) * one_pixel_size
    #     if depth > convert_x:
    #         Z = math.sqrt(depth * depth - convert_x * convert_x)
    #     convert_x, convert_y, Z, depth = round(convert_x), round(convert_y), round(Z), round(depth)
    #     return convert_x, convert_y, Z, depth
    
    # パラメータをセット
    def set_param(self):
        max_index = self.h_datas.index(max(self.h_datas))
        if not self.is_pylon_data == None:
            rospy.set_param('/pylon_detection/param_circle_data_flag', self.is_pylon_data)
            rospy.set_param('/pylon_detection/param_x', self.convert_x_datas[max_index])
            rospy.set_param('/pylon_detection/param_z', self.z_datas[max_index])
            #print self.z_datas[max_index]

    # イメージをパブリッシュ
    def pub_image(self):
        self.camera_publish.publish(self.image_imput.bridge.cv2_to_imgmsg(self.image, 'bgr8'))

    # パラメータをパブリッシュ
    def pub_param(self, convert_x, convert_y, Z, height, depth):
        self.convert_x_publish.publish(convert_x)
        self.convert_y_publish.publish(convert_y)
        self.z_publish.publish(Z)
        self.h_publish.publish(height)
        self.depth_publish.publish(depth)

def circle_dector_main():
    # ビデオ映像の取得
    rospy.init_node('pylon_dector')
    pylon_dector = PylonDetector()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        start = time.time()
        pylon_dector.image = pylon_dector.image_imput.bgr_image
        pylon_dector.detection_main()

        # 終了の合図
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()
        end_time = time.time() - start
        # print end_time
    cv2.destroyAllWindows()

if __name__ == '__main__':
    circle_dector_main()
