#!/usr/bin/env python
# -*- coding: utf-8 -*-
#GoProからの映像をImageとしてパブリッシュするノード

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from goprocam import GoProCamera, constants
import socket

class ImageInput:
    def __init__(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gopro = GoProCamera.GoPro(ip_address=GoProCamera.GoPro.getWebcamIP("usb1"), camera=constants.gpcontrol, webcam_device="usb1")
        gopro.webcamFOV(constants.Webcam.FOV.Narrow)
        gopro.startWebcam(resolution="720")
        self.cap = cv2.VideoCapture("udp://172.21.173.54:8554?overrun_nonfatal=1&fifo_size=50000000", cv2.CAP_FFMPEG)
        self.pub = rospy.Publisher('gopro_image', Image, queue_size=10)
        self.br = CvBridge()
    
    def image_publish(self):
        try:
            ret, frame = self.cap.read()
            #cv2.imshow("GoPro OpenCV", frame)
            bridge = self.br.cv2_to_imgmsg(frame, 'bgr8')
            self.pub.publish(bridge)
        except Exception as err:
            print(err)

if __name__=='__main__':
    rospy.init_node('GoPro')
    rate = rospy.Rate(100)
    cam = ImageInput()
    while not rospy.is_shutdown():
        cam.image_publish()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cam.cap.release()
    cv2.destroyAllWindows()

