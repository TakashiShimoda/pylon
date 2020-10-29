#!/usr/bin/env python
import rospy, cv2, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

print(sys.version)
