#!/usr/bin/env python
import rospy, sys, cv2
from std_msgs.msg import String
def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world "
        str2 = sys.version_info
        str = cv2.__version__
        rospy.loginfo(str)
        rospy.loginfo(str2)
        pub.publish(String(str))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass