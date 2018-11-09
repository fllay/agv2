#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('cam_dir', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    pub.publish(2)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
