#!/usr/bin/env python
import rospy

import dynamic_reconfigure.client
from std_msgs.msg import Int32

MAX_SPEED = 700000

def subCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard maxspeed %s", data.data)
    client.update_configuration({"maximun_speed":data.data})

def subCurveSpeedCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard slow speed %s", data.data)
    client.update_configuration({"slow_speed":data.data})

def callback(config):
    rospy.loginfo("Config is called ")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('param_controller', anonymous=True)

    rospy.Subscriber("max_speed", Int32, subCallback)
    rospy.Subscriber("slow_speed", Int32, subCurveSpeedCallback)
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    client = dynamic_reconfigure.client.Client("drive_motor", timeout=30, config_callback=callback)
    listener()
    

    

