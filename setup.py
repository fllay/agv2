#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['agv2'],
    package_dir={'':'/home/pi/ros_catkin_ws/src'},
    requires=['std_msgs', 'roscpp','rospy'],
    install_requires=['numoy','cv2','RPi.GPIO','pigpio']

)

setup(**d)
