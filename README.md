List which services are loaed:

pi@raspberrypi:~ $ systemctl list-units --type service
UNIT                                                        LOAD   ACTIVE SUB     DESCRIPTION                              
agv_dual.service                                            loaded active running "bringup agv_dual"                       
alsa-restore.service                                        loaded active exited  Save/Restore Sound Card State            
avahi-daemon.service                                        loaded active running Avahi mDNS/DNS-SD Stack                  
binfmt-support.service                                      loaded active exited  Enable support for additional executable 
bluetooth.service                                           loaded active running Bluetooth service                        


To enable single camera:
sudo systemctl disable agv_dual.service
sudo systemctl enable agv2.service 

To enable dual camera:
sudo systemctl disable agv2.service 
sudo systemctl enable agv_dual.service


shell scripts to start and stop agv service
pi@raspberrypi:~ $ ls -l /usr/sbin/agv*
-rwxr-xr-x 1 root root 2516 Oct  1 03:26 /usr/sbin/agv2-start
-rwxr-xr-x 1 root root  271 Sep 30 14:30 /usr/sbin/agv2-stop
-rwxr-xr-x 1 root root 2512 Oct  1 10:12 /usr/sbin/agv_dual-start
-rwxr-xr-x 1 root root  279 Oct  1 10:11 /usr/sbin/agv_dual-stop

Content of agv start file:
#!/bin/bash
# THIS IS A GENERATED FILE, NOT RECOMMENDED TO EDIT.

/home/pi/mount_camera.sh  <-------- load camera /dev/video0
#sleep 5
function log() {
  logger -s -p user.$1 ${@:2}
}

log info "agv2: Using workspace setup file /opt/ros/kinetic/setup.bash"
source /opt/ros/kinetic/setup.bash
JOB_FOLDER=/etc/ros/kinetic/agv2.d


Modify start up launch file (single camera).
sudo pico /etc/ros/kinetic/agv2.d/agv_single.launch
Modify start up launch file (dual camera).
sudo pico /etc/ros/kinetic/agv_dual.d/agv.launch


edit service file single camera:
sudo pico /lib/systemd/system/agv2.service 
edit service file dual cameras:
sudo pico /lib/systemd/system/agv_dual.service


Content of the service files:

# THIS IS A GENERATED FILE, NOT RECOMMENDED TO EDIT.

[Unit]
Description="bringup agv_dual"
After=network.target

[Service]
Type=idle  <------ need to be set to idle so that it will be loaded after all services are up
ExecStart=/usr/sbin/agv_dual-start

[Install]
WantedBy=multi-user.target

Install agv2 package:
pi@raspberrypi:~ $ cd ros_catkin_ws/
pi@raspberrypi:~/ros_catkin_ws $ ./src/catkin/bin/catkin_make_isolated  --pkg agv2 -j2 --install --install-space /opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release

