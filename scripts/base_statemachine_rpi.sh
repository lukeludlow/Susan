echo 'launching state machine on pi'
echo
export ROS_HOSTNAME=houston
export ROS_IP=192.168.1.50
export ROS_MASTER_URI=http://tx2:11311
export ROS_ROOT=/opt/ros/kinetic/share/ros
roslaunch tricks tricks_rpi.launch
