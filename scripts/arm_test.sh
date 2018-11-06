export ROS_HOSTNAME=houston
export ROS_IP=192.168.1.50
export ROS_MASTER_URI=http://tx2:11311
export ROS_ROOT=/opt/ros/kinetic/share/ros
sudo ln -s /dev/input/js0 /dev/input/xbox_one
sudo ln -s /dev/input/js1 /dev/input/xbox_one_arm

roslaunch hal_control base_arm.launch
