#!/bin/bash
echo 'TEST pubtrick_shake.sh on pi'
export ROS_IP=houston
export ROS_HOSTNAME=houston
export ROS_MASTER_URI=http://tx2:11311
export | grep ROS

nohup rostopic pub /tricks rover_msgs/Temperature 0.49 &

sleep 1
export ROS_IP=tx2
export ROS_HOSTNAME=tx2
