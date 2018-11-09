#!/bin/bash
#rostopic pub -r 1 /tricks rover_msgs/Trick 'nod'
#rostopic pub -r 1 /tricks rover_msgs/Temperature 0.69
#echo 'source ~/.bashrc; /opt/ros/kinetic/bin/rostopic pub /tricks rover_msgs/Temperature 0.69' | ssh nvidia@tx2
#echo '~/pubtrick_nod.sh' | ssh nvidia@tx2
echo 'TEST pubtrick_nod.sh on pi'
#nohup ssh nvidia@tx2 'bash -s' < pubnod.sh &
#export ROS_IP=192.168.1.50
#export ROS_MASTER_URI=http://tx2:11311
#export | grep ROS
#./setpath.sh
#export | grep ROS

export ROS_IP=houston
export ROS_HOSTNAME=houston
export ROS_MASTER_URI=http://tx2:11311
export | grep ROS

nohup rostopic pub /tricks rover_msgs/Temperature 0.69 &
#PID=$!
#sleep 1
#kill -SIGINT $PID

sleep 1
export ROS_IP=tx2
export ROS_HOSTNAME=tx2

export | grep ROS
