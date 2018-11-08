#!/bin/bash
#rostopic pub -r 1 /tricks rover_msgs/Trick 'nod'
#rostopic pub -r 1 /tricks rover_msgs/Temperature 0.69
echo 'source ~/.bashrc; rostopic pub /tricks rover_msgs/Temperature 0.69' | ssh nvidia@tx2
