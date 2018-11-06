#!/bin/bash
echo 'launching rover arm'
echo
nohup ssh nvidia@tx2 'bash -s' < rover_arm.sh &

echo 'launching pi'
echo
./base_arm.sh
