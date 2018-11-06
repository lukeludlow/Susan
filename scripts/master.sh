echo 'launching rover arm'
echo
nohup ssh nvidia@tx2 'bash -s' < cat.sh &

echo 'launching pi'
echo
./arm_test.sh
