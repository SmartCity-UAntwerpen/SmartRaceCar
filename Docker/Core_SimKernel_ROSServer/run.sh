#!/usr/bin/env bash

echo $@
START_POINT=$1
PORT_1=$2
PORT_2=$3
SIM_ID=$4

cd /home/docker/ROS/workspace

source devel/setup.bash

roslaunch rossimulator rossim.launch &

cd /home/docker

nohup java -jar SimKernel.jar ${PORT_2} ${PORT_1} ${SIM_ID} &
java -jar Core.jar ${START_POINT} ${PORT_1} ${PORT_2}