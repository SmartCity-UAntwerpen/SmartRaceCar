#!/usr/bin/env bash

START_POINT=$1
SERVER_PORT=$2
CLIENT_PORT=$3
SIM_ID=$4

cd /home/docker

nohup java -jar SimKernel.jar ${CLIENT_PORT} ${SERVER_PORT} &
java -jar Core.jar ${START_POINT} ${SERVER_PORT} ${CLIENT_PORT} ${SIM_ID}