#!/usr/bin/env bash

STARTPOINT=$1
SERVER_PORT=$2
CLIENT_PORT=$3

java -jar /home/jars/SimKernel.jar $CLIENT_PORT $SERVER_PORT
java -jar /home/jars/Core.jar $STARTPOINT $SERVER_PORT $CLIENT_PORT