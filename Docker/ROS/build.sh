#!/usr/bin/env bash

docker login
docker build -t astridvanneste/ros_indigo .
docker push astridvanneste/ros_indigo