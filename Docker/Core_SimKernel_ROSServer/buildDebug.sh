#!/usr/bin/env bash

docker login
docker build -t jaimievranckx/racecar_docker -f DockerfileDebug .
docker push jaimievranckx/
