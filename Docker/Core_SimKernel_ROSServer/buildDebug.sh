#!/usr/bin/env bash

docker login
docker build -t smartracecar/latest -f DockerfileDebug .
docker push jaimievranckx/

$SHELL