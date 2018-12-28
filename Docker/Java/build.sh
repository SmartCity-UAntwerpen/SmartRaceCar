#!/usr/bin/env bash

docker system prune -a -f
docker login
docker build -t astridvanneste/java .
docker push astridvanneste/java