#!/usr/bin/env bash

docker system prune -a -f
docker login
docker build -t astridvanneste/core_simkernel .
docker push astridvanneste/core_simkernel