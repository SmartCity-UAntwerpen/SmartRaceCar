#!/usr/bin/env bash

docker login
docker build -t astridvanneste/core_simkernel .
docker push astridvanneste/core_simkernel