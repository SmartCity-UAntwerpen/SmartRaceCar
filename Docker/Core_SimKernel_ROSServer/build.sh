#!/usr/bin/env bash

docker login
docker build -t astridvanneste/core_simkernel_rosserver .
docker push astridvanneste/core_simkernel_rosserver