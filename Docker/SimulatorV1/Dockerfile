# Docker with Core and Simkernel
# based on: https://dzone.com/articles/creating-a-docker-image-with-ubuntu-and-java

FROM ubuntu:15.04

# Variables
ENV CORE=https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/core/release/core.jar
ENV SIMKERNEL=

# Installing java8

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y openjdk-8-jdk && \
    apt-get clean

# Adding Core.jar and SimKernel.jar

ADD $SIMKERNEL /home/jars/SimKernel.jar &&/
    $CORE /home/jars/Core.jar
