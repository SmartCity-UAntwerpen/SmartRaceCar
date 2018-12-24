# Docker with Core and Simkernel
# based on: https://dzone.com/articles/creating-a-docker-image-with-ubuntu-and-java

FROM ubuntu:14.04

# Variables
ENV CORE=https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/core/release/core.jar
ENV SIMKERNEL=https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/simkernel/release/simkernel.jar

# Installing java8


RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y  software-properties-common && \
    add-apt-repository ppa:webupd8team/java -y && \
    apt-get update && \
    echo oracle-java7-installer shared/accepted-oracle-license-v1-1 select true | /usr/bin/debconf-set-selections && \
    apt-get install -y oracle-java8-installer && \
    apt-get clean

# Adding Core.jar and SimKernel.jar

ADD $CORE /home/docker/Core.jar
ADD $SIMKERNEL /home/docker/SimKernel.jar

ADD Simkernel.properties /home/docker/Simkernel.properties
ADD core.properties /home/docker/core.properties
ADD maps.xml /home/docker/maps/maps.xml

ADD run.sh /home/docker/run.sh

RUN chmod +x /home/docker/run.sh


ENTRYPOINT ["/home/docker/run.sh"]
CMD ["46"]
CMD ["5005"]
CMD ["5006"]
CMD ["0"]
