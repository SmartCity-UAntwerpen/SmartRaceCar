# Docker with Core and Simkernel
# based on: https://dzone.com/articles/creating-a-docker-image-with-ubuntu-and-java

FROM astridvanneste/java

# Variables
ENV CORE=https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/core/release/core.jar
ENV SIMKERNEL=https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/simkernel/release/simkernel.jar

# Adding Core.jar and SimKernel.jar

ADD $CORE /home/docker/Core.jar
ADD $SIMKERNEL /home/docker/SimKernel.jar

ADD maps.xml /home/docker/maps/maps.xml

ADD run.sh /home/docker/run.sh

RUN chmod +x /home/docker/run.sh
RUN mkdir /home/docker/config


ENTRYPOINT ["/home/docker/run.sh"]
CMD ["46"]
CMD ["0"]
CMD ["./"]
