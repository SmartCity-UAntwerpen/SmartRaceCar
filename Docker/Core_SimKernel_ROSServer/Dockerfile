FROM astridvanneste/ros_indigo

# Initialization
ADD init.sh /home/docker/init.sh
ADD init_python.sh /home/docker/init_python.sh

ADD run.sh /home/docker/run.sh

ADD ROS /home/docker/ROS

RUN svn checkout https://github.com/SmartCity-UAntwerpen/SmartRaceCar/trunk/ROS /home/docker/ROS

RUN /home/docker/init.sh
RUN /home/docker/init_python.sh

# Variables
ENV CORE=https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/core/release/core.jar
ENV SIMKERNEL=https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/simkernel/release/simkernel.jar

# Adding Core.jar and SimKernel.jar

ADD $CORE /home/docker/Core.jar
ADD $SIMKERNEL /home/docker/SimKernel.jar

ADD maps.xml /home/docker/maps/maps.xml

RUN chmod +x /home/docker/run.sh
RUN mkdir /home/docker/config

ENTRYPOINT ["/home/docker/run.sh"]
CMD ["46"]
CMD ["0"]
CMD ["./"]