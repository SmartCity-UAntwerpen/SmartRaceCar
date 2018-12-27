FROM astridvanneste/ros_indigo

ADD init.sh /home/docker/init.sh

ADD run.sh /home/docker/run.sh

ADD ROS /home/docker/ROS

RUN svn checkout https://github.com/SmartCity-UAntwerpen/SmartRaceCar/trunk/ROS /home/docker/ROS

RUN /home/docker/init.sh

RUN chmod +x /home/docker/run.sh

ENTRYPOINT ["/home/docker/run.sh"]