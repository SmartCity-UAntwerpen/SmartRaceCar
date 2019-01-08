#include "ros/ros.h"
#include "std_msgs/String.h"
#include "race/


#include <sstream>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <vector>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>

bool longRange = false;
float ranges[1024];
std::vector<float> franges;
float currentRange = 10;

void setRanges(float x){
	for(int i = 0; i<1024; i++){
		ranges[i] = x + ((std::rand()%100-50)/1000.0);
	}
	franges.assign(ranges, ranges+1024);
	
}

void range_input()
{
	while (ros::ok()){
		float n;
		std::cin>> n;
		setRanges(n);
    currentRange = n;
	ROS_INFO("Moved laserscan range to %f...", n);
	}

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::LaserScan>("scan", 100);

  ros::Rate loop_rate(32);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
 uint32_t count = 0;
	float zeros[1024];
	float ones[1024];

	for(int i = 0; i<1024; i++){
		zeros[i] = 0;
	}

	std::vector<float> fzeros;
	fzeros.assign(zeros, zeros+1024);

	setRanges(10);

  ROS_INFO("Starting...");
	std::thread t(range_input);
	
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

/*
		if(count > 200 && close == false){
		  ROS_INFO("Moved laserscan range to 1...");
			close = true;
		}
*/

    setRanges(currentRange);

		sensor_msgs::LaserScan scan;
		std_msgs::Header header;

		//Fill laserscan message
		//Fill header message
		header.seq = count;
		header.stamp = ros::Time::now();
		header.frame_id = "laser";

		scan.header = header;
		scan.angle_min = -2.26889;
		scan.angle_max = 2.26889;
		scan.angle_increment = 0.504198908806;
		scan.time_increment = 0.0;
		scan.scan_time = 0.0;		
		scan.range_min = 0.08;
		scan.range_max = 30;
		scan.ranges = franges;
		scan.intensities = fzeros;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(scan);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
