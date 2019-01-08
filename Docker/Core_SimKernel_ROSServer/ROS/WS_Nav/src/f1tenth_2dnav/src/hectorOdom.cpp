#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>

class genOdom
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_;
  ros::Subscriber sub_;
  nav_msgs::Odometry odom_;  
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;


  void callback(const nav_msgs::Odometry& msg)
  {
	odom_.header.stamp = ros::Time::now();
	odom_.header.frame_id = "odom";
	odom_.child_frame_id = "base_frame";

	//quaternion from yaw
	geometry_msgs::Quaternion odom_quat = msg.pose.pose.orientation;

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_frame";
	odom_trans.transform.translation.x = msg.pose.pose.position.x;
	odom_trans.transform.translation.y = msg.pose.pose.position.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transmform
	odom_broadcaster.sendTransform(odom_trans);

	//set the odometry message 
	odom_.pose.pose.position.x = msg.pose.pose.position.x;
	odom_.pose.pose.position.y = msg.pose.pose.position.y;
	odom_.pose.pose.position.z = 0;
	odom_.pose.pose.orientation = odom_quat;

	//publish the message
	pub_.publish(odom_);	
	
  }

  public:
  genOdom():nh_(), private_nh_("~")
  {
	pub_ = nh_.advertise<nav_msgs::Odometry>("hectorOdom",10);
	sub_ = nh_.subscribe("/scanmatch_odom",10,&genOdom::callback,this);

  }
  
};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"hectorOdom");
	genOdom odomObject;
	ROS_INFO("hectorOdom started");
	ros::spin();
	return 0;
}


