#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h> //Lidar msg
#include <geometry_msgs/Twist.h> //Velocity vector
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <clearpath_horizon/RawEncoders.h>
#include <indoor_pos/ips_msg.h>

#include <sstream>



void encoder_callback(const clearpath_horizon::RawEncodersConstPtr &msg)
{
	ROS_INFO("ENCODER:I got: [%d] as encoder ticks", msg->ticks[0]);
}

void gps_callback(const gps_common::GPSFixConstPtr &msg)
{
	ROS_INFO("GPS:I got: [%f] as latitude", msg->latitude);
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("LIDAR:I got: [%f] as range_min", msg->range_min);
}

void ips_callback(const indoor_pos::ips_msg::ConstPtr& msg)
{
	ROS_INFO("IPS:I got: X = %f, Y = %f, Yaw = %f", msg->X, msg->Y, msg->Yaw);
}

int main(int argc, char **argv)
{

	ros::init(argc,argv,"main_control");


	geometry_msgs::Twist velocity_command;

	ros::NodeHandle n;
	ros::Subscriber lidar_sub = n.subscribe("/scan",1000,lidar_callback);
	ros::Subscriber gps_sub = n.subscribe("fix",1000,gps_callback);
	ros::Subscriber encdoder_sub = n.subscribe("/clearpath/robots/default/data/raw_encoders",1000,encoder_callback);
	ros::Subscriber ips_sub = n.subscribe("indoor_pos",1000,ips_callback);

	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/clearpath/robots/default/cmd_vel", 1000);


	//main loop
	ros::Rate loop_rate(20);		//20Hz update rate
	while (ros::ok())
  	{
		velocity_command.linear.x = 100;
		velocity_command.angular.z = -50;
		velocity_publisher.publish(velocity_command);
		ros::spinOnce();	
		loop_rate.sleep();			//used for maintaining rate
	}
	return 0;
}

