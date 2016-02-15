#include <iostream>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

using namespace ros;

void print_msgs(const nav_msgs::Odometry::ConstPtr& msg) {
	std::cout << "pose = " << msg->pose.pose << std::endl;
}

int main(int argc, char** argv) {
	init(argc, argv, "listener");
	
	NodeHandle n;
	Subscriber sub = n.subscribe("/odom",1000,print_msgs);
	ros::spin();
	return 0;
}
