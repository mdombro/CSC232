#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "naviSub_cmdline.h"

using namespace ros;
using namespace std;

float command[2];

void print_msgs(const geometry_msgs::Twist::ConstPtr& msg) {
	command[0] = msg->linear.x;
	command[1] = msg->angular.z;
}

int main(int argc, char** argv) {
	init(argc, argv, "navi");
	geometry_msgs::Twist msg;
	NodeHandle n;
	Subscriber sub = n.subscribe("/cmd_vel_mux/input/navi",1000,print_msgs);
	Publisher pub = n.advertise<geometry_msgs::Twist>("navi", 1);
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		msg.linear.x = command[0];
		msg.angular.z = command[1];
		ros::spinOnce();
		pub.publish(msg);
		loop_rate.sleep();
	}
	return 0;
}
