// Matthew Dombroski
// ECE 232 Lab 1
// Subscriber file to listen to pos and twist data of the robot

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

using namespace std;
using namespace ros;

void print_msgs(const nav_msgs::Odometry::ConstPtr& msg) {
	ofstream pos;
	ofstream twist;
	pos.open("Pos_data.txt", std::ofstream::out | std::ofstream::app);      // open the files in append mode
	twist.open("Twist_data.txt", std::ofstream::out | std::ofstream::app);  // to not overwrite previous data
	pos << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.orientation.x << ", " << msg->pose.pose.orientation.y << ", " << msg->pose.pose.orientation.z << ", " << msg->pose.pose.orientation.w << endl;
	twist << msg->twist.twist.linear.x << ", " << msg->twist.twist.angular.z << endl;
	pos.close();
	twist.close();
}

int main(int argc, char** argv) {
	init(argc, argv, "listener");
	
	NodeHandle n;
	Subscriber sub = n.subscribe("/odom",1000,print_msgs);
	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}
