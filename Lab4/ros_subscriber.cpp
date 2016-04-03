// Matthew Dombroski
// ECE 232 Lab 3
// Subscriber file to listen to pos and twist data of the robot\
// Also saves beam sensor data to files

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"

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

void save_scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
	ofstream beams;
	beams.open("Beams_data.txt", std::ofstream::out | std::ofstream::app);
	int num = msg->ranges.size();
	for (int i = 0; i < num-1; i++) {
		beams << msg->ranges[i] << ", ";
	}
	beams << msg->ranges[num-1] << ", " << msg->angle_min << ", " << msg->angle_max << ", " << msg->angle_increment << ", " << msg->time_increment << ", " << msg->scan_time << ", " << msg->range_min << ", " << msg->range_max << endl;
        cout << "Got scans" << endl;
}

void save_mean(const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
	ofstream mean;
	mean.open("Mean_data.txt",std::ofstream::out | std::ofstream::app);
	mean << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", " << msg->pose.orientation.z << ", " << msg->pose.orientation.w;
	for (int i = 0; i < 10; i++) {
		mean << ", " << msg->covariance[i];
	}
	mean << endl;
}

int main(int argc, char** argv) {
	init(argc, argv, "listener");

	NodeHandle n;
	Subscriber sub = n.subscribe("/odom",1000,print_msgs);
	Subscriber laser = n.subscribe("/scan", 1000, save_scan);
	Subscriber localizer = n.subscribe("/pos", 1000, save_mean);
	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}
