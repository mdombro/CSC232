// Matthew Dombroski

#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "ros_publisher_cmdline.h"

using namespace std;
using namespace ros;


int main (int argc, char* argv[]) {
	gengetopt_args_info args;
	cmdline_parser(argc,argv,&args);
	double v = args.linearVelocity_arg;
	double a = args.angularVelocity_arg;
	double time = args.time_arg;
	int count = 0;
	init(argc, argv, "talker");
	geometry_msgs::Twist msg;
	msg.linear.x = v;
	msg.angular.z = a;

	NodeHandle n;
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	ros::Rate loop_rate(10);
	while (ros::ok() && count < (int)time*10) {
		ros::spinOnce();
		pub.publish(msg);
		loop_rate.sleep();
		count++;
	}
	for (int i = 0; i < 5; i++) {
		msg.linear.x = 0;
		msg.angular.z = 0;
		pub.publish(msg);
		ros::spinOnce();
	}



	return 0;

	//init(argc, argv, "talker");
	//geometry_msgs::Twist msg;
	//msg.linear.x = 1.0;
	//msg.angular.z = .5;

	//int max_count = 30;
	//int count = 0;
	//NodeHandle n;
	//Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	//ros::Rate loop_rate(10);
	//while (ros::ok() && count < max_count) {
	//	ros::spinOnce();
	//	pub.publish(msg);
	//	loop_rate.sleep();
	//	count++;
	//}
	//return 0;
}
