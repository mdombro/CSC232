// Matthew Dombroski
// ECE 232 Lab 1
// Publisher file for open loop robot control

#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "ros_publisher_cmdline.h"
#include <math.h>

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
		msg.angular.z = sin(count/10);   // uncomment to preform sin(t) movement
		//msg.linear.x = 0.25*sin(count/10.0);		
		ros::spinOnce();		
		pub.publish(msg);
		loop_rate.sleep();
		count++;
	}
	return 0;
}
