#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "naviSub_cmdline.h"

using namespace ros;
using namespace std;

float command[2];
Time last;  // time of last navi command
bool go = false; // flag for when navi commands stop being recieved

void cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
	command[0] = msg->linear.x;
	command[1] = msg->angular.z;
	last = Time::now();
	go = true;
}

int main(int argc, char** argv) {
	init(argc, argv, "navi");
	geometry_msgs::Twist msg;
	NodeHandle n;
	Subscriber sub = n.subscribe("/cmd_vel_mux/input/navi",1000,cmdUpdate);
	Publisher pub = n.advertise<geometry_msgs::Twist>("navi", 1);
	last = Time::now();

	Rate loop_rate(10);
	while (ros::ok()) {
		cout << Duration(Time::now().toSec() - last.toSec()).toSec() << endl;

		// If navi commands were previously recieved and now have stopped being recieved
		// reset the command velocities to 0 and start listening for new commands
		if (Duration(Time::now().toSec() - last.toSec()).toSec() > 0.15 && go == true) {
			command[0] = 0;
			command[1] = 0;
			go = false;
		}
		msg.linear.x = command[0];
		msg.angular.z = command[1];
		ros::spinOnce();
		pub.publish(msg);
		loop_rate.sleep();
	}
	return 0;
}
