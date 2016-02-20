#include <iostream>
#include <ros/ros.h>
#include <kobuki_msgs/BumperEvent.h>

using namespace ros;
using namespace std;

void print_msgs(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
	if (msg->bumper == 0 && msg->state == 1) {
		cout << "Left" << endl;
	}
	if (msg->bumper == 1 && msg->state == 1) {
		cout << "Center" << endl;
	}
	if (msg->bumper == 2 && msg->state == 1) {
		cout << "Right" << endl;
	}
}

int main(int argc, char** argv) {
	init(argc, argv, "listener");
	
	NodeHandle n;
	Subscriber sub = n.subscribe("/mobile_base/events/bumper",1000,print_msgs);
	ros::spin();
	return 0;
}
