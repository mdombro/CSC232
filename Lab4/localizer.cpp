#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <Matrix.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "localizer_cmdline.h"

using namespace ros;
using namespace std;

float command[2];

float minAngle;
float angleIncrement;
vector<float> scans;
double coneRadii;
float mu[3] = {0.0, 0.0, 0.0};

float z[3]; // holds range, bearing, and signature of feature

void handleScans(const sensor_msgs::LaserScan::ConstPtr& msg) {
    minAngle = msg->angle_min;
    angleIncrement = msg->angle_increment;
    scans.resize(msg->ranges.size());
    for (int i = 0; i < msg->ranges.size(); i++) {
        scans[i] = msg->ranges[i];
    }
}

void cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
	command[0] = msg->linear.x;
	command[1] = msg->angular.z;
}

int main(int argc, char* argv[]) {
    gengetopt_args_info args;
    cmdline_parser(argc,argv,&args);
    coneRadii = args.coneRadii_arg;
    init(argc, argv, "localizer");
    geometry_msgs::PoseWithCovariance msg;
    NodeHandle n;
    Publisher posWCov = n.advertise<geometry_msgs::PoseWithCovariance>("/pos", 1);
    Subscriber cntrl = n.subscribe("/cmd_vel_mux/input/navi", 1000, cmdUpdate);  // update the command velocities
    Subscriber beams = n.subscribe("/scan", 1000, handleScans);
    //ros::Duration(1.3).sleep();
    ros::Rate loop_rate(10);
    while (ros::ok()) {

        ros::spinOnce();
        posWCov.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}

// void findFeature(float[] z, float[] mu, vector<float> scans, float minAngle, float angleIncrement) {
//     float angle = minAngle;
//     float min[2] = {scans[0], minAngle};
//     for (int i = 0; i < scans.size(); i++) {
//         if (scans[i] == scans[i] && scans[i] < min) {
//             min[0] = scans[i];
//             min[1] = angle;
//         }
//         angle += angleIncrement;
//     }
//     z[0] = min[0] + coneRadii;
//     z[1] = min[1];
// }
