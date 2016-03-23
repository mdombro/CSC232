#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "odomPub_cmdline.h"

using namespace ros;
using namespace std;

float minAngle;
float angleIncrement;
vector<float> scans;
float coneRadii;
float mu[3] = {0.0, 0.0, 0.0};

float z[3]; // holds range, bearing, and signature of feature

void callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    minAngle = msg->min_angle;
    angleIncrement = msg->angle_increment;
    scans.resize(msg->ranges.size());
    for (int i = 0; i < msg->ranges.size(); i++) {
        scans[i] = msg->ranges[i];
    }
}

int main(int argc, char* argv[]) {
    gengetopt_args_info args;
    cmdline_parser(argc,argv,&args);
    coneRadii = args.coneRadii_arg;
    init(argc, argv, "localizer");
    geometry_msgs::PoseWithCovariance msg;
    NodeHandle n;
    Publisher posWCov = n.advertise<geometry_msgs::PoseWithCovariance>("pos", 1);
    Subscriber cntrl = n.subscribe("/cmd_vel_mux/input/navi", 1000, cmmdUpdate);  // update the command velocities
    Subscriber beams = n.subscribe("/scan", 1000, reset);
    //ros::Duration(1.3).sleep();
    ros::Rate loop_rate(10);
    while (ros::ok()) {

        ros::spinOnce();
        pub.publish(posWCov);
        loop_rate.sleep();
    }
    return 0;
}

void findFeature(float[] z, float[] mu, vector<float> scans, float minAngle, float angleIncrement) {
    float angle = minAngle;
    float min[2] = {scans[0], minAngle};
    for (int i = 0; i < scans.size(); i++) {
        if (scans[i] == scans[i] && scans[i] < min) {
            min[0] = scans[i];
            min[1] = angle;
        }
        angle += angleIncrement;
    }
    z[0] = min[0] + coneRadii;
    z[1] = min[1];
}
