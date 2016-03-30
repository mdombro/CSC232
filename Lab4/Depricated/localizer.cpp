#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "localizer_cmdline.h"
#include <Eigen/Dense>

// using namespace ros;
// using namespace std;
// using namespace Eigen;

/*** Odometry command vector  ***/
// RowVector2f u;
//
// /*** LaserScan return information ***/
// float minAngle;
// float angleIncrement;
// vector<float> scans;
//
// /*** predetermined cone radius ***/
// double coneRadii;
//
// /*** EKF Matrix variables ***/
// // current robot position belief
// RowVector3f mu
//
// // holds range, bearing, and signature of feature
// RowVector3f z
//
// void initVariables();
// void initVariables() {
//     u << 0.0, 0.0;
//     mu << 0.0, 0.0, 0.0;
//     z << 0.0, 0.0, 0.0;
// }

void Localizer::handleScans(const sensor_msgs::LaserScan::ConstPtr& msg) {
    minAngle = msg->angle_min;
    angleIncrement = msg->angle_increment;
    scans.resize(msg->ranges.size());
    for (int i = 0; i < msg->ranges.size(); i++) {
        scans[i] = msg->ranges[i];
    }
}

void Localizer::cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
	u(0) = msg->linear.x;
	u(1) = msg->angular.z;
}



// locate the feature from given LaserScan and update the feature vector z
void Localizer::findFeature() {
    // filter out unlikely cone returns


    // find center point of feature by taking minimum point of possible cone returns
    float angle = minAngle;
    float min[2] = {scans[0], minAngle};
    for (int i = 0; i < scans.size(); i++) {
        if (scans[i] == scans[i] && scans[i] < min) {
            min[0] = scans[i];
            min[1] = angle;
        }
        angle += angleIncrement;
    }
    z(0) = min[0] + coneRadii;
    z(1) = min[1];
}
