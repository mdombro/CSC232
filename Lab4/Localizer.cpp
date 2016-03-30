#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "Localizer.h"
#include <Eigen/Dense>

// using namespace ros;
using namespace std;
// using namespace Eigen;
//using namespace Localizer;

Localizer::Localizer() {
        u << 0.0, 0.0;        // linear velocity, angular velocity
        mu << 0.0, 0.0, 0.0;  // x, y, z of robot
        z << 0.0, 0.0, 0.0;   // distance, bearing, signature
        sigma << 0, 0, 0,     // covariance of robot pos
                 0, 0, 0,
                 0, 0, 0;
        St << 0.01, 0, 0,    // covariance of beam returns - play with values
              0, 0.01, 0,
              0, 0, 0.01;
        zest << 1.0, 0.0, 0.0;  // predicted beam return at initialization
}

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
    // filtered scans array
    vector<float> filterScans;

    // hold the angles of respective potential cone returns
    vector<float> angles;

    // filter out unlikely cone returns
    // find if either bearing or range are significantly outside expectation
    for (int o = 0; o < scans.size(); o++) {
        if ( abs(z(0)-zest(0)) > 4.0*sqrt(St(0)) || abs(z(1)-zest(1)) > 4.0*sqrt(St(1)) ) {
            continue;
        }
        else {
            filterScans.push_back(scans[o]);
            angles.push_back(minAngle+(angleIncrement*o));  // calulate and add current angle
        }
    }

    // find center point of feature by taking minimum point of possible cone returns
    //float angle = minAngle;
    float min[2] = {filterScans[0], minAngle};
    for (int i = 0; i < filterScans.size(); i++) {
        if (filterScans[i] == filterScans[i] && filterScans[i] < min[0]) {
            min[0] = filterScans[i];
            min[1] = angles[i];
        }
        //angle += angleIncrement;
    }
    z(0) = min[0] + coneRadii;
    z(1) = min[1];
}

void Localizer::setConeRadii(double r) {
    coneRadii = r;
}
