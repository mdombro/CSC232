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
        mu << 0.0, 0.0, 0.0;  // x, y, theta of robot
        z << 0.0, 0.0, 0.0;   // distance, bearing, signature
        sigma << 0, 0, 0,     // covariance of robot pos
                 0, 0, 0,
                 0, 0, 0;
        St << 0.01, 0, 0,    // covariance of beam returns - play with values
              0, 0.01, 0,
              0, 0, 0.01;
        zest << 1.0, 0.0, 0.0;  // predicted beam return at initialization
        Gt << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
        Vt.resize(3,2);
        Vt << 0, 0,
              0, 0,
              0, 0;
        Mt << 0, 0,
              0, 0;
        projMu << 0, 0, 0;
        projSigma << 0, 0, 0;
        Qt << 0.000001, 0, 0,
              0, 0.000001, 0,
              0, 0, 0.000001;
        Ht << 0, 0, 0,
              0, 0, -1,
              0, 0, 0;
        Kt << 0, 0, 0,
              0, 0, 0,
              0, 0, 0;
        cout << mu << endl;
}

void Localizer::setAlpha(float alphas) {
    alpha = alphas;
}

void Localizer::EKF() {
    float theta = mu(2);
    float spr = u(0)/u(1);
    // Gt
    Gt(0,2) = (-spr*cos(theta))+(spr*cos(theta+u(1)*0.1));
    Gt(1,2) = (-spr*sin(theta))+(spr*sin(theta+u(1)*0.1));
    // Vt
    Vt(0,0) = (-sin(theta)+sin(theta+u(1)*0.1))/u(1);
    Vt(1,0) = (-cos(theta)+cos(theta+u(1)*0.1))/u(1);
    Vt(0,1) = ( (u(0)*(sin(theta)-sin(theta+u(1)*0.1)))/pow(u(1),2) ) + (u(0)*cos(theta+u(1)*0.1)*0.1)/u(1);
    Vt(1,1) = -( (u(0)*(cos(theta)-cos(theta+u(1)*0.1)))/pow(u(1),2) ) + (u(0)*sin(theta+u(1)*0.1)*0.1)/u(1);
    Vt(2,1) = 0.1;
    Mt(0,0) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    Mt(1,1) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    projMu(0) = mu(0) + (-spr*sin(theta))+(spr*sin(theta+u(1)*0.1));
    projMu(1) = mu(1) + (spr*sin(theta))-(spr*sin(theta+u(1)*0.1));
    projMu(2) = u(1)*0.1;
    projSigma = Gt*sigma*Gt.transpose() + Vt*Mt*Vt.transpose();
    float q = pow(1.0 - projMu(0), 2) + pow(-projMu(1), 2);
    zest(0) = sqrt(q);
    zest(1) = atan2(-projMu(1), 1.0-projMu(0)) - projMu(2);
    zest(2) = 0;

    Ht(0,0) = -(1.0-projMu(0))/sqrt(q);
    Ht(0,1) = -(-projMu(1))/sqrt(q);
    Ht(1,0) = (-projMu(1))/q;
    Ht(1,1) = -(1.0-projMu(0))/q;
    St = Ht*projSigma*Ht.transpose() + Qt;
    Kt = projSigma*Ht.transpose()*St.inverse();
    mu = projMu + Kt*(z-zest).transpose();
    Eigen::Matrix3f I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    sigma = (I-Kt*Ht)*projSigma;

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

float Localizer::getx() {
    return mu(0);
}

float Localizer::gety() {
    return mu(1);
}
