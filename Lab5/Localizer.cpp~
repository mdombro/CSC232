#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "Localizer.h"
#include <Eigen/Dense>

using namespace std;

Localizer::Localizer() {
    Mx.resize(6);
    Mx[0] = 1.0;
    Mx[1] = 2.0;
    Mx[2] = 3.0;
    Mx[3] = 5.0;
    Mx[4] = 6.0;
    Mx[5] = 7.0;
    My.resize(6);
    My[0] = 0;
    My[1] = 0;
    My[2] = 0;
    My[3] = 0;
    My[4] = 0;
    My[5] = 0;


    quaternion.resize(4);
    u << 0.00000000001, 0.00000000001;        // linear velocity, angular velocity
    mu << 0.0, 0.0, 0.0;  // x, y, theta of robot

    // distance, bearing, signature
    z << 0.0, 0.0, 0.0;
    St.resize(6);
    for (int k = 0; k < 6; k++) {
        St[k] << 0.1, 0.0, 0.0,    // covariance of beam returns - play with values
                 0.0, 0.1, 0.0,
                 0.0, 0.0, 0.1;
    }
    zest.resize(6);
    for (int k = 0; k < 6; k++)
        zest[k] << Mx[k], My[k], k;  // predicted beam return at initialization - range bearing signature
    Ht.resize(6);
    for (int k = 0; k < 6; k++) {
        Ht[k] << 0.0, 0.0, 0.0,
                 0.0, 0.0, -1.0,
                 0.0, 0.0, 0.0;
    }
    Kt.resize(6);
    for (int k = 0; k < 6; k++) {
        Kt[k] << 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0;
    }

    sigma << 0.0, 0.0, 0.0,     // covariance of robot pos
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0;
    Gt << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;

    Vt.resize(3,2);
    Vt << 0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0;

    Mt << 0.0, 0.0,
          0.0, 0.0;

    projMu << 0.0,
	          0.0,
	          0.0;
    projSigma << 0.0, 0.0, 0.0,
	     0.0, 0.0, 0.0,
	     0.0, 0.0, 0.0;
    Qt << 0.001, 0.0, 0.0,
          0.0, 0.001, 0.0,
          0.0, 0.0, 0.001;
}

void Localizer::setAlpha(float alphas) {
    alpha = alphas;
}

void Localizer::setUpdateRate(float freq) {
    dt = 1.0/freq;
}

void Localizer::EKF() {
    if (u(1) == 0.0) {
        u(1) = 0.0000001;
    }
    float theta = mu(2);

    float spr = u(0)/u(1);

    Gt(0,2) = (-spr*cos(theta))+(spr*cos(theta+u(1)*dt));
    Gt(1,2) = (-spr*sin(theta))+(spr*sin(theta+u(1)*dt));
    Vt(0,0) = (-sin(theta)+sin(theta+u(1)*dt))/u(1);
    Vt(1,0) = (cos(theta)-cos(theta+u(1)*dt))/u(1);
    Vt(0,1) = ( (u(0)*(sin(theta)-sin(theta+u(1)*dt)))/pow(u(1),2) ) + (u(0)*cos(theta+u(1)*dt)*dt)/u(1);
    Vt(1,1) = -( (u(0)*(cos(theta)-cos(theta+u(1)*dt)))/pow(u(1),2) ) + (u(0)*sin(theta+u(1)*dt)*dt)/u(1);
    Vt(2,1) = dt;
    Mt(0,0) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    Mt(1,1) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    projMu(0) = mu(0) + (-spr*sin(theta))+(spr*sin(theta+u(1)*dt));
    projMu(1) = mu(1) + (spr*cos(theta))-(spr*cos(theta+u(1)*dt));
    projMu(2) = mu(2) + u(1)*dt;
    if (projMu(2) > M_PI) {
        projMu(2) -= 2.0*M_PI;
    }
    else if (projMu(2) < -M_PI) {
        projMu(2) += 2.0*M_PI;
    }
    projSigma = Gt*sigma*Gt.transpose() + Vt*Mt*Vt.transpose();
    for (int i = 0; i < 6; i++) {
        float q = pow(Mx[i] - projMu(0), 2) + pow(My[i] - projMu(1), 2);
        zest[i](0) = sqrt(q);
        zest[i](1) = atan2(My[i] - projMu(1), Mx[i] - projMu(0)) - projMu(2);
        zest[i](2) = i;
        Ht[i](0,0) = -(Mx[i]-projMu(0))/sqrt(q);
        Ht[i](0,1) = -(My[i]-projMu(1))/sqrt(q);
        Ht[i](1,0) = (My[i]-projMu(1))/q;
        Ht[i](1,1) = -(Mx[i]-projMu(0))/q;
        St[i] = Ht[i]*projSigma*Ht[i].transpose() + Qt;
        Kt[i] = projSigma*Ht[i].transpose()*St[i].inverse();
    }
    if(z(0) != -1000) projMu = projMu + Kt[z(2)]*((z-zest[z(2)]).transpose());
    else projMu = projMu;

    Eigen::Matrix3f I;
    I << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    if (z(0) != -1000) projSigma = (I-Kt[z(2)]*Ht[z(2)])*projSigma;
    mu = projMu;
    sigma = projSigma;
    quaternion[0] = cos(mu(2)/2);
    quaternion[1] = 0;
    quaternion[2] = 0;
    quaternion[3] = sin(mu(2)/2);
}

void Localizer::handleScans(const sensor_msgs::LaserScan::ConstPtr& msg) {
    minAngle = msg->angle_min;
    maxRange = msg->range_max;
    angleIncrement = msg->angle_increment;
    scans.resize(msg->ranges.size());
    for (int i = 0; i < msg->ranges.size(); i++) {
        scans[i] = msg->ranges[i];
    }
}

void Localizer::cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
	u(0) = msg->linear.x;
	u(1) = msg->angular.z + 0.00001;
}

// locate the feature from given LaserScan and update the feature vector z
void Localizer::findFeature() {
    float beamAngle = minAngle;
    vector<vector<float>> rangeS;
    vector<vector<float>> bearingS;
    float min[2] = {scans[0], beamAngle};
    for (int i = 0; i < scans.size(); i++) {
	for (int t = 0; t < 6; t++) {
		if (abs(scans[i] - zest[o](0)) < 4*St[o](0,0)) {
			rangeS[t].push_back(scans[i]);
			bearingS[t].push_back(beamAngle);
		}
	}
        //if (scans[i] != scans[i]) {
        //    continue;
        //}
        //if (scans[i] < min[0]) {
        //    min[0] = scans[i];
        //    min[1] = beamAngle;
        //}
        beamAngle += angleIncrement;
    }
	vector<float> minR = ramgeS[0][0];
	vector<float> minA = bearing
	for (int u = 0; u < 6; u++) {
		for (int g = 0; g < rangeS[u].size(); g++) {
			if (rangeS[u][g] < min) {
				minR = rangeS[u][g];
				minA = bearingS[u][g];
			}
		}
	}
    if (min[0] > 2.5) {  // skip if past 2.5 meter threshold
        z(0) = -1000;
        z(1) = -1000;
        z(2) = -1000;
    } else {
        int bestCorrelation = 0;
        int changed = 0;
        for (int o = 1; o < 6; o++) {
            // find closest in range and sanity check bearing
            if (abs(min[0]-zest[o](0)) <= abs(min[0]-zest[bestCorrelation](0)) ) {
                bestCorrelation = o; // hopefully the index of the closest cone
                changed = 1;
            }
        }
        if (abs(min[1]-zest[bestCorrelation](1)) < 4.2*sqrt(St[bestCorrelation](1,1))) {  // is the bearing as expected - filter out random objects
            z(0) = min[0]+0.1;
            z(1) = min[1];
            z(2) = bestCorrelation;
        }
        else {
            z(0) = -1000;
            z(1) = -1000;
            z(2) = -1000;
        }
    }
    cout << "Signature of detected cone: " << z(2) << endl;
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

float Localizer::getQuatx() {
    return quaternion[1];
}

float Localizer::getQuaty() {
    return quaternion[2];
}

float Localizer::getQuatz() {
    return quaternion[3];
}

float Localizer::getQuatw() {
    return quaternion[0];
}

Eigen::Matrix3f Localizer::getSigma() {
    return sigma;
}
