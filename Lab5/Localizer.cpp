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

    z.resize(6);
    for (int k = 0; k < 6; k++)
        z[k] << 0.0, 0.0, k;   // distance, bearing, signature
    St.resize(6);
    for (int k = 0; k < 6; k++) {
        St[k] << 0.2, 0.0, 0.0,    // covariance of beam returns - play with values
                 0.0, 0.2, 0.0,
                 0.0, 0.0, 0.2;
    }
    zest.resize(6);
    for (int k = 0; k < 6; k++)
        zest[k] << Mx[k], My[k], 0.0;  // predicted beam return at initialization - range bearing signature
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

void Localizer::EKF() {

    if (u(1) == 0.0) {
        u(1) = 0.0000001;
    }
    float theta = mu(2);

    float spr = u(0)/u(1);

    // Gt
    Gt(0,2) = (-spr*cos(theta))+(spr*cos(theta+u(1)*0.1));
    Gt(1,2) = (-spr*sin(theta))+(spr*sin(theta+u(1)*0.1));
    // Vt
    Vt(0,0) = (-sin(theta)+sin(theta+u(1)*0.1))/u(1);
    Vt(1,0) = (cos(theta)-cos(theta+u(1)*0.1))/u(1);
    Vt(0,1) = ( (u(0)*(sin(theta)-sin(theta+u(1)*0.1)))/pow(u(1),2) ) + (u(0)*cos(theta+u(1)*0.1)*0.1)/u(1);
    Vt(1,1) = -( (u(0)*(cos(theta)-cos(theta+u(1)*0.1)))/pow(u(1),2) ) + (u(0)*sin(theta+u(1)*0.1)*0.1)/u(1);
    Vt(2,1) = 0.1;
    Mt(0,0) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    Mt(1,1) = alpha*pow(u(0),2); + alpha*pow(u(1),2);
    projMu(0) = mu(0) + (-spr*sin(theta))+(spr*sin(theta+u(1)*0.1));
    projMu(1) = mu(1) + (spr*cos(theta))-(spr*cos(theta+u(1)*0.1));
    projMu(2) = mu(2) + u(1)*0.1;
    projSigma = Gt*sigma*Gt.transpose() + Vt*Mt*Vt.transpose();
    for (int i = 0; i < 6; i++) {
        if (z[i](0) != -1000) {
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
            //if(z[i](0) != -1000) projMu = projMu + Kt[i]*((z[i]-zest[i]).transpose());
            //else projMu = projMu;
            projMu = projMu + Kt[i]*((z[i]-zest[i]).transpose());
            quaternion[0] = cos(mu(2)/2);
            quaternion[1] = 0;
            quaternion[2] = 0;
            quaternion[3] = sin(mu(2)/2);
            Eigen::Matrix3f I;
            I << 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;
            projSigma = (I-Kt[i]*Ht[i])*projSigma;
        }
    }
    mu = projMu;
    sigma = projSigma;
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
	u(1) = msg->angular.z;
}

// locate the feature from given LaserScan and update the feature vector z
void Localizer::findFeature() {
    // filtered scans array
    vector<vector<float> > filterScans(6);

    // hold the angles of respective potential cone returns
    vector<vector<float> > angles(6);
    float beamAngle = minAngle;
    // filter out unlikely cone returns
    // find if either bearing or range are significantly outside expectation
    cout << St[2] << " " << zest[2][0] << endl;
    for (int i = 0; i < 6; i++) {
        for (int o = 0; o < scans.size(); o++) {
            if (beamAngle < 3.5*sqrt(St[i](1,1))+zest[i](1) && beamAngle > -3.5*sqrt(St[i](1,1))+zest[i](1) && scans[o] < 4.0*sqrt(St[i](0,0))+zest[i](0) && scans[o] > -4.0*sqrt(St[i](0,0))+zest[i](0)) {
            //if (scans[o] < 4.0*sqrt(St[i](0,0))+zest[i](0) && scans[o] > -4.0*sqrt(St[i](0,0))+zest[i](0)) {
                if (i == 2) {cout << scans[o] << " " << beamAngle << endl;}
                filterScans[i].push_back(scans[o]);
                angles[i].push_back(minAngle+(angleIncrement*o));  // calulate and add current angle
            }
            beamAngle += angleIncrement;
        }
        beamAngle = minAngle;
    }
    cout << "Filtered Scans: ";
    for (int j = 0; j < 6; j++) {
        cout << filterScans[j].size() << " ";
    }
    cout << endl;

    // find center point of feature by taking minimum point of possible cone returns
    //float angle = minAngle;
    float min[2] = {0,0};
    for (int g = 0; g < 6; g++) {
        if (filterScans[g].size() != 0) {
            min[0] = filterScans[g][0];
            min[1] = minAngle;
            for (int i = 0; i < filterScans[g].size(); i++) {
                if (filterScans[g][i] < min[0]) {
                    min[0] = filterScans[g][i];
                    min[1] = angles[g][i];
                }
            }
            z[g](0) = min[0] + coneRadii;
            z[g](1) = min[1];
            z[g](2) = g;
        }
        else {
            z[g](0) = -1000;
            z[g](1) = -1000;
            z[g](2) = g;
        }
    }
    //cout << "Estimated cone: " << z(0) << " " << z(1) << endl;
    for (int h = 0; h < 6; h++) { filterScans[h].clear(); angles[h].clear(); }
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
    return quaternion[0];
}

float Localizer::getQuaty() {
    return quaternion[1];
}

float Localizer::getQuatz() {
    return quaternion[2];
}

float Localizer::getQuatw() {
    return quaternion[3];
}

Eigen::Matrix3f Localizer::getSigma() {
    return sigma;
}