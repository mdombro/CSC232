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
        quaternion.resize(4);
        u << 0.00000000001, 0.00000000001;        // linear velocity, angular velocity
        mu << 0.0, 0.0, 0.0;  // x, y, theta of robot
        z << 0.0, 0.0, 0.0;   // distance, bearing, signature
        sigma << 0.0, 0.0, 0.0,     // covariance of robot pos
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0;
cout << sigma << endl;
        St << 0.1, 0.0, 0.0,    // covariance of beam returns - play with values
              0.0, 0.1, 0.0,
              0.0, 0.0, 0.1;
cout << St << endl;
        zest << 1.0, 0.0, 0.0;  // predicted beam return at initialization - range bearing signature
        cout << "zest: " << zest << endl;
        Gt << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;
cout << Gt << endl;
        Vt.resize(3,2);
        Vt << 0.0, 0.0,
              0.0, 0.0,
              0.0, 0.0;
cout << Vt << endl;
        Mt << 0.0, 0.0,
              0.0, 0.0;
cout << Mt << endl;
        projMu << 0.0,
		  0.0,
		  0.0;
        projSigma << 0.0, 0.0, 0.0,
		     0.0, 0.0, 0.0,
		     0.0, 0.0, 0.0;
        Qt << 0.001, 0.0, 0.0,
              0.0, 0.001, 0.0,
              0.0, 0.0, 0.001;
cout << Qt << endl;
        Ht << 0.0, 0.0, 0.0,
              0.0, 0.0, -1.0,
              0.0, 0.0, 0.0;
cout << Ht << endl;
        Kt << 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0;
cout << Kt << endl;
        cout << mu << endl;
}

void Localizer::setAlpha(float alphas) {
    alpha = alphas;
}

void Localizer::EKF() {
    float theta = mu(2);
    //cout << "Robot pose: " << mu(0) << " " << mu(1) << " " << mu(2) << endl;
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
    projMu(2) = u(1)*0.1;
    projSigma = Gt*sigma*Gt.transpose() + Vt*Mt*Vt.transpose();
    float q = pow(1.0 - projMu(0), 2) + pow(-projMu(1), 2);
    zest(0) = sqrt(q);
    zest(1) = atan2(-projMu(1), 1.0-projMu(0) - projMu(2));
    zest(2) = 0;
    //cout << "Estimated measurement: " << zest << "  Actual measurement: " << z << endl;
    Ht(0,0) = -(1.0-projMu(0))/sqrt(q);
    Ht(0,1) = -(-projMu(1))/sqrt(q);
    Ht(1,0) = (-projMu(1))/q;
    Ht(1,1) = -(1.0-projMu(0))/q;
    St = Ht*projSigma*Ht.transpose() + Qt;
    cout << sqrt(St(0,0)) << " " << sqrt(St(1,1)) << endl;
    Kt = projSigma*Ht.transpose()*St.inverse();
    if(z(0) != -1) mu = projMu + Kt*(z-zest).transpose();
    else mu = projMu;
    quaternion[0] = cos(mu(2)/2);
    quaternion[1] = 0;
    quaternion[2] = 0;
    quaternion[3] = sin(mu(2)/2);
    Eigen::Matrix3f I;
    I << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    sigma = (I-Kt*Ht)*projSigma;

}

void Localizer::handleScans(const sensor_msgs::LaserScan::ConstPtr& msg) {
    minAngle = msg->angle_min;
    maxRange = msg->range_max;
    angleIncrement = msg->angle_increment;
    scans.resize(msg->ranges.size());
    //cout << "orig scans: " << scans.size() << endl;
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
    float beamAngle = minAngle;
    // filter out unlikely cone returns
    // find if either bearing or range are significantly outside expectation
    //cout << "Scans size: " << scans.size() << endl;
    for (int o = 0; o < scans.size(); o++) {
        //cout << "Conditionals: " << Localizer::zest(0) << " " << scans[o]-zest(0) << " " << sqrt(St(0,0)) << endl;

        //if ( scans[o] < 1.5) { // 4.0*sqrt(St(0,0)) ) {//|| abs(beamAngle-zest(1)) > 4.0*sqrt(St(1,1)) ) {
        if (beamAngle < 4.0*sqrt(St(1,1))+zest(1) && beamAngle > -4.0*sqrt(St(1,1))+zest(1) && scans[o] < 4.0*sqrt(St(0,0))+zest(0) && scans[o] > -4.0*sqrt(St(0,0))+zest(0)) {
            filterScans.push_back(scans[o]);
            angles.push_back(minAngle+(angleIncrement*o));  // calulate and add current angle
        }
        beamAngle += angleIncrement;
    }
    cout << "Filter Scans size: " << filterScans.size() << endl;

    //cout << "scans: " << scans.size() << endl;
    //cout <<  "filter scans: " << filterScans.size() << endl;
    //cout << "angles: " << angles.size() << endl;


    // find center point of feature by taking minimum point of possible cone returns
    //float angle = minAngle;
    if (filterScans.size() != 0) {
        float min[2] = {filterScans[0], minAngle};

        /***** Circle fitting under test
        float sx = 0.0;
        float sy = 0.0;
        int q = 0;
        float delta;
        float epsilon = 0.0001;
        vector<float> pointx;
        vector<float> pointy;
        for (int g = 0; g < filterScans.size(); g++) {
            pointx.push_back(filterScans[g]*cos(angles[g]));
            pointy.push_back(filterScans[g]*sin(angles[g]));
        }
        cout << "Point x : " << pointx[0] << " Point y: " << pointy[0] << endl;
        for (int i = 1; i <= pointx.size()-2; i++) {
            for (int j = i+1; j <= pointx.size()-1; j++) {
                for (int k = j+1; k <= pointx.size(); k++) {
                    delta = (pointx[k]-pointx[j])*(pointy[j]-pointy[i])-(pointx[j]-pointx[i])*(pointy[k]-pointy[j]);
                    if (abs(delta) > epsilon) {
                        sx += ( (pointy[k]-pointy[j])*(pow(pointx[i],2)+pow(pointy[i],2))+(pointy[i]-pointy[k])*(pow(pointx[j],2)+pow(pointy[j],2))+(pointy[j]-pointy[i])*(pow(pointx[k],2)+pow(pointy[k],2)) )/(2*delta);
                        sy += -( (pointx[k]-pointx[j])*(pow(pointx[i],2)+pow(pointy[i],2))+(pointx[i]-pointx[k])*(pow(pointx[j],2)+pow(pointy[j],2))+(pointx[j]-pointx[i])*(pow(pointx[k],2)+pow(pointy[k],2)) )/(2*delta);
                        q++;
                    }
                }
            }
        }
        if (q == 0) {
            z(0) = z(0);
            z(1) = z(1);
        }
        cout << "sx: " << sx << " sy: " << sy << " delta: " << delta << endl;
        z(0) = sqrt(pow(sx/(float)q,2)+pow(sy/(float)q,2));
        z(1) = atan2(sy/(float)q, sx/(float)q);
        ********/

            for (int i = 0; i < filterScans.size(); i++) {
                if (filterScans[i] < min[0]) {
                    min[0] = filterScans[i];
                    min[1] = angles[i];
                }
                //angle += angleIncrement;
            }
            z(0) = min[0] + coneRadii;
            z(1) = min[1];
        }
    else {
        z(0) = -1;
        z(1) = -1;
    }
    filterScans.clear();
    angles.clear();
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
