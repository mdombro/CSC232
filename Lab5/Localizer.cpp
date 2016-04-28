#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "Localizer.h"
#include <Eigen/Dense>
#include "point.h"
#include "nav_msgs/Odometry.h"

using namespace std;

Point toGlobal(float range, float bearing, Eigen::RowVector3f mu);
float distanceP(Point & A, Point & B);

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
    z.resize(6);
    for (int e = 0; e < 6; e++)
        z[e] << Mx[e], 0.0, e;
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
        if(z[i](0) != -1000) projMu = projMu + Kt[i]*((z[i]-zest[i]).transpose());
        else projMu = projMu;
        Eigen::Matrix3f I;
        I << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0;
        projSigma = (I-Kt[i]*Ht[i])*projSigma;
    }
    // if(z(0) != -1000) projMu = projMu + Kt[z(2)]*((z-zest[z(2)]).transpose());
    // else projMu = projMu;

    Eigen::Matrix3f I;
    I << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    // for (int r = 0; r < 6; r++) {
    //     projSigma = (I-Kt[r]*Ht[r])*projSigma;
    //     //if (z[r](0) != -1000)
    //     //else projSigma = projSigma;
    // }
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

// when listening to the commands from the PFC
void Localizer::cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
	u(0) = msg->linear.x;
	u(1) = msg->angular.z + 0.00001;
}

// listening to commands from the robot
// void Localizer::cmdUpdate(const nav_msgs::Odometry::ConstPtr& msg) {
// 	u(0) = msg->twist.twist.linear.x;
// 	u(1) = msg->twist.twist.angular.z + 0.00001;
// }

// locate the feature from given LaserScan and update the feature vector z
void Localizer::findFeature() {
    float beamAngle = minAngle;
    vector<Point> endbeams;
    float bx, by;
    for (int i = 0; i < scans.size(); i++) {
        if (scans[i] < 0.4 || scans[i] > 5.0) {beamAngle++; continue;}
        Point bend = toGlobal(scans[i], beamAngle, Localizer::mu);
        endbeams.push_back(bend);
        beamAngle += angleIncrement;
    }
    vector<vector<Point> > potentialBeams;
    potentialBeams.resize(6);
    for (int o = 0; o < endbeams.size(); o++) {
        for (int g = 0; g < 6; g++) {
            Point cone(Mx[g],My[g]);
            if (distanceP(endbeams[o], cone) < 0.15) {
                cout << "Correspondance: " << g << endl;
                potentialBeams[g].push_back(endbeams[o]);
            }
        }
    }
    vector<float> mins;
    vector<Point> minsP;
    mins.resize(6,0);
    minsP.resize(6);
    for (int h = 0; h < 6; h++) {
        if (potentialBeams[h].size() > 5) {
            Point Mu(Localizer::mu(0), Localizer::mu(1));
            mins[h] = distanceP(Mu, potentialBeams[h][0]);
            minsP[h] = potentialBeams[h][0];
            for (int r = 1; r < potentialBeams[h].size(); r++) {
                if (distanceP(Mu, potentialBeams[h][r]) < mins[h]) {
                    mins[h] = distanceP(Mu, potentialBeams[h][r]);
                    minsP[h] = potentialBeams[h][r];
                }
            }
        }
    }
    cout << "Seen Cones: ";
    for (int f = 0; f < 6; f++) {
        if (mins[f] != 0) {
            Point Mu(Localizer::mu(0), Localizer::mu(1));
            float range = distanceP(minsP[f], Mu) + coneRadii;
            float bearing = atan2(minsP[f].y - Localizer::mu(1), minsP[f].x - Localizer::mu(0)) - Localizer::mu(2);
            z[f](0) = range;
            z[f](1) = bearing;
            z[f](2) = f;
            cout << " " << z[f](2) << " " << range;
        }
        else {
            z[f](0) = -1000;
            z[f](1) = -1000;
            z[f](2) = f;
        }
    }
    cout << endl;

    // float beamAngle = minAngle;
    // vector<vector<float>> rangeS;
    // vector<vector<float>> bearingS;
    // float min[2] = {scans[0], beamAngle};
    // for (int i = 0; i < scans.size(); i++) {
	// for (int t = 0; t < 6; t++) {
	// 	if (abs(scans[i] - zest[o](0)) < 4*St[o](0,0)) {
	// 		rangeS[t].push_back(scans[i]);
	// 		bearingS[t].push_back(beamAngle);
	// 	}
	// }
    //     //if (scans[i] != scans[i]) {
    //     //    continue;
    //     //}
    //     //if (scans[i] < min[0]) {
    //     //    min[0] = scans[i];
    //     //    min[1] = beamAngle;
    //     //}
    //     beamAngle += angleIncrement;
    // }
	// vector<float> minR = ramgeS[0][0];
	// vector<float> minA = bearing
	// for (int u = 0; u < 6; u++) {
	// 	for (int g = 0; g < rangeS[u].size(); g++) {
	// 		if (rangeS[u][g] < min) {
	// 			minR = rangeS[u][g];
	// 			minA = bearingS[u][g];
	// 		}
	// 	}
	// }
    // if (min[0] > 2.5) {  // skip if past 2.5 meter threshold
    //     z(0) = -1000;
    //     z(1) = -1000;
    //     z(2) = -1000;
    // } else {
    //     int bestCorrelation = 0;
    //     int changed = 0;
    //     for (int o = 1; o < 6; o++) {
    //         // find closest in range and sanity check bearing
    //         if (abs(min[0]-zest[o](0)) <= abs(min[0]-zest[bestCorrelation](0)) ) {
    //             bestCorrelation = o; // hopefully the index of the closest cone
    //             changed = 1;
    //         }
    //     }
    //     if (abs(min[1]-zest[bestCorrelation](1)) < 4.2*sqrt(St[besfloat distanceP(Point & A, Point & B) tCorrelation](1,1))) {  // is the bearing as expected - filter out random objects
    //         z(0) = min[0]+0.1;
    //         z(1) = min[1];
    //         z(2) = bestCorrelation;
    //     }
    //     else {
    //         z(0) = -1000;
    //         z(1) = -1000;
    //         z(2) = -1000;
    //     }
    // }
    // cout << "Signature of detected cone: " << z(2) << endl;
}

Point toGlobal(float range, float bearing, Eigen::RowVector3f mu) {
    float Lx = range*cos(bearing);
    float Ly = range*sin(bearing);
    float Gx = Lx*cos(mu(2))-Ly*sin(mu(2))+mu(0);
    float Gy = Lx*sin(mu(2))+Ly*cos(mu(2))+mu(1);
    //cout << Gx << " " << Gy << endl;
    Point G(Gx, Gy);
    return G;
}


float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
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
