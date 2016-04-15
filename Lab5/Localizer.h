#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
//#include "localizer_cmdline.h"
#include <vector>
#include <Eigen/Dense>

class Localizer {
    /*** Odometry command vector  ***/
    Eigen::RowVector2f u;  // [lv, av]

    /*** LaserScan return information ***/
    float minAngle;
    float maxRange;
    float angleIncrement;
    float alpha;

    /*** predetermined cone radius ***/
    double coneRadii;

    /*** EKF Matrix variables ***/
    // current robot position belief
    Eigen::RowVector3f mu;
    std::vector<float> quaternion;

    // holds range, bearing, and signature of feature
    Eigen::RowVector3f z;

    // Estimated beam range, bearing, and signature
    Eigen::RowVector3f zest;

    // Covariance matrix for robot pos
    Eigen::Matrix3f sigma;

    // Covariance matrix of predicted beam returns
    Eigen::Matrix3f St;

    // Other EKF matrices
    Eigen::Matrix3f Gt;
    Eigen::MatrixXf Vt;
    Eigen::Matrix2f Mt;
    Eigen::Vector3f projMu;
    Eigen::Matrix3f projSigma;
    Eigen::Matrix3f Qt;
    Eigen::Matrix3f Ht;
    Eigen::Matrix3f Kt;


public:
    std::vector<float> scans;
    Localizer();
    void findFeature();
    void handleScans(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg);
    void setConeRadii(double radii);
    void setAlpha(float alphas);
    float getQuatx();
    float getQuaty();
    float getQuatz();
    float getQuatw();
    Eigen::Matrix3f getSigma();
    float getx();
    float gety();
    void EKF();
};
