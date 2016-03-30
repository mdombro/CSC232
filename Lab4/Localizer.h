#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "localizer_cmdline.h"
#include <vector>
#include <Eigen/Dense>

class Localizer {
    /*** Odometry command vector  ***/
    Eigen::RowVector2f u;

    /*** LaserScan return information ***/
    float minAngle;
    float angleIncrement;
    std::vector<float> scans;

    /*** predetermined cone radius ***/
    double coneRadii;

    /*** EKF Matrix variables ***/
    // current robot position belief
    Eigen::RowVector3f mu;

    // holds range, bearing, and signature of feature
    Eigen::RowVector3f z;

    // Estimated beam range, bearing, and signature
    Eigen::RowVector3f zest;

    // Covariance matrix for robot pos
    Eigen::Matrix3f sigma;

    // Covariance matrix of predicted beam returns
    Eigen::Matrix3f St;

public:
    Localizer();
    void findFeature();
    void handleScans(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg);
    void setConeRadii(double radii);
};
