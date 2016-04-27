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
    // update rate
    float dt;

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
    std::vector<Eigen::RowVector3f> z;
    //Eigen::RowVector3f z;

    // Estimated beam range, bearing, and signature
    std::vector<Eigen::RowVector3f> zest;

    // Covariance matrix for robot pos
    Eigen::Matrix3f sigma;

    // Covariance matrix of predicted beam returns
    std::vector<Eigen::Matrix3f> St;

    // Other EKF matrices
    Eigen::Matrix3f Gt;
    Eigen::MatrixXf Vt;
    Eigen::Matrix2f Mt;
    Eigen::Vector3f projMu;
    Eigen::Matrix3f projSigma;
    Eigen::Matrix3f Qt;
    std::vector<Eigen::Matrix3f> Ht;
    std::vector<Eigen::Matrix3f> Kt;

    // Map from the executive
    std::vector<float> Mx;
    std::vector<float> My;


public:
    std::vector<float> scans;
    Localizer();
    void findFeature();
    void handleScans(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg);
    void setConeRadii(double radii);
    void setAlpha(float alphas);
    void setUpdateRate(float freq);
    float getQuatx();
    float getQuaty();
    float getQuatz();
    float getQuatw();
    Eigen::Matrix3f getSigma();
    float getx();
    float gety();
    void EKF();
};
