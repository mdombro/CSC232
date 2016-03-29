#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "localizer_cmdline.h"
#include <Eigen/Dense>

class Localizer {
    /*** Odometry command vector  ***/
    RowVector2f u;

    /*** LaserScan return information ***/
    float minAngle;
    float angleIncrement;
    vector<float> scans;

    /*** predetermined cone radius ***/
    // /double coneRadii;

    /*** EKF Matrix variables ***/
    // current robot position belief
    RowVector3f mu

    // holds range, bearing, and signature of feature
    RowVector3f z

public:
    void findFeature();
    void handleScans(const sensor_msgs::LaserScan::ConstPtr& msg)
    void cmdUpdate(const geometry_msgs::Twist::ConstPtr& msg)
}
