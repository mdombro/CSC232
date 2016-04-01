// Matthew Dombroski
// ECE 232 Lab 3
// Computes odometry data and beam model data and publishes to the odometry and scan topics

#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "odomPub_cmdline.h"


using namespace ros;
using namespace std;

float command[2];  //to come from navi sub
float quaternion[4];
float pose[3] = {0}; // formatted as [x y yaw]

void yawToQuaternion(float yaw, float quaternion[]);
float sampleDistribution(float val);
void sampleMotionModel(float command[], float commandReal[], float pose[], float coeffs[], float timeInc);
void calcTrueDistance(float trueDistances[], int numBeams, float inc);
void calcNoisyDistance(float noisyDistances[], float trueDistances[], float sigmaHitm, int numBeams);

// Laser Scan variables
float sigmaHit;
float angleMin;
float angleMax;
float angleIncrement;
int numBeams;
float dCone;
//float dWall = 2.0;
float rCone = 0.1;

void cmmdUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
    command[0] = msg->linear.x;
    command[1] = msg->angular.z;
}

void reset(const std_msgs::Empty::ConstPtr& msg) {
    pose[0] = 0;
    pose[1] = 0;
    pose[2] = 0;
    quaternion[0] = 1;
    quaternion[1] = 0;
    quaternion[2] = 0;
    quaternion[3] = 0;
}

int main(int argc, char* argv[]) {
    gengetopt_args_info args;
    cmdline_parser(argc,argv,&args);
    float coeffs[6];
    coeffs[0] = args.a1_arg;
    coeffs[1] = args.a2_arg;
    coeffs[2] = args.a3_arg;
    coeffs[3] = args.a4_arg;
    coeffs[4] = args.a5_arg;
    coeffs[5] = args.a6_arg;
    sigmaHit = args.sigma_arg;
    angleMin = args.angleMin_arg;
    angleMax = args.angleMax_arg;
    angleMin *= (M_PI/180.0);
    angleMax *= (M_PI/180.0);
    angleIncrement = (angleMax-angleMin)/args.numBeams_arg;
    numBeams = args.numBeams_arg;
    float trueDistances[numBeams];

    float noisyDistances[numBeams];

    float commandReal[2];

    init(argc, argv, "Odom");
    nav_msgs::Odometry msg;
    sensor_msgs::LaserScan msg2;
    NodeHandle n;
    Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    Publisher las = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
    Subscriber sub = n.subscribe("navi", 1000, cmmdUpdate);  // update the command velocities
    Subscriber rst = n.subscribe("/mobile_base/commands/reset_odometry", 1000, reset);
    ros::Duration(1.3).sleep();
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        sampleMotionModel(command, commandReal, pose, coeffs, 0.1);
        yawToQuaternion(pose[2], quaternion);
        msg.pose.pose.position.x = pose[0];
        msg.pose.pose.position.y = pose[1];
        msg.pose.pose.orientation.w = quaternion[0];
        msg.pose.pose.orientation.x = quaternion[1];
        msg.pose.pose.orientation.y = quaternion[2];
        msg.pose.pose.orientation.z = quaternion[3];
        msg.twist.twist.linear.x = commandReal[0];
        msg.twist.twist.angular.z = commandReal[1];

        // Laser scanning section
        calcTrueDistance(trueDistances, numBeams, angleIncrement);
        calcNoisyDistance(noisyDistances, trueDistances, sigmaHit, numBeams);
        msg2.angle_max = angleMax;
        msg2.angle_min = angleMin;
        msg2.time_increment = 0;
        //angleIncrement = M_PI/numBeams;
        msg2.angle_increment = angleIncrement;
        msg2.scan_time = 0.1;
        msg2.range_min = 0;
        msg2.range_max = 2.5;
        msg2.ranges.resize(numBeams);
        for (int i = 0; i < numBeams; i++) {
            msg2.ranges[i] = noisyDistances[i];
        }
        ros::spinOnce();
        pub.publish(msg);
        las.publish(msg2);
        loop_rate.sleep();
    }
    return 0;
}

void yawToQuaternion(float yaw, float quaternion[]) {
    quaternion[0] = cos(yaw/2);
    quaternion[1] = 0;
    quaternion[2] = 0;
    quaternion[3] = sin(yaw/2);
}

float sampleDistribution(float val) {
    float sample = 0;
    val = sqrt(val);
    for (int i = 0; i < 12; i++) {
        sample += ((float)rand()/RAND_MAX)*2*val-val;
    }
    sample *= 0.5;
    return sample;
}

void calcTrueDistance(float trueDistances[], int numBeams, float inc) {
    float angle = angleMin + pose[2];  // find the starting angle beam
    dCone = sqrt( pow(pose[0]-1.0,2) + pow(pose[1],2 ) );
    float psi = 2*atan(rCone/dCone);   // angular diameter of the cone
    float phi = -atan(pose[1]/(pose[0]-1));  // bearing of feature from robot
    cout << phi+psi/2.0 << " " << phi-psi/2.0 << endl;
    for (int i = 0; i < numBeams; i++) {
        // if (angle < -atan(rCone/dCone) || angle > atan(rCone/dCone)) {
        //     trueDistances[i] = dWall/sin((M_PI/2) - angle);
        // }
        // if (angle > -atan(rCone/dCone) && angle < atan(rCone/dCone)) {
        //     float b = asin((dCone*sin(-angle))/rCone);
        //     float c = M_PI - angle - b;
        //     trueDistances[i] = (dCone*sin(c))/sin(b);
        // }

        //if ( (float)(phi+(psi/2.0)+angleMax) > pose[2] && (float)(phi-(psi/2.0)-angleMin) < pose[2] ) {
        if (angle < phi+psi/2.0 && angle > phi-psi/2.0 && dCone < 2.5) {
            float b = asin((dCone*sin(-angle))/rCone);
            float c = M_PI - angle - b;
            trueDistances[i] = (dCone*sin(c))/sin(b);
        }
        else {
            trueDistances[i] = 2.5;
        }
        angle += inc;
    }
}

void calcNoisyDistance(float noisyDistances[], float trueDistances[], float sigmaHitm, int numBeams) {
    for (int i = 0; i < numBeams; i++) {
        noisyDistances[i] = trueDistances[i] + sampleDistribution(pow(sigmaHit,2));
    }
}

void sampleMotionModel(float command[], float commandReal[], float pose[], float coeffs[], float timeInc) {
    // assuming command is [v, w]
    commandReal[0] = command[0]+sampleDistribution(coeffs[0]*pow(command[0],2)+coeffs[1]*pow(command[1],2));
    commandReal[1] = command[1]+sampleDistribution(coeffs[2]*pow(command[0],2)+coeffs[3]*pow(command[1],2));
    float gammaHat = sampleDistribution(coeffs[4]*pow(command[0],2)+coeffs[5]*pow(command[1],2));
    // assuming pose is [x y theta]
    float velRatio = commandReal[0]/commandReal[1];
    pose[2] = pose[2] + (commandReal[1]*timeInc)+gammaHat*timeInc;
    if (pose[2] > M_PI) {
        pose[2] -= 2*M_PI;
    }
    else if (pose[2] < -M_PI) {
        pose[2] += 2*M_PI;
    }
    if (commandReal[1] > 0.00000000000000000000001) {
        pose[0] = pose[0] - velRatio*sin(pose[2]) + (velRatio)*sin(pose[2]+commandReal[1]*timeInc);
        pose[1] = pose[1] + velRatio*cos(pose[2]) - (velRatio)*cos(pose[2]+commandReal[1]*timeInc);
    }
    else {
        // Applying L'Hopitals rule to the velocity equations as omega becomes close to 0
        pose[0] = pose[0] + commandReal[0]*(timeInc*cos(pose[2]+commandReal[1]*timeInc));
        pose[1] = pose[1] + commandReal[0]*(timeInc*sin(pose[2]+commandReal[1]*timeInc));
    }
}
