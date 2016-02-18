#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "odomPub_cmdline.h"


using namespace ros;
using namespace std;

float command[2];  //to come from navi sub

void yawToQuaternion(float yaw, float quaternion[]);
float sampleDistribution(float val);
void sampleMotionModel(float command[], float commandReal[], float pose[], float coeffs[], float timeInc);

void cmmdUpdate(const geometry_msgs::Twist::ConstPtr& msg) {
    command[0] = msg->linear.x;
    command[1] = msg->angular.z;
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

    float quaternion[4];
    float pose[3] = {0}; // formatted as [x y yaw]
    float commandReal[2];

    init(argc, argv, "Odom");
    nav_msgs::Odometry msg;
    NodeHandle n;
    Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    Subscriber sub = n.subscribe("navi", 1000, cmmdUpdate);  // update the command velocities
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
        ros::spinOnce();
        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}

void yawToQuaternion(float yaw, float quaternion[]) {
    float r11 = cos(yaw);
    float r12 = -sin(yaw);
    float r21 = sin(yaw);
    float r22 = cos(yaw);
    // quaternion[0] = 0.5*sqrt(r11+r22+2); // normalizer
    // quaternion[1] = 0.5*sqrt(r11-r22);
    // quaternion[2] = 0.5*sqrt(r22-r11);
    // if (r21-r12 >= 0) {
    //     quaternion[3] = 0.5*sqrt(-r11-r22+2);
    // }
    // else {
    //     quaternion[3] = -0.5*sqrt(-r11-r22+2);
    // }
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

void sampleMotionModel(float command[], float commandReal[], float pose[], float coeffs[], float timeInc) {
    // assuming command is [v, w]
    commandReal[0] = command[0]+sampleDistribution(coeffs[0]*pow(command[0],2)+coeffs[1]*pow(command[1],2));
    commandReal[1] = command[1]+sampleDistribution(coeffs[2]*pow(command[0],2)+coeffs[3]*pow(command[1],2));
    float gammaHat = sampleDistribution(coeffs[4]*pow(command[0],2)+coeffs[5]*pow(command[1],2));
    // assuming pose is [x y theta]
    float velRatio = commandReal[0]/commandReal[1];
    cout << commandReal[1] << endl;
    if (commandReal[1] > 0.00000000000000000000001) {
        pose[0] = pose[0] - velRatio*sin(pose[2]) + (velRatio)*sin(pose[2]+commandReal[1]*timeInc);
        pose[1] = pose[1] + velRatio*cos(pose[2]) - (velRatio)*cos(pose[2]+commandReal[1]*timeInc);
        pose[2] = pose[2] + (commandReal[1]*timeInc)+gammaHat*timeInc;
    }
    else {
        // Applying L'Hopitals rule to the velocity equations as omega becomes close to 0
        pose[0] = pose[0] + commandReal[0]*(timeInc*cos(pose[2]+commandReal[1]*timeInc));
        pose[1] = pose[1] + commandReal[0]*(timeInc*sin(pose[2]+commandReal[1]*timeInc));
        pose[2] = pose[2] + (commandReal[1]*timeInc)+gammaHat*timeInc;
    }
}
