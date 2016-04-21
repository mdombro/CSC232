#include "point.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "actionlib_msgs/GoalStatus.h"
#include "nav_msgs/Odometry.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "geometry_msgs/PoseWithCovariance.h"

using namespace std;
using namespace ros;

void handle_odom( const geometry_msgs::PoseWithCovariance::ConstPtr& msg);
Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y, float prev, float next);
float distanceP(Point & A, Point & B);  // distance is in the std namespace
//Point findClosestPoint(vector<float> path_x, vector<float> path_y, Point mu);
Point findClosestPoint(Point A, Point B, Point mu);
void handle_path(const geometry_msgs::Polygon::ConstPtr& msg);
void stopGo(const actionlib_msgs::GoalStatus::ConstPtr& msg);
//Point closestPoint(Point A, Point B, Point P);

float Wthresh = 0.1;
float maxAngularVelocity = M_PI;
float lookAheadDistance = 0.15;
float linearVelocity = 0.1;
float angularVelocity;
Point lookahead(0,0);
Point Mu(0,0);
float quaternion[4] = {0,0,0,1};
float discResolution = 40.0;
float lookAheadThresh = 0.05;

int currentWaypoint = 0;
int prevWaypoint = 0;

vector<float> path_x;
vector<float> path_y;
vector<int> passed;
int oldPathSize = 0;
int halt = 1;   // 1 is halt 0 is go - for holding the robot while getting path

int main(int argc, char** argv) {
    init(argc, argv, "pfc");
    geometry_msgs::Twist msg;
    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path_and_lookahead;
    geometry_msgs::Polygon next_path;
    actionlib_msgs::GoalStatus goal_stat;

    NodeHandle n;
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    Publisher path = n.advertise<geometry_msgs::Polygon>("path_and_lookahead", 1000);
    Publisher goals = n.advertise<actionlib_msgs::GoalStatus>("next_pathFlag" ,1000);
    Subscriber sub = n.subscribe("/pos",1000, handle_odom);
    Subscriber np = n.subscribe("next_path",1000, handle_path);
    Subscriber ready = n.subscribe("stop_and_go", 1000, stopGo);
	ros::Rate loop_rate(10);

    // while (path_x.size() == 0) {
    //     // pass
    // }

    path_x.resize(1,0);  // give it something so there are no errors on init
    path_y.resize(1,0);
    passed.resize(1, 0);

    //Duration(0.5).sleep();


    // vector<float> path_x;
    // path_x.push_back(0.0);
    // path_x.push_back(0.5);
    // path_x.push_back(0.75);
    // path_x.push_back(1.25);
    // path_x.push_back(1.5);;
    // vector<float> path_y;
    // path_y.push_back(0.0);
    // path_y.push_back(0.0);
    // path_y.push_back(0.5);
    // path_y.push_back(0.5);
    // path_y.push_back(0.0);


    //vector<int> passed(path_x.size(), 0);
    //passed[0] = 1;
    // ros::spinOnce();
    // ros::spinOnce();
    // ros::spinOnce();
    // ros::spinOnce();
    // ros::spinOnce();
    // ros::spinOnce();
    // Point goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    // int currentWaypoint = 1;
    // int prevWaypoint = 0;
    //Point currentWaypoint(path_x[1], path_y[1]);
    //Point prevWaypoint(path_x[0], path_y[0]);


    while (ros::ok()) {
        //cout << path_x.size() << endl;
        // while (path_x.size() == 1) {
        //     // pass
        // }
        //cout << path_x.size() << endl;
        Point goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);

        cout << prevWaypoint << " " << currentWaypoint << endl;
        goal_stat.status = 1;
        cout << "request: " << currentWaypoint << " " << path_x.size() << endl;
        if (currentWaypoint == path_x.size()) {
            cout << "new path" << endl;
            // set some flag to start computing path
            goal_stat.status = 3;
        }
        // if (currentWaypoint == path_x.size()) {
        //     //currentWaypoint = 1;
        //     //prevWaypoint = 0;
        //     passed.resize(path_x.size(), 0);
        //     //for (int w = 0; w < passed.resize()
        //     //linearVelocity = 0;
        //     //angularVelocity = 0;
        // }

        tf::Quaternion q(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //float theta = (atan2(lookahead.y - Mu.y, lookahead.x - Mu.x) - yaw);

        Point waypoint(path_x[currentWaypoint], path_y[currentWaypoint]);
        if (distanceP(Mu, waypoint) < Wthresh) {
            passed[currentWaypoint] = 1;
            prevWaypoint = currentWaypoint;
            currentWaypoint += 1;
        }
        lookahead = lookAheadPoint(Mu, path_x, path_y, prevWaypoint, currentWaypoint);
        //cout << "Lookahead: " << lookahead.x << " " << lookahead.y << endl;
        //cout << yaw << endl;
        //cout << "Pos: " << Mu.x << " " << Mu.y << " " << yaw << endl;

        //cout << theta << endl;

        float y_offset = (lookahead.x-Mu.x)*sin(-yaw)+(lookahead.y-Mu.y)*cos(-yaw);
        //float y_offset = lookAheadDistance*sin(theta);


        //cout << "Y offset: " << y_offset << endl;
        float lambda = (2*(y_offset))/pow(lookAheadDistance,2);
        angularVelocity = linearVelocity*lambda;
        //}
        if (angularVelocity > maxAngularVelocity) {
            angularVelocity = maxAngularVelocity;
        }

        //cout << "yaw: " << yaw << " theta: " << theta << endl;
        // if (yaw-theta > M_PI/4) {
        //     linearVelocity = 0;
        //     angularVelocity = 1.0;
        // }
        // else if (yaw-theta > -M_PI/4) {
        //     linearVelocity = 0;
        //     angularVelocity = 1.0;
        // }
        // else {
        //     linearVelocity = 0.1;
        // }

        path_and_lookahead.points.clear();
        p.x = lookahead.x;
        p.y = lookahead.y;
        p.z = 0;
        path_and_lookahead.points.push_back(p);
        for (int g = 0; g < path_x.size(); g++) {
            p.x = path_x[g];
            p.y = path_y[g];
            p.z = 0;
            path_and_lookahead.points.push_back(p);
        }
        float linVel, angVel;
        if (halt == 1) {linVel = 0; angVel = 0; currentWaypoint = 0; prevWaypoint = 0;}
        else {linVel = linearVelocity; angVel = angularVelocity;}
        if (path_x.size() == 1) {linVel = 0; angVel = 0; currentWaypoint = 0; prevWaypoint = 0;}
        else {linVel = linearVelocity; angVel = angularVelocity;}
        msg.linear.x = linVel;
        msg.angular.z = angVel;
        //cout << angularVelocity << endl;
        ros::spinOnce();
		pub.publish(msg);
        path.publish(path_and_lookahead);
        goals.publish(goal_stat);
		loop_rate.sleep();
		//count++;
	}
}

void stopGo(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
    halt = msg->status; // 1 is halt, 0 is go
}

void handle_path(const geometry_msgs::Polygon::ConstPtr& msg) {
    oldPathSize = msg->points.size();
    //if (msg->points.size() != oldPathSize) {
        path_x.resize(msg->points.size());
        path_y.resize(msg->points.size());
        for (int i = 0; i < msg->points.size(); i++) {
            path_x[i] = msg->points[i].x;
            path_y[i] = msg->points[i].y;
        }
        passed.resize(msg->points.size(), 0);

    //}
    //cout << oldPathSize << endl;
}

// listen to odometry for tuning
void handle_odom( const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    // implement storing of robot pose here
    Mu.x = msg->pose.position.x;
    Mu.y = msg->pose.position.y;
    quaternion[0] = msg->pose.orientation.w;    //.assign(0, msg->pose.pose.orientation.w);
    quaternion[1] = msg->pose.orientation.x;    //.assign(1, msg->pose.pose.orientation.x);
    quaternion[2] = msg->pose.orientation.y;      // .assign(2, msg->pose.pose.orientation.y);
    quaternion[3] = msg->pose.orientation.z;    //.assign(3, msg->pose.pose.orientation.z);
    //cout << quaternion[0] << endl;
    //return;
}

Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y, float prev, float next) {
    Point goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    Point p(path_x[prev], path_y[prev]);
    Point n(path_x[next], path_y[next]);
    Point closest = findClosestPoint(p, n, mu);
    //cout << closest.x << " " << closest.y << endl;
    if (distanceP(mu, goal) < lookAheadDistance ){
        return Point(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    }

    if (distanceP(mu, closest) > lookAheadDistance) {
        return closest;
    }

    vector<Point> discPath;
    float xd, yd, xInc, yInc;
    Point start(0,0);
    for (int i = prev; i < path_x.size()-1; i++) {
        float nextWp = i + 1; // keep next one waypoint ahead of prev
        if (i == prev) {
            start.setx(closest.x);
            start.sety(closest.y);
        }
        else {
            start.setx(path_x[i]);
            start.sety(path_y[i]);
        }
        xd = path_x[nextWp] - start.x;
        yd = path_y[nextWp] - start.y;
        //cout << yd << endl;
        xInc = xd/discResolution;
        yInc = yd/discResolution;
        for (int g = 0; g < (int)discResolution; g++) {
            discPath.push_back(Point(start.x+g*xInc, start.y+g*yInc));
        }
    }

    for (int h = 0; h < discPath.size(); h++) {
        //cout << abs(distanceP(closest, discPath[h])) - lookAheadDistance << endl;
        if (abs(distanceP(closest, discPath[h]) - lookAheadDistance) < lookAheadThresh ) {
            return discPath[h];
        }
    }
}

float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
}


Point findClosestPoint(Point A, Point B, Point P) {
    vector<float> a_to_p;
    a_to_p.push_back(P.x - A.x);
    a_to_p.push_back(P.y - A.y);

    vector<float> a_to_b;
    a_to_b.push_back(B.x - A.x);
    a_to_b.push_back(B.y - A.y);

    float atb2 = pow(a_to_b[0],2)+pow(a_to_b[1],2);

    float atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1];

    float t = atp_dot_atb/atb2;

    Point closest(A.x+a_to_b[0]*t, A.y+a_to_b[1]*t);
    return closest;
}
