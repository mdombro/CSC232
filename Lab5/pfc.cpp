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
Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y);
float distanceP(Point & A, Point & B);  // distance is in the std namespace
Point findClosestPoint(Point A, Point B, Point mu);
void handle_path(const geometry_msgs::Polygon::ConstPtr& msg);
void stopGo(const actionlib_msgs::GoalStatus::ConstPtr& msg);
float runPurePursuit();

float Wthresh = 0.1;
float maxAngularVelocity = M_PI;
float lookAheadDistance = 0.15;
float linearVelocity = 0.1;
float linVel;
float angularVelocity;
Point lookahead(0,0);
Point Mu(0,0);
float quaternion[4] = {0,0,0,1};
float discResolution = 40.0;
float lookAheadThresh = 0.05;
bool gotPath = false;
geometry_msgs::Polygon oldMsg;

int segment = 1;

int currentWaypoint = 1;
int prevWaypoint = 0;

vector<float> oldPath_x;
vector<float> path_x;
vector<float> path_y;
vector<int> passed;
int halt = 1;   // 1 is halt 0 is go - for holding the robot while getting path

int main(int argc, char** argv) {
    init(argc, argv, "pfc");
    geometry_msgs::Twist cmd;
    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path_and_lookahead;
    geometry_msgs::Polygon next_path;
    actionlib_msgs::GoalStatus goal_stat;
    //geometry_msgs::Polygon oldMsg;

    //oldPath_x.push_back(0.0);
    //path_x.push_back(0.0);
    //path_y.push_back(0.0);
    //path_x.push_back(0.0);
    //path_y.push_back(0.0);

    NodeHandle n;
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    Publisher path = n.advertise<geometry_msgs::Polygon>("path_and_lookahead", 1000);
    Publisher goals = n.advertise<actionlib_msgs::GoalStatus>("plannerFlag" ,1000);
    Subscriber sub = n.subscribe("/pos",1000, handle_odom);
    Subscriber np = n.subscribe("next_path",100, handle_path);
    //Subscriber ready = n.subscribe("stop_and_go", 1000, stopGo);
	ros::Rate loop_rate(20);

    // path_x.resize(1,0);  // give it something so there are no errors on init
    // path_y.resize(1,0);
    // passed.resize(1, 0);

    while (ros::ok()) {
        goal_stat.status = 1;
        p.x = Mu.x;
        p.y = Mu.y;
        path_and_lookahead.points.push_back(p);
        //cout << "Path size: " << path_x.size() << endl;
        //cout << "Path dif: " <<  oldPath_x[oldPath_x.size()-1] << " " << path_x[path_x.size()-1] << endl;
        //if (oldPath_x[oldPath_x.size()-1] != path_x[path_x.size()-1])
        //    gotPath = true;
        //else
        //    gotPath = false
        gotPath = false;

        //cout << "WPs: " << prevWaypoint << " " << currentWaypoint << endl;
        if (path_x.size() != 0) {
            Point waypoint(path_x[segment], path_y[segment]);
            if (distanceP(Mu, waypoint) < Wthresh) {
                segment++;
            }
            angularVelocity = runPurePursuit();
            linVel = linearVelocity;
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
        }
        // else {
        //     //cout << "Are we in this" << endl;
        //     angularVelocity = 0;
        //     linVel = 0;
        //     goal_stat.status = 3;
        //     gotPath = false;
        // }
        // if (segment == path_x.size()) {
        //     cout << "new path" << endl;
        //     // set some flag to start computing path
        //     goal_stat.status = 3;
        //     path_x.clear();
        //     path_y.clear();
        //     passed.clear();
        //     segment = 0;
        //     //prevWaypoint = 0;
        //     //currentWaypoint = 1;
        // }
        //Point waypoint(path_x[currentWaypoint], path_y[currentWaypoint]);
        if (path_x.size() != 0) {

                //passed[currentWaypoint] = 1;
                //prevWaypoint = currentWaypoint;
                //currentWaypoint += 1;
            //}
            Point Goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);
            if (distanceP(Mu, Goal) < Wthresh) {
                linVel = 0;
                angularVelocity = 0;
            }
        }
        //if (halt == 1) {linVel = 0; angVel = 0; currentWaypoint = 0; prevWaypoint = 0;}
        //else {linVel = linearVelocity; angVel = angularVelocity;}
        //if (path_x.size() == 1) {linVel = 0; angVel = 0; currentWaypoint = 0; prevWaypoint = 0;}
        //else {linVel = linearVelocity; angVel = angularVelocity;}
        //oldPath_x = path_x;
        cmd.linear.x = linVel;
        cmd.angular.z = angularVelocity;
        //cout << "Goal req: " << goal_stat.status << endl;
        ros::spinOnce();
		pub.publish(cmd);
        path.publish(path_and_lookahead);
        goals.publish(goal_stat);
		loop_rate.sleep();
	}
}

float runPurePursuit() {
    float angVelocity;
    tf::Quaternion q(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    lookahead = lookAheadPoint(Mu, path_x, path_y);

    float y_offset = (lookahead.x-Mu.x)*sin(-yaw)+(lookahead.y-Mu.y)*cos(-yaw);

    float lambda = (2*(y_offset))/pow(lookAheadDistance,2);
    angVelocity = linearVelocity*lambda;
    if (angVelocity > maxAngularVelocity) {
        angVelocity = maxAngularVelocity;
    }
    return angVelocity;
}

// void stopGo(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
//     halt = msg->status; // 1 is halt, 0 is go
// }

void handle_path(const geometry_msgs::Polygon::ConstPtr& msg) {
    //if (msg->points[msg->points.size()-1].x != oldMsg.points[oldMsg.points.size()-1].x && msg->points[msg->points.size()-1].y != oldMsg.points[oldMsg.points.size()-1].y) {
    //if (msg->points.size() != 0) {
    ///if () {
        path_x.clear();
        path_y.clear();
        for (int i = 0; i < msg->points.size(); i++) {
            path_x.push_back(msg->points[i].x);
            path_y.push_back(msg->points[i].y);
        }
        gotPath = true;
    //}
        //prevWaypoint = 0;
        //currentWaypoint = 1; // no real need to go to the first waypoint
        //passed.resize(msg->points.size(), 0);
        //passed[0] = 1;  // not exactly neccesary
                        // Ideally
    //}
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

Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y) {
    Point goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    //cout << "Goal: " << goal.x << " " << goal.y << endl;
    Point p(path_x[segment-1], path_y[segment-1]);
    Point n(path_x[segment], path_y[segment]);
    cout << "P: " << p.x << " " << p.y << " N: " << n.x << " " << n.y << endl;
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
    for (int i = segment-1; i < path_x.size()-1; i++) {
        float nextWp = i + 1; // keep next one waypoint ahead of prev
        if (i == segment-1) {
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
            //cout << "lookahead: " << discPath[h].x << " " << discPath[h].y << endl;
            return discPath[h];
        }
    }
}

float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
}


Point findClosestPoint(Point A, Point B, Point P) {
    if (A.x == B.x && A.y == B.y) {
        return Point(A.x, P.x);
    }
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
