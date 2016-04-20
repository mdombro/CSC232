#include "point.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using namespace ros;

void handle_odom( const nav_msgs::Odometry::ConstPtr& msg);
Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y, Point prev, Point next);
float distanceP(Point & A, Point & B);  // distance is in the std namespace
//Point findClosestPoint(vector<float> path_x, vector<float> path_y, Point mu);
Point findClosestPoint(Point A, Point B, Point mu);
//Point closestPoint(Point A, Point B, Point P);

float Wthresh = 0.1;
float maxAngularVelocity = 2;
float lookAheadDistance = 0.5;
float linearVelocity = 0.1;
float angularVelocity;
Point lookahead(0,0);
Point Mu(0,0);
float quaternion[4] = {0,0,0,1};

int main(int argc, char** argv) {
    init(argc, argv, "pfc");
    geometry_msgs::Twist msg;

    NodeHandle n;
	Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    Subscriber sub = n.subscribe("/odom",1000, handle_odom);
	ros::Rate loop_rate(10);

    vector<float> path_x;
    path_x.push_back(-0.1);
    path_x.push_back(1.0);
    path_x.push_back(2.0);
    vector<float> path_y;
    path_y.push_back(0.0);
    path_y.push_back(0.0);
    path_y.push_back(1.0);

    vector<int> passed(path_x.size(), 0);

    Point goal(path_x[2], path_y[2]);
    while (ros::ok()) {
        for (int i = 0; i < path_x.size(); i++) {
            Point waypoint(path_x[i],path_y[i]);
            if (distanceP(Mu, waypoint) < Wthresh) {
                passed[i] = 1;
            }
        }
        if ( distanceP(Mu, goal) < Wthresh ) {
            linearVelocity = 0;
            angularVelocity = 0;
            // request new path
        }
        else {
            Point prev(0,0);
            Point next(0,0);
            if (passed[0] == 0) {  // have not passed first waypoint
                prev.setx(path_x[0]);
                prev.sety(path_y[0]);
                next.setx(path_x[1]);
                next.sety(path_y[1]);
                // Point prev(path_x[0], path_y[0]);
                // Point next(path_x[1], path_y[1]);
            }
            else {  // find what segment robot is on
                for (int k = 0; k < passed.size()-1; k++) {
                    if (passed[k] == 1 && passed[k+1] == 0) {
                        prev.setx(path_x[k]);
                        prev.sety(path_y[k]);
                        next.setx(path_x[k+1]);
                        next.sety(path_y[k+1]);
                        //Point prev(path_x[k], path_y[k]);
                        //Point next(path_x[k+1], path_y[k+1]);
                    }
                }
            }
            //cout << prev.x << " " << prev.y << " " << next.x << " " << next.y << endl;
            //cout << Mu.x << " " << Mu.y << endl;
            lookahead = lookAheadPoint(Mu, path_x, path_y, prev, next);
            cout << lookahead.x << " " << lookahead.y << endl;
            tf::Quaternion q(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            //cout << yaw << endl;
            float theta = atan2(lookahead.y - Mu.y, lookahead.x - Mu.x) - yaw;
            //cout << theta << endl;
            float x_offset = lookAheadDistance*cos(theta);
            //cout << x_offset << endl;
            float lambda = (2*(x_offset)/pow(lookAheadDistance,2));
            angularVelocity = linearVelocity*lambda;
        }
        if (angularVelocity > maxAngularVelocity) {
            angularVelocity = maxAngularVelocity;
        }
        msg.linear.x = linearVelocity;
        msg.angular.z = angularVelocity;
        //cout << angularVelocity << endl;
        ros::spinOnce();
		pub.publish(msg);
		loop_rate.sleep();
		//count++;
	}


}

// listen to odometry for tuning
void handle_odom( const nav_msgs::Odometry::ConstPtr& msg) {
    // implement storing of robot pose here
    Mu.x = msg->pose.pose.position.x;
    Mu.y = msg->pose.pose.position.y;
    quaternion[0] = msg->pose.pose.orientation.w;    //.assign(0, msg->pose.pose.orientation.w);
    quaternion[1] = msg->pose.pose.orientation.x;    //.assign(1, msg->pose.pose.orientation.x);
    quaternion[2] = msg->pose.pose.orientation.y;      // .assign(2, msg->pose.pose.orientation.y);
    quaternion[3] = msg->pose.pose.orientation.z;    //.assign(3, msg->pose.pose.orientation.z);
    //cout << quaternion[0] << endl;
    //return;
}

Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y, Point prev, Point next) {
    // go over all segmesnt in current path_x
    // Use lookahead distance as the radius of a circle with the closest path point as the center
    // find the intersection points of each segment with this circle
    // test if one of the intersection points lies within the path segment, if so this is the lookahead point
    // Failure conditions:
    // - path segment larger than the circle drawn by lookahead distance
    //   Would need to test which point is forward of the robot
    // -
    Point goal(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    if (distanceP(mu, goal) < lookAheadDistance ){
        return Point(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    }
    else {
        Point closest = findClosestPoint(prev, next, mu);
        float x1,x2,y1,y2;
        for (int i = 0; i < path_x.size()-1; i++) {
            //cout << closest.x << " " << closest.y << endl;
            if (path_x[i] != path_x[i+1]) {
                float m = (path_y[i]-path_y[i+1])/(path_x[i]-path_x[i+1]);
                float c = path_y[i] - m*path_x[i];
                float A = pow(m,2)+1;
                float B = 2*(m*c - m*closest.y - closest.x);
                float C = pow(closest.y,2) - pow(lookAheadDistance, 2) + pow(closest.x,2) - 2.0*c*closest.y + pow(c,2);
                float Disc = pow(B, 2) - 4.0*A*C;
                if (Disc > 0.0) {
                    x1 = (-B + sqrt(pow(B, 2) - 4*A*C))/(2*A);
                    x2 = (-B - sqrt(pow(B, 2) - 4*A*C))/(2*A);
                    y1 = m*((-B + sqrt(pow(B, 2) - 4*A*C))/(2*A))+c;
                    y2 = m*((-B - sqrt(pow(B, 2) - 4*A*C))/(2*A))+c;
                }
                else {
                    x1 = -1000;
                    y1 = -1000;
                    x2 = -900;
                    y2 = -900;
                }
            }
            else {
                x1 = path_x[i];
                y1 = closest.y + sqrt(pow(lookAheadDistance,2) - pow(path_x[i]-closest.x,2));
                x2 = path_x[i];
                y2 = -y1;
            }
            Point candidate1(x1,y1);
            Point candidate2(x2,y2);
            Point a(path_x[i], path_y[i]);
            Point b(path_x[i+1], path_y[i+1]);
            if (distanceP(a,candidate1) + distanceP(candidate1,b) == distanceP(a,b)) {
                return candidate1;
            }
            else if (distanceP(a,candidate2) + distanceP(candidate2,b) == distanceP(a,b)) {
                return candidate2;
            }
        }
    }
}

float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
}

// Point findClosestPoint(vector<float> path_x, vector<float> path_y, Point mu) {
//     // if (mu.x < path_x[0]){
//     //     Point start(path_x[0], path_y[0]);
//     //     return start;
//     // }
//     // else {
//     Point prev(0,0);
//     Point next(0,0);
//     //Point a(0,0);
//     vector<Point> candidates;  // will hold the closes point from the robot to each segment
//     for (int i = 0; i < path_x.size()-1; i++) {
//         prev.setx(path_x[i]);
//         prev.sety(path_y[i]);
//         next.setx(path_x[i+1]);
//         next.sety(path_y[i+1]);
//         Point a = closestPoint(prev,next,mu);
//         if (distanceP(prev,a) + distanceP(a, next) == distanceP(prev,next) ) // point is on the segment
//             candidates.push_back(a);                                     // Otherwise reject the point
//     }
//     Point closest = candidates[0];
//     for (int f = 0; f < candidates.size(); f++) {
//         if ( (pow(candidates[f].x-mu.x,2)+pow(candidates[f].y-mu.y,2)) < (pow(closest.x-mu.x,2)+pow(closest.y-mu.y,2)) ) {
//             closest.setx(candidates[f].x);
//             closest.sety(candidates[f].y);
//         }
//     }
//     //Point closest = closestPoint(prev, next, mu);
//     return closest;
//     //}
// }


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
