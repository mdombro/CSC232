#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "actionlib_msgs/GoalStatus.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace ros;

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg);

int flag = 0;
int halt = 0;

int main(int argc, char** argv) {
    init(argc, argv, "exec");

    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path;
    actionlib_msgs::GoalStatus h;

    NodeHandle n;
    Publisher npath = n.advertise<geometry_msgs::Polygon>("next_path", 1000, true);
    Publisher halt = n.advertise<actionlib_msgs::GoalStatus>("stop_and_go", 1000);
    Subscriber sub = n.subscribe("next_pathFlag",1000, path_dispatch);
	ros::Rate loop_rate(10);

    vector<float> path_x;
    path_x.push_back(0.0);
    path_x.push_back(0.5);
    path_x.push_back(0.75);
    path_x.push_back(1.25);
    path_x.push_back(1.5);;
    vector<float> path_y;
    path_y.push_back(0.0);
    path_y.push_back(0.0);
    path_y.push_back(0.5);
    path_y.push_back(0.5);
    path_y.push_back(0.0);

    vector<float> path_x2;
    //path_x2.push_back(1.5);
    path_x2.push_back(1.75);
    path_x2.push_back(2.25);
    path_x2.push_back(2.5);
    vector<float> path_y2;
    //path_y2.push_back(0.0);
    path_y2.push_back(-0.5);
    path_y2.push_back(-0.5);
    path_y2.push_back(0.0);

    for (int i = 0; i < path_x.size(); i++) {
        p.x = path_x[i];
        p.y = path_y[i];
        path.points.push_back(p);
    }

    while (ros::ok()) {
        h.status = 0;
        if (flag && path.points.size() != 8) {
            flag = 0;
            for (int i = 0; i < path_x2.size(); i++) {
                p.x = path_x2[i];
                p.y = path_y2[i];
                path.points.push_back(p);
            }
        }
        if (flag && path.points.size() == 8) {
            h.status = 1;
        }
        ros::spinOnce();
        npath.publish(path);
        halt.publish(h);
		loop_rate.sleep();
    }

}

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
    if (msg->status == 3) {
        cout << "Requested" << endl;
        flag = 1;
    }
}
