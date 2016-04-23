#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point.h"
#include "actionlib_msgs/GoalStatus.h"
#include <iostream>
#include <ros/ros.h>
#include "point.h"

using namespace std;
using namespace ros;

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg);

int flag = 0;
int halt = 0;

int main(int argc, char** argv) {
    init(argc, argv, "exec");

    geometry_msgs::Point goal;
    //geometry_msgs::Polygon goal;
    actionlib_msgs::GoalStatus h;

    NodeHandle n;
    Publisher npath = n.advertise<geometry_msgs::Point>("next_goal", 1000, true);
    //Publisher halt = n.advertise<actionlib_msgs::GoalStatus>("stop_and_go", 1000);
    Publisher reqPath = n.advertise<actionlib_msgs::GoalStatus>("plannerFlag",1000, true);
    Subscriber reqGoal = n.subscribe("next_pathFlag", 1000, path_dispatch);
	ros::Rate loop_rate(1);

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
    //
    // vector<float> path_x2;
    // path_x2.push_back(1.5);
    // path_x2.push_back(1.75);
    // path_x2.push_back(2.25);
    // path_x2.push_back(2.5);
    // vector<float> path_y2;
    // path_y2.push_back(0.0);
    // path_y2.push_back(-0.5);
    // path_y2.push_back(-0.5);
    // path_y2.push_back(0.0);
    //
    // vector<vector<float> > py;
    // py.push_back(path_y2);
    // py.push_back(path_y);
    //
    //
    // vector<vector<float> > px;
    // px.push_back(path_x2);
    // px.push_back(path_x);

    vector<Point> goals;
    goals.push_back(Point(0.5, 0.0));
    goals.push_back(Point(1.5, 0.0));
    goals.push_back(Point(2.5, 0.0));
    goals.push_back(Point(4.0, 0.0));
    goals.push_back(Point(5.5, 0.0));
    goals.push_back(Point(6.5, 0.0));
    goals.push_back(Point(7.5, 0.0));
    goals.push_back(Point(6.5, 0.0));
    goals.push_back(Point(5.5, 0.0));
    goals.push_back(Point(4.0, 0.0));
    goals.push_back(Point(2.5, 0.0));
    goals.push_back(Point(1.5, 0.0));

    // for (int i = 0; i < path_x.size(); i++) {
    //     p.x = path_x[i];
    //     p.y = path_y[i];
    //     path.points.push_back(p);
    // }

    while (ros::ok()) {
        // if (goal == 13) {
        //     path.points.clear();
        //     flag = 0;
        // }
        h.status = 1;
        if (flag && !goals.empty()) {
            cout << "Goal release: " << endl;
            goal.x = goals.back().x;
            goal.y = goals.back().y;
            goals.pop_back();
            goals.pop_back();
            h.status = 3;
            flag = 0;
            // vector<float> pxt = px.back();
            // vector<float> pyt = py.back();
            // px.pop_back();
            // py.pop_back();
            // for (int i = 0; i < pxt.size(); i++) {
            //     p.x = pxt[i];
            //     p.y = pyt[i];
            //     path.points.push_back(p);
            // }
            // goal++;
        }


        // h.status = 0;
        // if (flag && path.points.size() != 8) {
        //     path.points.clear();
        //     flag = 0;
        //     for (int i = 0; i < path_x2.size(); i++) {
        //         p.x = path_x2[i];
        //         p.y = path_y2[i];
        //         path.points.push_back(p);
        //     }
        // }
        // if (flag && path.points.size() == 8) {
        //     h.status = 1;
        // }
        ros::spinOnce();
        npath.publish(goal);
        reqPath.publish(h);
		loop_rate.sleep();
    }

}

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
    if (msg->status == 3) {
        cout << "Requested" << endl;
        flag = 1;
    }
}
