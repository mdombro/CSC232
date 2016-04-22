#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "actionlib_msgs/GoalStatus.h"
#include "point.h"
#include "node.h"
#include <priority_que>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace ros;

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg);
void get_goal(const geometry_msgs::Point32::ConstPtr& msg);
vector<Point> Astar(Point start, Point goal, int goalNum);
int heuristic(Node neighbor, Node goal);
int min(int a, int b);
bool inSet(Node neighbor, vector<Node> closed);
void sort(vector<Node> q);
int computeCost(Point from, Point to, int goalNum)
Point getLoc(Point from, int i);


int flag = 0;
int halt = 0;
Point goal(0,0);
vector<Point> pathAstar;
float coneExpansion = 0.3;

int main(int argc, char** argv) {
    init(argc, argv, "exec");

    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path;

    NodeHandle n;
    Publisher npath = n.advertise<geometry_msgs::Polygon>("next_path", 1000, true);
    Subscriber sub = n.subscribe("next_pathFlag",1000, path_dispatch);
    Subscriber sub = n.subscribe("next_goal",1000, get_goal);
	ros::Rate loop_rate(10);

    while (ros::ok()) {

        pathAstar = Astar(start, goal, goalNum);

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

void get_goal(const geometry_msgs::Point32::ConstPtr& msg) {
    goal.setx(msg->x);
    goal.sety(msg->y;)
}

vector<Point> Astar(Point start, Point goal, int goalNum) {
    Node startN(0, NULL, start);
    //priority_que<Node, vector<Node>, Compare> openList;
    vector<Node> openList;
    openList.push_back(startN);
    vector<Node> closedList;
    int cost;
    while (openList.size() != 0) {
        Node current;
        current = openList[0];
        openList.erase(openList.begin()); // remove the first element
        sort(openList);
        if (current.x == goal.x && current.y == goal.y) break;
        closedList.push_back(current);
        for (int i = 0; i < 8; i++) {
            Node neighbor;
            neighbor.location = getLoc(current.location, i);
            cost = current.cost + computeCost(current, neighbor, goalNum);  // goal needed to extract orientation
            if (inSet(neighbor, closedList)) continue;
            if (!inSet(neighbor, openList)) {
                neighbor.from = curent;
                neighbor.cost = cost;
                neighbor.priority = cost + heuristic(neighbor, goal);
                openList.push_back(neighbor);
                sort(openList);
            }
            else if (cost >= neighbor.cost)  // if in openList and this path is greater cost
                continue;                    // ignore this path

        }
    }
    vector<Point> path;
    Node travel = closedList[closedList.size()-1];
    while (travel.from != null) {
        path.push_back(travel);
        travel = travel.from;
    }
    reverse(path.begin(), path.end());
    return path;
}

int heuristic(Node neighbor, Node goal) {
    int dx = (int)(abs(neighbor.location.x - goal.location.x)/0.25);
    int dy = (int)(abs(neighbor.location.y - goal.location.y)/0.25);
    return (dx +dy)+(-1)*min(dx, dy);
}

int min(int a, int b) {
    return (a < b) ? a : b;
}

bool inSet(Node neighbor, vector<Node> closed) {
    for (int i = 0; i < closed.size(); i++) {
        if (neighbor.location.x == closed[i].location.x && neighbor.location.y == closed[i].location.y)
            return true;
    }
    else
        return false;
}

void sort(vector<Node> q) {  // hopefully there is never many nodes....
    Node* swap;
    for (c = 0 ; c < ( q.size() - 1 ); c++) {
        for (d = 0 ; d < q.size() - c - 1; d++) {
            if (q[d].priority < q[d+1].priority) {
                swap = q[d];
                q[d] = q[d+1];
                q[d+1] = swap;
            }
        }
    }
}

int computeCost(Point from, Point to, int goalNum) {
    int cost = 1;  // base transition cost of 1 - this includes diagonals, will monitor performance
    switch (goalNum) {
        case 1:
        case 3:
        case 5:
        case 7:
        case 9:
        case 11:
            if (to.y < 0) cost += 50;
            break;
        case 2:
        case 4:
        case 6:
        case 8:
        case 12:
            if (to.y > 0) cost += 50;
            break;
    }
    float Mx[] = {1.0, 2.0, 3.0, 5.0, 6.0, 7.0};
    for (int i = 0; i < 6; i++) {
        if (distanceP(to, Point(Mx[i], 0.0) < coneExpansion) {
            cost += 50;
        }
    }
    return cost;
}

Point getLoc(Point from, int i) {
    switch (i) {  // going in a ccw fashion from r to l
        case 0:
            return Point(from.x, from.y-0.25);
        case 1:
            return Point(from.x+0.25, from.y-0.25);
        case 2:
            return Point(from.x+0.25, from.y);
        case 3:
            return Point(from.x+0.25, from.y+0.25);
        case 4:
            return Point(from.x, from.y+0.25);
        case 5:
            return Point(from.x-0.25, from.y+0.25);
        case 6:
            return Point(from.x-0.25, from.y);
        case 7:
            return Point(from.x-0.25, from.y-0.25);
    }
}
