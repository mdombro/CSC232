#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point.h"
#include "actionlib_msgs/GoalStatus.h"
#include "point.h"
#include "node.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;
//using namespace ros;

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg);
void get_goal(const geometry_msgs::Point::ConstPtr& msg);
vector<Point> Astar(Point& start, Point& goal, int goalNum);
int heuristic(Node neighbor, Point goal);
int min(int a, int b);
//bool inSet(Node neighbor, vector<Node> closed);
void sort(vector<Node> q);
int computeCost(Point from, Point to, int goalNum);
Point getLoc(Point from, int i);
float distanceP(Point & A, Point & B);
Node* inSet(Node neighbor, vector<Node> set);


int pathFlag = 0;
int halt = 0;
vector<Point> pathAstar;
float coneExpansion = 0.3;
int goalNum = 0;
Point goal(0,0);  // set by exec
Point start(0,0);  // set by exec

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path;
    actionlib_msgs::GoalStatus h;

    ros::NodeHandle n;
    ros::Publisher npath = n.advertise<geometry_msgs::Polygon>("next_path", 1000, true);
    ros::Publisher Halt = n.advertise<actionlib_msgs::GoalStatus>("halt", 1000);
    ros::Subscriber sub = n.subscribe("plannerFlag",1000, path_dispatch);
    ros::Subscriber nGoal = n.subscribe("next_goal",1000, get_goal);
	ros::Rate loop_rate(1);

    while (ros::ok()) {
        cout << "Inputs: " << start.x << ", " << start.y << "  " << goal.x << ", " << goal.y << "   " << goalNum << endl;
        if (pathFlag) {goalNum++; pathAstar = Astar(start, goal, goalNum);}
        ros::spinOnce();
        npath.publish(path);
        Halt.publish(h);
        loop_rate.sleep();
    }
}

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
    if (msg->status == 3) {
        cout << "Requested" << endl;
        pathFlag = 1;
    }
    else {
        pathFlag = 0;
    }
}

void get_goal(const geometry_msgs::Point::ConstPtr& msg) {
    goal.setx(msg->x);
    goal.sety(msg->y);
}

vector<Point> Astar(Point& start, Point& goal, int goalNum) {
    Node startN(0, NULL, start);
    //priority_que<Node, vector<Node>, Compare> openList;
    vector<Node> openList;
    openList.push_back(startN);
    vector<Node> closedList;
    bool updateNeighbor = false;
    int cost;
    while (openList.size() != 0) {
        cout << "OpenList size: " << openList.size() << endl;
        Node *current;
        current = & openList[0];
        if ((*current).location.x == goal.x && (*current).location.y == goal.y) break;
        closedList.push_back(*current);
        for (int i = 0; i < 8; i++) {
            Node neighbor;
            neighbor.location = getLoc((*current).location, i);
            cost = (*current).cost + computeCost((*current).location, neighbor.location, goalNum);  // goal needed to extract orientation
            if (inSet(neighbor, closedList) != NULL) continue;
            else if (inSet(neighbor, openList) != NULL) {
                Node *n = inSet(neighbor, openList);
                if (cost < (*n).cost) {
                    updateNeighbor = true;
                }
                else
                    updateNeighbor = false;
            }
            else if (inSet(neighbor, openList) == NULL || updateNeighbor == true) { // if neighbor isnt in open or if a better path is found
                neighbor.from = current;
                neighbor.cost = cost;
                neighbor.priority = cost + heuristic(neighbor, goal);
                openList.push_back(neighbor);
                sort(openList);
            }
        }
    }
    vector<Point> path;
    Node* travel = &closedList[closedList.size()-1];
    while ((*travel).from != NULL) {
        path.push_back((*travel).location);
        travel = (*travel).from;
    }
    reverse(path.begin(), path.end());
    return path;
}

int heuristic(Node neighbor, Point goal) {
    int dx = (int)(abs(neighbor.location.x - goal.x)/0.25);
    int dy = (int)(abs(neighbor.location.y - goal.y)/0.25);
    return (dx +dy)+(-1)*min(dx, dy);
}

int min(int a, int b) {
    return (a < b) ? a : b;
}

Node* inSet(Node neighbor, vector<Node> set) {
    for (int i = 0; i < set.size(); i++) {
        if (neighbor.location.x == set[i].location.x && neighbor.location.y == set[i].location.y)
            return &set[i];
    }
    return NULL;
}

void sort(vector<Node> q) {  // hopefully there is never many nodes....
    Node* swap;
    for (int c = 0 ; c < ( q.size() - 1 ); c++) {
        for (int d = 0 ; d < q.size() - c - 1; d++) {
            if (q[d].priority < q[d+1].priority) {
                swap = &q[d];
                q[d] = q[d+1];
                q[d+1] = *swap;
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
        Point obs(Mx[i], 0.0);
        if (distanceP(to, obs) < coneExpansion) {
            cost += 50;
        }
    }
    return cost;
}

float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
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
