#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point.h"
#include "point.h"
#include "node.h"
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovariance.h"

using namespace std;

vector<Point> Astar(Point start, Point goal, int goalNum);
int heuristic(Node neighbor, Point goal);
int min(int a, int b);
bool Sort(Node* a, Node* b);
int computeCost(Point from, Point to, int goalNum, int cmd);
Point getLoc(Point from, int i);
float distanceP(Point & A, Point & B);
void handle_local(const geometry_msgs::PoseWithCovariance::ConstPtr& msg);
float distanceP(Point & A, Point & B);
Point mu(0,0);

vector<Point> pathAstar;
float coneExpansion = 0.40;
int goalNum = 0;    // keep track of goal number
Point goal(0,0);
Point start(0,0);
float goalThresh = 0.001;  // to allow for float inaccuracies
float GoalThresh = 0.4;    // Radial distance for when to compute the next path
float gridDisc = 0.25;
float closedNodeThresh = 0.001;   // essentially same as goalThresh, just used in a different part of code

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path;

    vector<Point> goals;
    goals.push_back(Point(0.0, 0.0));
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
    goals.push_back(Point(0.5, 0.0));
    p.x = goals[0].x;
    p.y = goals[0].y;
    path.points.push_back(p);

    ros::NodeHandle n;
    ros::Publisher npath = n.advertise<geometry_msgs::Polygon>("next_path", 1000, true);
    ros::Subscriber local = n.subscribe("/pos", 1000, handle_local);
	ros::Rate loop_rate(20);

    while (ros::ok()) {
        cout << "Inputs: " << start.x << ", " << start.y << "  " << goal.x << ", " << goal.y << "   " << goalNum << endl;
        if (distanceP(mu, goals[goalNum]) < GoalThresh) {
            start = goals[goalNum];
            goalNum++;
            pathAstar = Astar(start, goals[goalNum], goalNum);
            for (int i = 1; i < pathAstar.size(); i++) {
                p.x = pathAstar[i].x;
                p.y = pathAstar[i].y;
                cout << "Path: " << p.x << " " << p.y << endl;
                path.points.push_back(p);
            }
        }
        ros::spinOnce();
        npath.publish(path);
        loop_rate.sleep();
    }
}

void handle_local(const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    mu.x = msg->pose.position.x;
    mu.y = msg->pose.position.y;
}

float distanceP(Point & A, Point & B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
}

vector<Point> Astar(Point start, Point goal, int goalNum) {
    Node startN(start);
    startN.cost = 0;
    startN.priority = 0;
    vector<Node*> openList;
    openList.push_back(&startN);
    vector<Node*> closedList;
    int cost;
    while (openList.size() != 0) {
        Node *current = openList[0];
        openList.erase(openList.begin());
        (*current).visited = true;
        closedList.push_back(current);
        if ( abs((*current).location.x - goal.x) < goalThresh && abs((*current).location.y - goal.y) < goalThresh) break;
        for (int i = 0; i < 8; i++) {
            Node* neighbor = new Node(getLoc((*current).location, i));
            cost = (*current).cost + computeCost((*current).location, (*neighbor).location, goalNum, i);  // goal needed to extract orientation
            for (int o = 0; o < closedList.size(); o++) {  // search the closedList for the node
                Node *tmp = closedList[o];
                if (distanceP((*neighbor).location, (*tmp).location) < closedNodeThresh) {
                    (*neighbor).visited = true;
                }
            }  // inefficient but restricts nodes being double expanded - A* will not converge otherwise
            if (!(*neighbor).visited) {
                if (cost < (*neighbor).cost) {
                    (*neighbor).priority = cost + heuristic(*neighbor, goal);
                    (*neighbor).cost = cost;
                    (*neighbor).from = current;
                    openList.push_back(neighbor);
                }
            }
            std::sort(openList.begin(), openList.end(), Sort);
        }
    }
    vector<Point> path;
    Node* travel = closedList[closedList.size()-1];
    while ((*travel).from != NULL) {
        //cout << "Path reconstruct: " << (*travel).location.x << " " << (*travel).location.y << endl;
        path.push_back((*travel).location);
        travel = (*travel).from;
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}

int heuristic(Node neighbor, Point goal) {
    float h;
    // equation for diaganol distance found at:
    // http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
    // where D1 = 1
    // D2 = sqrt(2)
    int dx = (int)(abs(neighbor.location.x - goal.x)/gridDisc);
    int dy = (int)(abs(neighbor.location.y - goal.y)/gridDisc);
    h = (dx +dy)+(sqrt(2)-2)*min(dx, dy);
    //h *= (1.0 + 1.0/30.0);  // makes the heuristic inadmissable
                              //  but can potentially make the path straighter
    return h;
}

int min(int a, int b) {
    return (a < b) ? a : b;
}

bool Sort(Node* a, Node* b) {
    return (*a).priority < (*b).priority;
}

int computeCost(Point from, Point to, int goalNum, int cmd) {
    int cost;
    switch (cmd) {
        case 0:
        case 2:
        case 4:
        case 6:
            cost = 1;
            break;
        case 1:
        case 3:
        case 5:
        case 7:
            cost = sqrt(2);
            break;
    }
    // cout << "Cmd: " << cmd << " prevCmd: " << prevCmd << endl;
    // if (abs(cmd-prevCmd) > 2) {
    //     cost += 5;
    // }
    // prevCmd = cmd;
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
        case 10:
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

Point getLoc(Point from, int i) {
    switch (i) {  // going in a ccw fashion from moving in negative y to negative x and y
        case 0:
            return Point(from.x, from.y-gridDisc);
        case 1:
            return Point(from.x+gridDisc, from.y-gridDisc);
        case 2:
            return Point(from.x+gridDisc, from.y);
        case 3:
            return Point(from.x+gridDisc, from.y+gridDisc);
        case 4:
            return Point(from.x, from.y+gridDisc);
        case 5:
            return Point(from.x-gridDisc, from.y+gridDisc);
        case 6:
            return Point(from.x-gridDisc, from.y);
        case 7:
            return Point(from.x-gridDisc, from.y-gridDisc);
    }
}
