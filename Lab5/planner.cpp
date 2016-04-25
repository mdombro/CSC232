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
vector<Point> Astar(Point start, Point goal, int goalNum);
int heuristic(Node neighbor, Point goal);
int min(int a, int b);
//bool inSet(Node neighbor, vector<Node> closed);
void sort(vector<Node> q);
bool sort2(Node* a, Node* b);
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
float goalThresh = 0.001;

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    geometry_msgs::Point32 p;
    geometry_msgs::Polygon path;
    actionlib_msgs::GoalStatus h;

    vector<Point> goals;
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

    ros::NodeHandle n;
    ros::Publisher npath = n.advertise<geometry_msgs::Polygon>("next_path", 1000, true);
    //ros::Publisher Halt = n.advertise<actionlib_msgs::GoalStatus>("halt", 1000);
    ros::Subscriber sub = n.subscribe("plannerFlag",1000, path_dispatch);
    //ros::Subscriber nGoal = n.subscribe("next_goal",1000, get_goal);
	ros::Rate loop_rate(1);

    // Point a(0,0);
    // Point b(1.5,0);
    // cout << "We in it" << endl;
    // pathAstar = Astar(a, b, 1);
    // cout << "Path size: " << pathAstar.size() << endl;
    // for (int t = 0; t < pathAstar.size(); t++) {
    //     cout << pathAstar[t].x << " " << pathAstar[t].y << endl;
    // }
    // cout << "Out of it" << endl;

    while (ros::ok()) {
        cout << "Inputs: " << start.x << ", " << start.y << "  " << goal.x << ", " << goal.y << "   " << goalNum << endl;
        if (pathFlag) {
            path.points.clear();
            goalNum++;
            pathFlag = 0;
            pathAstar = Astar(start, goals[goalNum-1], goalNum);
            for (int i = 0; i < pathAstar.size(); i++) {
                p.x = pathAstar[i].x;
                p.y = pathAstar[i].y;
                cout << "Path: " << p.x << " " << p.y << endl;
                path.points.push_back(p);
            }
        }
        ros::spinOnce();
        npath.publish(path);
        //Halt.publish(h);
        loop_rate.sleep();
    }
}

void path_dispatch(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
    cout << msg->status << endl;
    if (msg->status == 3) {
        cout << "Requested" << endl;
        pathFlag = 1;
    }
}

void get_goal(const geometry_msgs::Point::ConstPtr& msg) {
    goal.setx(msg->x);
    goal.sety(msg->y);
}

vector<Point> Astar(Point start, Point goal, int goalNum) {
    Node startN(start);
    startN.cost = 0;
    startN.priority = 0;
    //priority_que<Node, vector<Node>, Compare> openList;
    vector<Node*> openList;
    openList.push_back(&startN);
    vector<Node*> closedList;
    bool updateNeighbor = false;
    int cost;
    while (openList.size() != 0) {
        //cout << "OpenList size: " << openList.size() << endl;
        Node *current = openList[0];
        openList.erase(openList.begin());
        (*current).visited = true;
        closedList.push_back(current);
        if ( abs((*current).location.x - goal.x) < goalThresh && abs((*current).location.y - goal.y) < goalThresh) break;
        for (int i = 0; i < 8; i++) {
            //cout << (*current).location.x << " " << (*current).location.y << endl;
            Node* neighbor = new Node(getLoc((*current).location, i)); //getLoc((*current).location, i);
            //cout << "neighbor loc: " << neighbor->location.x << " " << neighbor->location.y << endl;
            cost = (*current).cost + computeCost((*current).location, (*neighbor).location, goalNum);  // goal needed to extract orientation
            //cout << "Cost to neighbor: " << cost << endl;
            if (!(*neighbor).visited) {   //inSet(neighbor, closedList) != NULL
                if (cost < (*neighbor).cost) {
                    (*neighbor).priority = cost + heuristic(*neighbor, goal);
                    (*neighbor).cost = cost;
            //        cout << "Cost for neighbor " << i << " Heuristic: " << heuristic(*neighbor, goal) << endl;
                    (*neighbor).from = current;
                    openList.push_back(neighbor);
            //        cout << "Open List new size: " << openList.size() << endl;
                    //sort(openList);
                    std::sort(openList.begin(), openList.end(), sort2);
            //        cout << "Did we sort" << endl;
                }
            }
        }
    }
    cout << "Does A* exit" << endl;
    vector<Point> path;
    Node* travel = closedList[closedList.size()-1];
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
        if (abs(neighbor.location.x - set[i].location.x) < goalThresh && abs(neighbor.location.y - set[i].location.y) < goalThresh)
            return &set[i];
    }
    return NULL;
}

bool sort2(Node* a, Node* b) {
    return (*a).priority < (*b).priority;
}

void sort(vector<Node>& q) {  // hopefully there is never many nodes....
    Node swap;
    //vector<Node> tmp;
    for (int c = 0 ; c < (q.size() - 1 ); c++) {
        for (int d = 0 ; d < q.size() - c - 1; d++) {
            //tmp = *q;
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

// Node getLoc(Point from, int i) {
//     switch (i) {  // going in a ccw fashion from r to l
//         case 0:
//             return Node(Point(from.x, from.y-0.25));
//         case 1:
//             return Node(Point(from.x+0.25, from.y-0.25));
//         case 2:
//             return Node(Point(from.x+0.25, from.y));
//         case 3:
//             return Node(Point(from.x+0.25, from.y+0.25));
//         case 4:
//             return Node(Point(from.x, from.y+0.25));
//         case 5:
//             return Node(Point(from.x-0.25, from.y+0.25));
//         case 6:
//             return Node(Point(from.x-0.25, from.y));
//         case 7:
//             return Node(Point(from.x-0.25, from.y-0.25));
//     }
// }

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
