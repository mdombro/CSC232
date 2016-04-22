#include "point.h"

class Node {
public:
    int cost;
    int priority;
    Point location();
    Node* from;
    Node(int cost1, Node* from1);
    Node(int cost1, Node* from1, Point loc);
};

Node::Node(int cost1, Node* from1) {
    cost = cost1;
    from = from1;
}

Node::Node(int cost1, Node* from1, Point loc) {
    cost = cost1;
    from = from1;
    location = loc;
}
