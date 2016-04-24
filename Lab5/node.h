//#include "point.h"

class Node {
public:
    int cost;
    int priority;
    bool visited;
    Point location;
    Node* from;
    Node(Point loc);
    Node();
};

// Node::Node(int cost1 = 1, Node* from1 = NULL) {
//     cost = cost1;
//     from = from1;
//     visited = false;
// }
Node::Node() {
    cost = 0;
    priority = 0;
    from = NULL;
    visited = false;
}

Node::Node(Point loc) {
    cost = 0;
    priority = 0;
    from = NULL;
    location = loc;
    visited = false;
    //location = loc;
}
