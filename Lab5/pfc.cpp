#include "point.h"
#include <math.h>
#include <vector>

using namespace std;

float Wthresh = 0.1;
lookAheadDistance = 0.5;

int main(int argc, char** argv) {


}

Point lookAheadPoint(Point mu, vector<float> path_x, vector<float> path_y) {
    // go over all segmesnt in current path_x
    // Use lookahead distance as the radius of a circle with the closest path point as the center
    // find the intersection points of each segment with this circle
    // test if one of the intersection points lies within the path segment, if so this is the lookahead point
    // Failure conditions:
    // - path segment larger than the circle drawn by lookahead distance
    //   Would need to test which point is forward of the robot
    // -
    if (distance(mu, Point(path_x[path_x.size()-1], path_y[path_y.size()-1])) < lookAheadDistance ){
        return Point(path_x[path_x.size()-1], path_y[path_y.size()-1]);
    }
    else {
        Point closest = findClosestPoint(path_x, path_y, mu);
        float x1,x2,y1,y2;
        for (int i = 0; i < path_x.size()-1; i++) {
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
            if (distance(a,candidate1) + distance(candidate1,b) == distance(a,b)) {
                return candidate1
            }
            else if (distance(a,candidate2) + distance(candidate2,b) == distance(a,b)) {
                return candidate2;
            }
        }
    }
}

float distance(Point A, Point B) {
    return sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2));
}

Point findClosestPoint(vector<float> path_x, vector<float> path_y, Point mu) {
    // if (mu.x < path_x[0]){
    //     Point start(path_x[0], path_y[0]);
    //     return start;
    // }
    // else {
    Point prev;
    Point next;
    Point a;
    vector<Point> candidates;  // will hold the closes point from the robot to each segment
    for (int i = 0; i < path_x.size()-1; i++) {
        prev.setx(path_x[i]);
        prev.sety(path_y[i]);
        next.setx(path_x[i+1]);
        next.sety(path_y[i+1]);
        a = closestPoint(prev,next,mu);
        if (distance(prev,a) + distance(a, next) == distance(prev,next)) // point is on the segment
            candidates.push_back(a);                                     // Otherwise reject the point
    }
    Point closest = candidates[0];
    for (int f = 0; i < candidates.size(); f++) {
        if ( (pow(candidates[f].x-mu.x,2)+pow(candidates[f].y-mu.y,2)) < (pow(closest.x-mu.x,2)+pow(closest.y-mu.y,2)) ) {
            closest.setx(candidates[f].x);
            closest.sety(candidates[f].y);
        }
    }
    //Point closest = closestPoint(prev, next, mu);
    return closest;
    //}
}


Point closestPoint(Point A, Point B, Point P) {
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
