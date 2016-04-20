// Matthew Dombroski
// ECE 232 Lab 3

#include "gui.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "geometry_msgs/PoseWithCovariance.h"

GUI::GUI( QWidget * parent ) : QGLWidget( parent ), timer() {
    setMinimumSize( 600, 600 );
    timer = new QTimer( this );
    connect( timer, SIGNAL( timeout() ), this, SLOT( timer_callback() ) );
    timer->start( 100 ); // call timer_callback at 0.1 Hz (period is 100 ms)
    GUI::quaternion.resize(4);
    //ector<float> quaternion(4);
}

GUI::
~GUI() {

}

void GUI::handle_laserscan( const sensor_msgs::LaserScan::ConstPtr& msg ){
    // implement storing of laserscan message here
    GUI::scans.resize(msg->ranges.size());
    GUI::scans = msg->ranges;
    GUI::angleMin = msg->angle_min;
    GUI::angleIncrement = msg->angle_increment;
    return;
}

void GUI::handle_robot_mu( const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    GUI::robot_x = msg->pose.position.x;
    GUI::robot_y = msg->pose.position.y;
}

void GUI::handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
    // implement storing of robot pose here
    GUI::posx = msg->pose.pose.position.x;
    GUI::posy = msg->pose.pose.position.y;
    GUI::quaternion[0] = msg->pose.pose.orientation.w;    //.assign(0, msg->pose.pose.orientation.w);
    GUI::quaternion[1] = msg->pose.pose.orientation.x;    //.assign(1, msg->pose.pose.orientation.x);
    GUI::quaternion[2] = msg->pose.pose.orientation.y;      // .assign(2, msg->pose.pose.orientation.y);
    GUI::quaternion[3] = msg->pose.pose.orientation.z;    //.assign(3, msg->pose.pose.orientation.z);
    return;
}


void GUI::timer_callback( void ){
    ros::spinOnce(); // Process the messages in here
    GUI::repaint();
    return;
}

void GUI::initializeGL(){
    glClearColor( 1.0, 1.0, 1.0, 1.0 );
    //gluOrtho2D(-5,5,-5,5);
    glMatrixMode( GL_PROJECTION );
    gluOrtho2D( -5, 5, -5, 5 );
    glMatrixMode( GL_MODELVIEW );
    return;
}

void GUI::paintGL(){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    // draw a coordinate system at the origin
    glBegin( GL_LINES );
    glColor4f( 1.0, 0.0, 0.0, 1.0 );
    glVertex3f( 0.0, 0.0, 0.0 );
    glVertex3f( 1.0, 0.0, 0.0 );
    glColor4f( 0.0, 1.0, 0.0, 1.0 );
    glVertex3f( 0.0, 0.0, 0.0 );
    glVertex3f( 0.0, 1.0, 0.0 );
    glColor4f( 0.0, 0.0, 1.0, 1.0 );
    glVertex3f( 0.0, 0.0, 0.0 );
    glVertex3f( 0.0, 0.0, 1.0 );
    glEnd();

    // actual robot location
    glBegin(GL_LINE_LOOP);
    float cy = posy;
    float cx = posx;
    float r = 0.4;
    float num_segments = 20;
    float x = 0.0;
    float y = 0.0;
	for(int ii = 0; ii < num_segments; ii++)
	{
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle

		float x = r * cosf(theta);//calculate the x component
		float y = r * sinf(theta);//calculate the y component

        glColor4f(1.0, 0.0, 0.0, 1.0);
		glVertex3f(x + cx, y + cy, 0.0);//output vertex
	}
    glEnd();

    // Laser Scan Drawing
    tf::Quaternion q(GUI::quaternion[1], GUI::quaternion[2], GUI::quaternion[3], GUI::quaternion[0]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    glBegin(GL_LINES);
    glColor4f( 1.0, 0.0, 0.0, 1.0 );
    //std::cout << roll << " " << pitch << " " << yaw << std::endl;
    //std::cout << yaw << std::endl;
    float inc = angleIncrement;
    float angle = angleMin+yaw;
    for (int i = 0; i < GUI::scans.size(); i++) {
        glVertex3f(posx, posy, 0.0);
        glVertex3f(posx + (GUI::scans[i]*cos(angle)), posy + (GUI::scans[i]*sin(angle)), 0.0);
        angle += inc;
    }
    glEnd();

    glBegin(GL_LINES);
    glColor4f( 0.7, 0.2, 0.0, 1.0 );

    // Line from robot to cone
    glVertex3f(posx, posy, 0.0);
    glVertex3f(1.0,0.0,0.0);
    glEnd();

    // predicted robot location
    glBegin(GL_LINE_LOOP);
    cy = robot_y;
    cx = robot_x;
    r = 0.13;
    num_segments = 20;
    x = 0.0;
    y = 0.0;
	for(int f = 0; f < num_segments; f++)
	{
		float theta = 2.0f * 3.1415926f * float(f) / float(num_segments);//get the current angle

		float x = r * cosf(theta);//calculate the x component
		float y = r * sinf(theta);//calculate the y component

        glColor4f(0.2, 1.0, 0.1, 1.0);
		glVertex3f(x + cx, y + cy, 0.0);//output vertex
	}
    glEnd();

    return;
}
