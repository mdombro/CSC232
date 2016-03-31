// Matthew Dombroski
// ECE 232 Lab 3

#include "gui.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

GUI::GUI( QWidget * parent ) : QGLWidget( parent ), timer() {
    setMinimumSize( 600, 600 );
    timer = new QTimer( this );
    connect( timer, SIGNAL( timeout() ), this, SLOT( timer_callback() ) );
    timer->start( 100 ); // call timer_callback at 0.1 Hz (period is 100 ms)
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


void GUI::handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
    // implement storing of robot pose here
    GUI::posx = msg->pose.pose.position.x;
    GUI::posy = msg->pose.pose.position.y;
    GUI::quaternion.push_back(msg->pose.pose.orientation.w);
    GUI::quaternion.push_back(msg->pose.pose.orientation.x);
    GUI::quaternion.push_back(msg->pose.pose.orientation.y);
    GUI::quaternion.push_back(msg->pose.pose.orientation.z);
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
    glBegin(GL_LINE_LOOP);
    float cy = posy;
    float cx = posx;
    float r = 0.1;
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
    glBegin(GL_LINES);
    glColor4f( 1.0, 0.0, 0.0, 1.0 );
    tf::Quaternion q(GUI::quaternion[1], GUI::quaternion[2], GUI::quaternion[3], GUI::quaternion[0]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    float inc = angleIncrement;
    float angle = angleMin - yaw;
    for (int i = 0; i < GUI::scans.size(); i++) {
        glVertex3f(posx, posy, 0.0);
        glVertex3f((GUI::scans[i]*cos(angle)), (GUI::scans[i]*sin(angle)), 0.0);
        angle += inc;
    }
    glEnd();
    return;
}
