#include "gui.h"
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
    return;
}


void GUI::handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
    // implement storing of robot pose here
    return;
}


void GUI::timer_callback( void ){
    ros::spinOnce(); // Process the messages in here
    return;
}

void GUI::initializeGL(){
    glClearColor( 1.0, 1.0, 1.0, 1.0 );
    gluOrtho2D(-5,5,-5,5);
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
    // implement drawing of laserscan and robot pose here
    return;
}
