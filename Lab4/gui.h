#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <QtGui/QApplication>
#include <QtGui/QWidget>
#include <QtOpenGL/QGLWidget>
#include <QtCore>
#include <QTimer>
#include <GL/glu.h>
#include <vector>
class GUI: public QGLWidget {
    Q_OBJECT
    float posx, posy;
    std::vector<float> scans;
    float angleMin, angleIncrement;
    std::vector<float> quaternion;
    public:
        GUI( QWidget * parent = NULL );
        virtual ~GUI();
        void handle_laserscan( const sensor_msgs::LaserScan::ConstPtr& msg );
        void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
        QTimer* timer;
        protected slots:
        void timer_callback( void );
    protected:
        virtual void initializeGL();
        virtual void paintGL();
};
