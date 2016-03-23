// Matthew Dombroski
// ECE 232 Lab 3

#include <QtGui/QApplication>
#include "gui.h"
#include "gui_process_cmdline.h"

using namespace std;

int main( int argc, char* argv[] ){
    gengetopt_args_info args;
    cmdline_parser( argc, argv, &args );
    QApplication app( argc, argv );
    ros::init( argc, argv, "gui" );
    ros::NodeHandle node_handle;
    GUI gui;
    ros::Subscriber subscriber_reset_odometry = node_handle.subscribe( "/laserscan", 1, &GUI::handle_laserscan, &gui );
    ros::Subscriber subscriber_odom = node_handle.subscribe( "/odom", 1, &GUI::handle_odom, &gui );
    gui.show();
    return app.exec();
}
