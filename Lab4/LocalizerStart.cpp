#include "Localizer.h"
#include "LocalizerStart_cmdline.h"
#include <ros/ros.h>

using namespace std;
//using namespace ros;
//using namespace Localizer;

int main(int argc, char* argv[]) {
    gengetopt_args_info args;
    cmdline_parser(argc,argv,&args);
    ros::init(argc, argv, "localizer");
    nav_msgs::Odometry msg;
    Localizer localizer;
    localizer.setConeRadii(args.coneRadii_arg);
    float alphas = 0.01;
    localizer.setAlpha(alphas);
    ros::NodeHandle n;
    ros::Publisher posWCov = n.advertise<nav_msgs::Odometry>("/pos", 1);
    ros::Subscriber cntrl = n.subscribe("/cmd_vel_mux/input/navi", 1000, &Localizer::cmdUpdate, &localizer);  // update the command velocities
    ros::Subscriber beams = n.subscribe("/scan", 1000, &Localizer::handleScans, &localizer);
    //ros::Duration(1.3).sleep();
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        msg.pose.pose.position.x = localizer.getx();
        msg.pose.pose.position.y = localizer.gety();
        ros::spinOnce();
        posWCov.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
