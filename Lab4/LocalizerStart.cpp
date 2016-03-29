#include "localizer.h"
#include "localizer_cmdline.h"

int main(int argc, char* argv[]) {
    gengetopt_args_info args;
    cmdline_parser(argc,argv,&args);
    coneRadii = args.coneRadii_arg;
    init(argc, argv, "localizer");
    geometry_msgs::PoseWithCovariance msg;
    Localizer localizer;

    NodeHandle n;
    Publisher posWCov = n.advertise<geometry_msgs::PoseWithCovariance>("/pos", 1);
    Subscriber cntrl = n.subscribe("/cmd_vel_mux/input/navi", 1000, &Localizer::cmdUpdate, &localizer);  // update the command velocities
    Subscriber beams = n.subscribe("/scan", 1000, &Localizer::handleScans, &localizer);
    //ros::Duration(1.3).sleep();
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        posWCov.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
