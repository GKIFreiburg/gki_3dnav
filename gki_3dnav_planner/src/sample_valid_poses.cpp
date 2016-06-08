#include <ros/ros.h>
#include <gki_3dnav_planner/SampleValidPoses.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample_valid_poses");
    ros::NodeHandle nh;

    if(argc < 2) {
        printf("Usage: %s n_poses [max_tries]\n", argv[0]);
        return 1;
    }

    ros::Publisher pubPoses = nh.advertise<geometry_msgs::PoseArray>("valid_poses", 3);

    gki_3dnav_planner::SampleValidPoses srv;
    srv.request.n = atoi(argv[1]);
    if(argc >= 3)
        srv.request.max_tries = atoi(argv[2]);
    else
        srv.request.max_tries = 1000;

    if(!ros::service::call("/move_base_node/GKI3dNavPlanner/sample_valid_poses", srv)) {
        ROS_ERROR("Could not sample");
        return 1;
    }

    while(pubPoses.getNumSubscribers() <= 0) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    pubPoses.publish(srv.response.poses);
    ros::Duration(1.0).sleep();
}

