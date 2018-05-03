//
// Created by waxz on 18-3-22.
//


#include <csm_wrapper/csm_wrapper.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_simulator/laser_simulator.h>
#include <fft_wrapper/fft_wrapper.h>
#include <locate/locate.h>


int main(int argc, char **argv) {


    ROS_INFO("start locate node");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    int rate;
    if (!nh_private.getParam("update_rate",rate)){
        rate = 15;
    }

    Locate_Fusion locate_fusion(nh,nh_private);
//    ros::spin();
    ros::Rate r(rate);

    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }


}