//
// Created by waxz on 18-4-18.
//

#ifndef LOCATE_LOCATE_H
#define LOCATE_LOCATE_H

#include <ros/ros.h>
#include <string>
#include <laser_simulator/laser_simulator.h>
#include <fft_wrapper/fft_wrapper.h>
#include <csm_wrapper/csm_wrapper.h>


using namespace std;

class Locate_Fusion{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;


    // params
    bool enable_csm_ ;
    bool enable_fft_;
    bool tf_broadcast_ ;
    bool pub_pose_;
    string odom_frame_id_ ;
    string base_frame_id_ ;
    string global_frame_id_ ;
    string laser_frame_id_;
    string pose_topic_;
    double tmp_tol;
    double stddev_x, stddev_y, stddev_yaw, match_prob_thresh;


    // *** lib
    Csm_Wrapper * csm_wrapper;
    Laser_Simulator *gen_ptr;
    FFT_Fitter *fft_fitter_ptr;

    // tf listener must initialize after ros node init
    tf::TransformListener *tf_listener_ptr;
    tf::TransformBroadcaster* tfb_;

    void init_params();

    // subscribe
    ros::Subscriber scan_sub_, base_pose_sub_;

    // publish
    ros::Publisher base_pose_pub_;

    // time sync filter

    // callback
    void call_back();


    // method
    // compute match error
    float get_match_error();

    // get scan_ref given pose
    void get_scan_ref();

    // update odom
    void update_odom();
    //
public:
    Locate_Fusion(ros::NodeHandle nh_,ros::NodeHandle nh_private_);


};



#endif //LOCATE_LOCATE_H
