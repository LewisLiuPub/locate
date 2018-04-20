//
// Created by waxz on 18-4-18.
//

#include <locate/locate.h>
#include <string>
using  namespace std;



Locate_Fusion::Locate_Fusion(ros::NodeHandle nh,ros::NodeHandle nh_private):nh_private_(nh_private), nh_(nh) {


}

void Locate_Fusion::init_params() {

    if(!nh_private_.getParam("base_frame_id",base_frame_id_))
        base_frame_id_ = "base_link";
    if(!nh_private_.getParam("laser_frame_id",laser_frame_id_))
        laser_frame_id_ = "laser";
    if(!nh_private_.getParam("pose_topic",pose_topic_))
        pose_topic_ = "amcl_pose";
    if(!nh_private_.getParam("enable_csm",enable_csm_))
        enable_csm_ = true;
    if(!nh_private_.getParam("enable_fft",enable_fft_))
        enable_fft_ = true;

    if(!nh_private_.getParam("transform_tolerance",tmp_tol))
        tmp_tol = 0.1;
    if(!nh_private_.getParam("global_frame_id",global_frame_id_))
        global_frame_id_ = "map";
    if(!nh_private_.getParam("odom_frame_id",odom_frame_id_))
        odom_frame_id_ = "odom";
    if(!nh_private_.getParam("tf_broadcast",tf_broadcast_))
        tf_broadcast_ = true;
    if(!nh_private_.getParam("pub_pose",pub_pose_))
        pub_pose_ = true;

    if(!nh_private_.getParam("stddev_x",stddev_x))
        stddev_x = 0.1;
    if(!nh_private_.getParam("stddev_y",stddev_y))
        stddev_y = 0.1;
    if(!nh_private_.getParam("stddev_yaw",stddev_yaw))
        stddev_yaw = 0.1;
    if(!nh_private_.getParam("match_prob_thresh",match_prob_thresh))
        match_prob_thresh = 0.3;

}

void Locate_Fusion::call_back() {
    /* compute match error, only update odom when match error low enough
     * if one step's match error reach limitation
     *  goto next step, or stop match use latest pose to update odom
     *
     * */
    // get base_pose and laser_scan

    // get base_pose, generate scan_ref

    // *****  1) check amcl match_error
    // compute match_error between scan_ref with scan_sens

    // if match_error > thresh, END match, return, not update odom

    // how to recovery amcl !!!

    // if match_error < thresh, save base_pose to lasted_match_pose

    // send pose to csm_matcher

    // *****  2) check csm match_error

    // compute match_error between scan_ref with scan_sens

    // if match_error > thresh, END match, update odom with lasted_match_pose

    // if match_error < thresh, save base_pose to lasted_match_pose

    // send pose to fftw

    // compute match_error between scan_ref with scan_sens

    // *****  3) check fftw match_error

    // if match_error > thresh, END match, update odom with lasted_match_pose

    // if match_error < thresh, save base_pose to lasted_match_pose




}

