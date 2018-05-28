//
// Created by waxz on 18-4-18.
//

#ifndef LOCATE_LOCATE_H
#define LOCATE_LOCATE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <laser_simulator/laser_simulator.h>
#include <fft_wrapper/fft_wrapper.h>
#include <csm_wrapper/csm_wrapper.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include "tf/message_filter.h"
#include "geometry_msgs/PoseArray.h"


#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>

// Custom particle service
#include "amcl/amcl_particles.h"



using namespace std;

typedef  message_filters::sync_policies::ApproximateTime<sm::LaserScan, gm::PoseWithCovarianceStamped> PS_Sync;

// Use a child class to get access to tf2::Buffer class inside of tf_
struct TransformListenerWrapper : public tf::TransformListener
{
    inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
};



double pdf_3d(std::valarray<double> sample, std::valarray<double> mean, std::valarray<double> stddev);


class Locate_Fusion{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::NodeHandle nh2_;


    ros::CallbackQueue queue;
    ros::CallbackQueue queue2;
    ros::CallbackQueue queue3;


    // *** 3rd party lib
    Csm_Wrapper * csm_wrapper;
    Laser_Simulator *gen_ptr;
    FFT_Fitter *fft_fitter_ptr;

    // **** parameters
    bool enable_csm_ ;
    bool enable_fft_;
    bool tf_broadcast_ ;
    bool pub_pose_;
    string odom_frame_id_ ;
    string base_frame_id_ ;
    string global_frame_id_ ;
    string laser_frame_id_;
    string pose_topic_;
    string vel_topic_;
    double tmp_tol;
    double stddev_x, stddev_y, stddev_yaw, match_prob_thresh;
    double csm_match_error_limit_;
    double csm_match_valid_limit_;
    double fft_match_error_limit_;
    bool odom_tf_update_;
    float tf_update_rate_;
    float vel_angular_min_;
    bool update_odom_chage_;
    float rate;
    float pose_change_dist_min_, pose_change_angle_min_;
    int max_partial_number_;
    int point_on_circle_;
    double radius_max_;
    int reset_amcl_filter_cnt_;
    double reset_x_cov_;
    double reset_y_cov_;
    double reset_yaw_cov_;




    // **** latest state
    sm::LaserScan latest_scan_;
    bool init_pose_set_;
    bool get_amcl_pose_ ;
    bool get_amcl_tf_;
    gm::PoseWithCovarianceStamped latest_pose_;
    geometry_msgs::Twist latest_vel_;
    tf::Transform latest_map_odom_tf_;
    tf::Stamped<tf::Pose> latest_amcl_map_odom_tf_;
    tf::Transform base_laser_tf_;
    geometry_msgs::PoseArray latest_partial_cloud_;
    tf::Stamped<tf::Pose> latest_odom_base_tf;
    gm::PoseWithCovarianceStamped latest_final_pose_;
    tf::Transform temp_map_odom_tf;




    // **** flag
    int match_count_;
    int nomove_cnt_;





    // **** sub
    // amcl_map_odom_tf, initial_pose, laser_scan, partial_cloud, vel, patialcloud
    // sync filter (odom_baselink, laser_scan)
    ros::Subscriber amclTf_sub_;
    ros::Subscriber initialPose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber partial_cloud_sub_;


    message_filters::Subscriber<sm::LaserScan>* laserscan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laserscan_filter_;



    // **** tf
    // listener, broadcaster
    // tranform method
    // lookup odom_baselink,
    // send map_odom
    tf::TransformListener *tf_listener_ptr;
    tf::TransformBroadcaster* tfb_;
    TransformListenerWrapper* tf_;
    ros::Duration transform_tolerance_;



    // **** pub
    // refine_pose
    ros::Publisher pose_pub_;


    // **** service client
    // update amcl partial cloud use base_link(or refine pose)
    ros::ServiceClient set_filters_client_;



    // **** callback
    // 4 callback laser+tf, initpose, vel, amcltf
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void amcltfReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void velReceived(const geometry_msgs::Twist::ConstPtr &msg);

    void amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void amclPartialReceived(const geometry_msgs::PoseArrayConstPtr &msg);






    // **** method
    void init_params();
    // lookup tf;
    bool lookup_tf(string frame1, string frame2, tf::Transform &transform_matrix,ros::Time t);
    // lookup tf
    bool lookup_base_laser_tf(const string &base_frame, const  string &laser_frame) ;
    bool lookup_odom_base_tf(tf::Transform &odom_base_pose,ros::Time t);

    void lookup_tf_change(string frame1, string frame2,tf::StampedTransform &tx_odom,ros::Time t_latst, ros::Time t_now);


    // lookup odom to baselink tf

    // compute icp pose with amcl pose
    double check_pose_prob(geometry_msgs::Pose mean_pose, geometry_msgs::Pose sample_pose);

    // use icp match pose to update map->odom tf
    void update_odom(gm::Pose base_pose,ros::Time t);
    // continious update map->odom tf in thread
    void update_tf();

    void update_local_tf();




    // set amcl partial filters to free space
    void set_filter(gm::Pose latest_pose,double x_cov,  double y_cov, double yaw_cov, double cloud_yaw);



#if 0
    // sync amclpose with laserscan
    message_filters::Subscriber<sm::LaserScan>* sync_scan_sub_;
    message_filters::Subscriber<gm::PoseWithCovarianceStamped>* sync_base_pose_sub_;

    message_filters::Synchronizer<PS_Sync> *sync_;


    void pose_cbk(const sm::LaserScan::ConstPtr &scan_msg, const gm::PoseWithCovarianceStamped::ConstPtr &pose_msg);
#endif
public:
    Locate_Fusion(ros::NodeHandle nh_,ros::NodeHandle nh_private_);


};



#endif //LOCATE_LOCATE_H
