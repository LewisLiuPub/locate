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

    // **** parameters



    // **** sub
    // amcl_map_odom_tf, initial_pose, laser_scan, partial_cloud
    // sync filter (odom_baselink, laser_scan)


    // **** tf
    // listener, broadcaster
    // tranform method
    // lookup odom_baselink,
    // send map_odom


    // **** pub
    // refine_pose

    // **** service client
    // update amcl partial cloud use base_link(or refine pose)


    // **** callback
    //

    // **** method








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
    string vel_topic_;
    double tmp_tol;
    double stddev_x, stddev_y, stddev_yaw, match_prob_thresh;
    double csm_match_error_limit_;
    double fft_match_error_limit_;
    bool odom_tf_update_;
    float tf_update_rate_;
    float vel_angular_min_;
    bool init_pose_set_;
    bool get_amcl_pose_ ;


    sm::LaserScan latest_scan_;





    // *** lib
    Csm_Wrapper * csm_wrapper;
    Laser_Simulator *gen_ptr;
    FFT_Fitter *fft_fitter_ptr;

    // tf listener must initialize after ros node init
    tf::TransformListener *tf_listener_ptr;
    tf::TransformBroadcaster* tfb_;
    TransformListenerWrapper* tf_;

    tf::Transform latest_tf_;

    tf::Transform base_laser_tf_;
    ros::Duration transform_tolerance_;

    // pub
    ros::Publisher pose_pub_;

    // sub
    message_filters::Subscriber<sm::LaserScan>* sync_scan_sub_;
    message_filters::Subscriber<gm::PoseWithCovarianceStamped>* sync_base_pose_sub_;

    message_filters::Synchronizer<PS_Sync> *sync_;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;


    ros::Subscriber vel_sub_, initial_pose_sub_, amcl_pose_sub_;


    //service client
    ros::ServiceClient set_filters_client_;



    //
    gm::PoseWithCovarianceStamped latest_pose_;

    // vel
    geometry_msgs::Twist latest_vel_;




    // subscribe
    ros::Subscriber scan_sub_, base_pose_sub_;

    // publish
    ros::Publisher base_pose_pub_;

    // time sync filter

    // callback


    // method
    // compute match error
    float get_match_error();

    // get scan_ref given pose
    void get_scan_ref();

    // update odom

    void init_params();



    // compute icp pose with amcl pose
    double check_pose_prob(geometry_msgs::Pose mean_pose, geometry_msgs::Pose sample_pose);

    // lookup tf
    void lookup_base_laser_tf(const string &base_frame, const  string &laser_frame) ;

    // use icp match pose to update map->odom tf
    void update_odom(gm::Pose base_pose);

    // continious update map->odom tf in thread
    void update_tf();


    void pose_cbk(const sm::LaserScan::ConstPtr &scan_msg, const gm::PoseWithCovarianceStamped::ConstPtr &pose_msg);

    // listen velocity
    // if go_forward without large rotation
    // enable icp
    void current_vel_cbk(const geometry_msgs::Twist::ConstPtr &msg);

    // set amcl partial filters to free space
    void set_filter();

    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);

    bool get_baselink(const sensor_msgs::LaserScanConstPtr &laser_scan,tf::Stamped<tf::Pose> &odom_pose);

    void amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);




    //
public:
    Locate_Fusion(ros::NodeHandle nh_,ros::NodeHandle nh_private_);


};



#endif //LOCATE_LOCATE_H
