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
#include <plog/Log.h>

// lib for test
Csm_Wrapper * csm_wrapper;
Laser_Simulator *gen_ptr;
FFT_Fitter *fft_fitter_ptr;

// tf listener must initialize after ros node init
tf::TransformListener *tf_listener_ptr;
tf::Transform base_laser_tf;
gm::Pose laser_pose;

ros::Publisher scan_t, scan_s;
sm::LaserScan pre_scan, cur_scan;

// base_pose cbk
void base_pose_cbk(const gm::PoseWithCovarianceStamped::ConstPtr &pose_msg) {
//    ROS_INFO("base_pose_cbk!!");
    // first transform base_pose to get laser_pose
    tf::Transform fix_base_tf_;
    tf::poseMsgToTF(pose_msg->pose.pose, fix_base_tf_);
    // apply transform , get laser pose in fix frame
//    ROS_INFO("poseTFToMsg");
    tf::poseTFToMsg(fix_base_tf_ * base_laser_tf, laser_pose);

}

//void scan_cbk(const sm::LaserScan::ConstPtr & scan_msg){
//    if (pre_scan.ranges.empty()){
//        pre_scan = *scan_msg;
//        return;
//    }
//    cur_scan = * scan_msg;
//
//
//
//    ROS_INFO("start csm!!!!");
//
//    vector<double> res = csm_wrapper->csm_fit(pre_scan,cur_scan);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");
//
//}
void lookup_base_laser_tf(const string &base_frame, const  string &laser_frame) {
    ROS_INFO("wait for tf,%s, %s", base_frame.c_str(), laser_frame.c_str());
    tf::StampedTransform base_laser_tf_stamp;

    while (ros::ok()) {
        ros::Time t = ros::Time::now();
        try {

            tf_listener_ptr->waitForTransform(base_frame, laser_frame, t, ros::Duration(5.0));
            tf_listener_ptr->lookupTransform(base_frame, laser_frame, t, base_laser_tf_stamp);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("lookup transformation %s to %s failure", base_frame.c_str(), laser_frame.c_str());
        }
        break;
    }
    base_laser_tf = base_laser_tf_stamp;
    ROS_INFO("lookup transformation %s to %s  successful", base_frame.c_str(), laser_frame.c_str());

}

void locate(const sm::LaserScan::ConstPtr &scan_msg, const gm::PoseWithCovarianceStamped::ConstPtr &pose_msg){

    ROS_INFO("sync!!!!!!");

    // base pose to laser pose
    tf::Transform fix_base_tf_;
    tf::poseMsgToTF(pose_msg->pose.pose, fix_base_tf_);
    // apply transform , get laser pose in fix frame
//    ROS_INFO("poseTFToMsg");
    tf::poseTFToMsg(fix_base_tf_ * base_laser_tf, laser_pose);


    sm::LaserScan::Ptr map_scan_ptr = gen_ptr->get_laser(laser_pose);

    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");


    gm::Pose base_pose;
    base_pose =pose_msg->pose.pose;

    gm::Pose res_pose = csm_wrapper->get_base_pose(*map_scan_ptr,*scan_msg,base_pose, base_laser_tf);


    ROS_INFO("get amcl pose [%f,%f,%f] \n get icp pose [%f,%f,%f]",
             pose_msg->pose.pose.position.x,pose_msg->pose.pose.position.y,pose_msg->pose.pose.orientation.w,
             res_pose.position.x,res_pose.position.y,res_pose.orientation.w);
}

void locate2(const gm::PoseWithCovarianceStamped::ConstPtr &pose_msg, const nav_msgs::Odometry::ConstPtr &true_pose_msg){

    ROS_INFO("sync!!!!!!");

    // base pose to laser pose
    tf::Transform fix_base_tf_, true_fix_base_tf_;
    tf::poseMsgToTF(pose_msg->pose.pose, fix_base_tf_);
    tf::poseMsgToTF(true_pose_msg->pose.pose, true_fix_base_tf_);


    // apply transform , get laser pose in fix frame
//    ROS_INFO("poseTFToMsg");
    gm::Pose true_laser_pose;

    tf::poseTFToMsg(fix_base_tf_ * base_laser_tf, laser_pose);
    tf::poseTFToMsg(true_fix_base_tf_ * base_laser_tf, true_laser_pose);


    // fake pose
    double true_error_x = 0.1, true_error_y = 0.2;
//    laser_pose = true_laser_pose;
//    laser_pose.position.x = laser_pose.position.x + true_error_x;
//    laser_pose.position.y  = laser_pose.position.y + true_error_y;




    sm::LaserScan::Ptr map_scan_ptr = gen_ptr->get_laser(true_laser_pose);
    sm::LaserScan sm(*map_scan_ptr);



    sm::LaserScan::Ptr sensor_scan_ptr = gen_ptr->get_laser(laser_pose);
    sm::LaserScan ss(*sensor_scan_ptr);

    scan_s.publish(sm);
    scan_t.publish(ss);




    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");


    gm::Pose base_pose;
    base_pose =pose_msg->pose.pose;

    gm::Pose res_pose = csm_wrapper->get_base_pose(ss,sm,base_pose, base_laser_tf);

    fft_fitter_ptr->transform(ss,res_pose);



    ROS_INFO("\n get true base pose [%f,%f,%f,%f,%f,%f]\nget amcl base  pose [%f,%f] \n get icp pose [%f,%f,%f,%f,%f,%f]",
             true_pose_msg->pose.pose.position.x,true_pose_msg->pose.pose.position.y,true_pose_msg->pose.pose.orientation.x,
             true_pose_msg->pose.pose.orientation.y,true_pose_msg->pose.pose.orientation.z,true_pose_msg->pose.pose.orientation.w,

             base_pose.position.x,base_pose.position.y,
             res_pose.position.x,res_pose.position.y,res_pose.orientation.x,
             res_pose.orientation.y,res_pose.orientation.z,res_pose.orientation.w
    );
}


int main(int argc, char **argv) {
    plog::init(plog::verbose, "/home/waxz/log.csv", 10000000, 5);


    ROS_INFO("start locate node");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");


    // parameters
    string base_frame, laser_frame, pose_topic;

    if(!nh.getParam("base_frame",base_frame))
        base_frame = "base_link";
    if(!nh.getParam("laser_frame",laser_frame))
        laser_frame = "base_laser_link";
    if(!nh.getParam("pose_topic",pose_topic))
        pose_topic = "amcl_pose";



    // initlize lib
    csm_wrapper = new Csm_Wrapper(nh,nh_private) ;
    gen_ptr = new Laser_Simulator(nh, nh_private);
    fft_fitter_ptr = new FFT_Fitter(nh, nh_private);

    tf_listener_ptr = new(tf::TransformListener);




    ROS_INFO("start wait tf");

    // get tf
    lookup_base_laser_tf(base_frame,laser_frame);

    // initialize simulator

//    ros::Subscriber base_pose_sub = nh.subscribe(pose_topic,1,base_pose_cbk);
//    ros::Subscriber scan_sub = nh.subscribe("scan",1,scan_cbk);
    scan_t = nh.advertise<sm::LaserScan>("scan_map",1);
    scan_s = nh.advertise<sm::LaserScan>("scan_sim",1);

    pre_scan.ranges.clear();
    cur_scan.ranges.clear();


    // *** message filter
//    message_filters::Subscriber<sm::LaserScan> sync_scan_sub(nh, "scan", 1);
    message_filters::Subscriber<gm::PoseWithCovarianceStamped> sync_base_pose_sub(nh, "amcl_pose",1);
//    typedef  message_filters::sync_policies::ApproximateTime<sm::LaserScan, gm::PoseWithCovarianceStamped> My_Sync;

    message_filters::Subscriber<nav_msgs::Odometry> true_sync_base_pose_sub(nh, "base_pose_ground_truth",1);
    typedef  message_filters::sync_policies::ApproximateTime<gm::PoseWithCovarianceStamped, nav_msgs::Odometry> My_Sync;



    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<My_Sync> sync(My_Sync(10), sync_base_pose_sub, true_sync_base_pose_sub );
    sync.registerCallback(boost::bind(&locate2, _1, _2));


    ros::spin();


}