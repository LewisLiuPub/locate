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

// lib for test
Csm_Wrapper * csm_wrapper;
Laser_Simulator *gen_ptr;
FFT_Fitter *fft_fitter_ptr;

// tf listener must initialize after ros node init
tf::TransformListener *tf_listener_ptr;
tf::TransformBroadcaster* tfb_;
tf::Transform base_laser_tf;
gm::Pose laser_pose;

ros::Publisher scan_t, scan_s, pose_pub;

bool enable_csm ;
bool enable_fft;
bool tf_broadcast_ ;
bool pub_pose_;
string odom_frame_id_ ;
string base_frame_id_ ;
string global_frame_id_ ;
ros::Duration transform_tolerance_;
gm::PoseWithCovarianceStamped latest_pose_;

// stddev
float stddev_x, stddev_y, stddev_yaw;
float match_prob_thresh;

// Use a child class to get access to tf2::Buffer class inside of tf_
struct TransformListenerWrapper : public tf::TransformListener
{
    inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
};

TransformListenerWrapper* tf_;
/*compute nomal distribution pdf
 * */
double pdf_3d(std::valarray<double> sample, std::valarray<double> mean, std::valarray<double> stddev){
    double p = 1.0;
    sample = (sample - mean)/stddev;

    std::valarray<double> probs = double(1.0)/(sqrt(double(2.0 * M_PI)))* exp(sample*sample/double(-2.0));

    ROS_ERROR("get prob : %.2f, %.2f, %2f",probs[0], probs[1],probs[2]);
    for(int i=0; i < 3;i++)
        p*=probs[i];

    return p;


}

double check_pose_prob(geometry_msgs::Pose mean_pose, geometry_msgs::Pose sample_pose){

    double mean_[] = {mean_pose.position.x, mean_pose.position.y, tf::getYaw(mean_pose.orientation) };
    double stddev_[] = {stddev_x,stddev_y,stddev_yaw};

    double sample_[] = {sample_pose.position.x, sample_pose.position.y, tf::getYaw(sample_pose.orientation)};
    std::valarray<double> stddev(stddev_, 3), mean(mean_,3), sample(sample_, 3);

    double p = pdf_3d(sample, mean, stddev);
    return p;

}

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
    ROS_INFO("locate node wait for tf,%s, %s", base_frame.c_str(), laser_frame.c_str());
    tf::StampedTransform base_laser_tf_stamp;

    while (ros::ok()) {
        ros::Time t = ros::Time::now();
        try {

            tf_listener_ptr->waitForTransform(base_frame, laser_frame, t, ros::Duration(0.1));
            tf_listener_ptr->lookupTransform(base_frame, laser_frame, t, base_laser_tf_stamp);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("lookup transformation %s to %s failure: \n %s", base_frame.c_str(), laser_frame.c_str(), ex.what());
            continue;
        }

        base_laser_tf = tf::Transform(base_laser_tf_stamp);
        gm::Pose check_pose;
        tf::poseTFToMsg(base_laser_tf, check_pose);
        ROS_INFO("lookup transformation %s to %s  successful, get %f,%f,%f", base_frame.c_str(), laser_frame.c_str(),
                 check_pose.position.x, check_pose.position.y, check_pose.position.z);

        if (check_pose.position.z > 0.01  )
            break;

    }

}

void update_odom(gm::Pose base_pose){
    // subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
        tf::Transform tmp_tf;

        tf::poseMsgToTF(base_pose,tmp_tf);
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              latest_pose_.header.stamp,
                                              base_frame_id_);
        tf_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
    }
    catch(tf::TransformException)
    {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
    }

    tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));

    if (tf_broadcast_)
    {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = (latest_pose_.header.stamp +
                                          transform_tolerance_);
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                            transform_expiration,
                                            global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
    }
}

void locate(const sm::LaserScan::ConstPtr &scan_msg, const gm::PoseWithCovarianceStamped::ConstPtr &pose_msg){
    latest_pose_ = *pose_msg;

    ROS_INFO("sync!!!!!!");

    // base pose to laser pose
    tf::Transform fix_base_tf_;
    tf::poseMsgToTF(pose_msg->pose.pose, fix_base_tf_);
    // apply transform , get laser pose in fix frame
//    ROS_INFO("poseTFToMsg");
    tf::poseTFToMsg(fix_base_tf_ * base_laser_tf, laser_pose);


    sm::LaserScan::Ptr sensor_scan_ptr = gen_ptr->get_laser(laser_pose);

    sm::LaserScan scan_ref(*sensor_scan_ptr);


    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");


    gm::Pose base_pose;
    bool csm_ok = false;
    base_pose =pose_msg->pose.pose;
    if (enable_csm){
        ROS_INFO("start csm");
        csm_ok = csm_wrapper->get_base_pose(scan_ref,*scan_msg,base_pose, base_laser_tf);
        ROS_INFO("done csm");
        if (csm_ok){
            ROS_INFO("csm match ok!!!");

        }
    }

    // csm may fail without match failure, so the pose jumps
    // add check step, if error too big, cancle ffft, restore to amcl_pose
    if (csm_ok && enable_fft){
        ROS_INFO("start fft");
        // feed laser pose
        fft_fitter_ptr->get_base_pose(*scan_msg,base_pose,base_laser_tf);
        // return laser pose
        ROS_INFO("done fft");
    }

    // check diff between refine pose with amcl_pose
    double p = check_pose_prob(pose_msg->pose.pose,base_pose );

    ROS_ERROR("match prob : %f",p);

    if (p < match_prob_thresh){
        ROS_ERROR("restire to amcl pose");

        base_pose = pose_msg->pose.pose;
    }



    // piblish
    if (pub_pose_){
        ROS_INFO("refine_locate_node publish pose!");
        gm::PoseWithCovarianceStamped final_pose;
        final_pose.header.stamp = ros::Time::now();
        final_pose.header.frame_id = "map";
        final_pose.pose.pose = base_pose;
        pose_pub.publish(final_pose);
    }


    update_odom(base_pose);

    ROS_INFO("get amcl pose [%f,%f,%f] \n get icp pose [%f,%f,%f]",
             pose_msg->pose.pose.position.x,pose_msg->pose.pose.position.y,pose_msg->pose.pose.orientation.w,
             base_pose.position.x,base_pose.position.y,base_pose.orientation.w);
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
    sm::LaserScan scan_sens(*map_scan_ptr);



    sm::LaserScan::Ptr sensor_scan_ptr = gen_ptr->get_laser(laser_pose);
    sm::LaserScan scan_ref(*sensor_scan_ptr);

    scan_s.publish(scan_sens);
    scan_t.publish(scan_ref);






    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");


    gm::Pose base_pose;
    base_pose =pose_msg->pose.pose;
    // add first fft
//    fft_fitter_ptr->get_base_pose(scan_sens,base_pose,base_laser_tf);
//
//    sensor_scan_ptr = gen_ptr->get_laser(laser_pose);
//    sm::LaserScan scan_ref2(*sensor_scan_ptr);





    bool csm_ok = csm_wrapper->get_base_pose(scan_ref,scan_sens,base_pose, base_laser_tf);

    if (csm_ok){
        ROS_INFO("csm_ok");
        ROS_INFO("start fft");
        // feed laser pose
        fft_fitter_ptr->get_base_pose(scan_sens,base_pose,base_laser_tf);
        // return laser pose
        ROS_INFO("end fft");
    }else{
        ROS_INFO("csm_error");

    }

    // piblish
    gm::PoseWithCovarianceStamped final_pose;
    final_pose.header.stamp = ros::Time::now();
    final_pose.header.frame_id = "map";
    final_pose.pose.pose = base_pose;
    pose_pub.publish(final_pose);




    ROS_INFO("\n get true base pose [%f,%f,%f,%f,%f,%f]\nget amcl base  pose [%f,%f] \n get icp pose [%f,%f,%f,%f,%f,%f]",
             true_pose_msg->pose.pose.position.x,true_pose_msg->pose.pose.position.y,true_pose_msg->pose.pose.orientation.x,
             true_pose_msg->pose.pose.orientation.y,true_pose_msg->pose.pose.orientation.z,true_pose_msg->pose.pose.orientation.w,

             base_pose.position.x,base_pose.position.y,
             base_pose.position.x,base_pose.position.y,base_pose.orientation.x,
             base_pose.orientation.y,base_pose.orientation.z,base_pose.orientation.w
    );
}


int main(int argc, char **argv) {


    ROS_INFO("start locate node");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh, nh2, nh3;
    ros::NodeHandle nh_private("~");


    // parameters
    string  laser_frame_id_, pose_topic;



    if(!nh_private.getParam("base_frame_id",base_frame_id_))
        base_frame_id_ = "base_link";
    if(!nh_private.getParam("laser_frame_id",laser_frame_id_))
        laser_frame_id_ = "laser";
    if(!nh_private.getParam("pose_topic",pose_topic))
        pose_topic = "amcl_pose";
    if(!nh_private.getParam("enable_csm",enable_csm))
        enable_csm = true;
    if(!nh_private.getParam("enable_fft",enable_fft))
        enable_fft = true;
    double tmp_tol;
    if(!nh_private.getParam("transform_tolerance",tmp_tol))
        tmp_tol = 0.1;
    if(!nh_private.getParam("global_frame_id",global_frame_id_))
        global_frame_id_ = "map";
    if(!nh_private.getParam("odom_frame_id",odom_frame_id_))
        odom_frame_id_ = "odom";
    if(!nh_private.getParam("tf_broadcast",tf_broadcast_))
        tf_broadcast_ = true;
    if(!nh_private.getParam("pub_pose",pub_pose_))
        pub_pose_ = true;

    if(!nh_private.getParam("stddev_x",stddev_x))
        stddev_x = 0.1;
    if(!nh_private.getParam("stddev_y",stddev_y))
        stddev_y = 0.1;
    if(!nh_private.getParam("stddev_yaw",stddev_yaw))
        stddev_yaw = 0.1;
    if(!nh_private.getParam("match_prob_thresh",match_prob_thresh))
        match_prob_thresh = 0.3;


    // initlize lib
    csm_wrapper = new Csm_Wrapper(nh,nh_private) ;
    gen_ptr = new Laser_Simulator(nh2, nh_private);
    fft_fitter_ptr = new FFT_Fitter(nh3, nh_private);

    tf_listener_ptr = new(tf::TransformListener);
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new TransformListenerWrapper();

    transform_tolerance_.fromSec(tmp_tol);

    // get tf
    lookup_base_laser_tf(base_frame_id_,laser_frame_id_);

    // initialize simulator

//    ros::Subscriber base_pose_sub = nh.subscribe(pose_topic,1,base_pose_cbk);
//    ros::Subscriber scan_sub = nh.subscribe("scan",1,scan_cbk);
    scan_t = nh.advertise<sm::LaserScan>("scan_map",1);
    scan_s = nh.advertise<sm::LaserScan>("scan_sim",1);
    pose_pub = nh.advertise<gm::PoseWithCovarianceStamped>("refine_pose",1);



    // *** message filter
    message_filters::Subscriber<sm::LaserScan> sync_scan_sub(nh, "scan", 1);
    message_filters::Subscriber<gm::PoseWithCovarianceStamped> sync_base_pose_sub(nh, "amcl_pose",1);
    typedef  message_filters::sync_policies::ApproximateTime<sm::LaserScan, gm::PoseWithCovarianceStamped> PS_Sync;


    // stage simulator
//    message_filters::Subscriber<nav_msgs::Odometry> true_sync_base_pose_sub(nh, "base_pose_ground_truth",1);
//    typedef  message_filters::sync_policies::ApproximateTime<gm::PoseWithCovarianceStamped, nav_msgs::Odometry> My_Sync;
//    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    message_filters::Synchronizer<My_Sync> sync(My_Sync(10), sync_base_pose_sub, true_sync_base_pose_sub );
//    sync.registerCallback(boost::bind(&locate2, _1, _2));

    // robot
    message_filters::Synchronizer<PS_Sync> sync(PS_Sync(10), sync_scan_sub, sync_base_pose_sub);
    sync.registerCallback(boost::bind(&locate, _1, _2));

    ros::spin();


}