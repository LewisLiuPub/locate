//
// Created by waxz on 18-4-18.
//

#include <locate/locate.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "amcl/amcl_particles.h"

using  namespace std;

// mutex
boost::mutex io_mutex;
boost::mutex mutex;

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




Locate_Fusion::Locate_Fusion(ros::NodeHandle nh,ros::NodeHandle nh_private):nh_private_(nh_private), nh_(nh){


    // initlize lib
    csm_wrapper = new Csm_Wrapper(nh_,nh_private_) ;
    gen_ptr = new Laser_Simulator(nh_, nh_private_);
    fft_fitter_ptr = new FFT_Fitter(nh_, nh_private_);

    tf_listener_ptr = new(tf::TransformListener);
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new TransformListenerWrapper();

    ros::NodeHandle nh3_;
    nh3_.setCallbackQueue(&queue);



    init_params();
    odom_tf_update_ = false;
    init_pose_set_ = false;
    get_amcl_pose_ = false;
    lookup_base_laser_tf(base_frame_id_,laser_frame_id_);

    //

    pose_pub_ = nh.advertise<gm::PoseWithCovarianceStamped>("refine_pose",1);
#if 0
    sync_scan_sub_ = new message_filters::Subscriber<sm::LaserScan>(nh_, "scan", 1);

    sync_base_pose_sub_ = new message_filters::Subscriber<gm::PoseWithCovarianceStamped> (nh_, "amcl_pose",1);

    sync_ = new message_filters::Synchronizer<PS_Sync> (PS_Sync(10), *sync_scan_sub_, *sync_base_pose_sub_);



    //sync_->registerCallback(boost::bind(&locate, _1, _2));
    sync_ ->registerCallback(boost::bind(&Locate_Fusion::pose_cbk,this ,_1, _2));
#endif

    // synv tf with laser
    laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 1);
    laser_scan_filter_ =
            new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                          *tf_,
                                                          odom_frame_id_,
                                                          100);
    laser_scan_filter_->registerCallback(boost::bind(&Locate_Fusion::laserReceived,
                                                     this, _1));


    // ++++++
    vel_sub_ = nh_.subscribe(vel_topic_,1,&Locate_Fusion::current_vel_cbk, this);

    set_filters_client_ = nh_.serviceClient<amcl::amcl_particles>("amcl/set_particles");

    //initial_pose_sub_;
    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);
    amcl_pose_sub_ = nh_.subscribe("amcl_pose", 2, &Locate_Fusion::amclPoseReceived, this);

//
//    initial_pose_sub_ =nh3_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);
//    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);




    // start thread
    ROS_INFO("create thread!!");
    boost::thread thread_update_tf(&Locate_Fusion::update_tf,this);
//    thread_update_tf.join();




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
    if(!nh_private_.getParam("csm_match_error_limit",csm_match_error_limit_))
        csm_match_error_limit_ = 0.5;
    if(!nh_private_.getParam("fft_match_error_limit",fft_match_error_limit_))
        fft_match_error_limit_ = 0.5;
    if(!nh_private_.getParam("tf_update_rate",tf_update_rate_))
        tf_update_rate_ = 5;
    if(!nh_private_.getParam("vel_topic",vel_topic_))
        vel_topic_ = "/mobile_base/commands/velocity";
    if(!nh_private_.getParam("vel_angular_min",vel_angular_min_))
        vel_angular_min_ = 0.2;


    transform_tolerance_.fromSec(tmp_tol);


}


double Locate_Fusion::check_pose_prob(geometry_msgs::Pose mean_pose, geometry_msgs::Pose sample_pose){

    double mean_[] = {mean_pose.position.x, mean_pose.position.y, tf::getYaw(mean_pose.orientation) };
    double stddev_[] = {stddev_x,stddev_y,stddev_yaw};

    double sample_[] = {sample_pose.position.x, sample_pose.position.y, tf::getYaw(sample_pose.orientation)};

    ROS_INFO("get data mean[%.3f, %.3f,%.3f], sample :[%.3f, %.3f,%.3f] ",mean_[0],mean_[1],mean_[2],sample_[0],sample_[1],sample_[2]  );
    std::valarray<double> stddev(stddev_, 3), mean(mean_,3), sample(sample_, 3);

    double p = pdf_3d(sample, mean, stddev);
    return p;

}

void Locate_Fusion::lookup_base_laser_tf(const string &base_frame, const  string &laser_frame) {
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

        base_laser_tf_ = tf::Transform(base_laser_tf_stamp);
        gm::Pose check_pose;
        tf::poseTFToMsg(base_laser_tf_, check_pose);
        ROS_INFO("lookup transformation %s to %s  successful, get %f,%f,%f", base_frame.c_str(), laser_frame.c_str(),
                 check_pose.position.x, check_pose.position.y, check_pose.position.z);

        if (check_pose.position.z > 0.01  )
            break;

    }

}


void Locate_Fusion::update_odom(gm::Pose base_pose) {
    // subtracting base to odom from map to base and send map to odom instead
    mutex.lock();
    ROS_ERROR("refine locate node update odom!!!");
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
        tf::Transform tmp_tf;

        tf::poseMsgToTF(base_pose,tmp_tf);
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              latest_scan_.header.stamp,
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

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                             tf::Point(odom_to_map.getOrigin()));

    if (tf_broadcast_ && init_pose_set_)
    {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
//        ros::Time transform_expiration = (latest_pose_.header.stamp +
//                                          transform_tolerance_);
        ros::Time tn = ros::Time::now();
        ros::Time transform_expiration = (tn +
                                          transform_tolerance_);

        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                            transform_expiration,
                                            global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);

    }
    mutex.unlock();

}


void Locate_Fusion::pose_cbk(const sm::LaserScan::ConstPtr &scan_msg,
                             const gm::PoseWithCovarianceStamped::ConstPtr &pose_msg) {


    if (fabs(latest_vel_.angular.z)>vel_angular_min_){
        ROS_INFO("latest_vel_.angular.z : %.2f, disable csm",latest_vel_.angular.z );
        enable_csm_ = false;
    }else{
        ROS_INFO("latest_vel_.angular.z : %.2f, enable csm",latest_vel_.angular.z );
        enable_csm_ = true;

    };

//    queue.callAvailable(ros::WallDuration(0.01));
    if (init_pose_set_ && !odom_tf_update_){
        update_odom(pose_msg->pose.pose);
        odom_tf_update_ = true;
    }

    mutex.lock();



    // call




    latest_pose_ = *pose_msg;

    ROS_INFO("get amcl pose !!!! \n [%.3f, %.3f, %.3f]",latest_pose_.pose.pose.position.x, latest_pose_.pose.pose.position.y, tf::getYaw(latest_pose_.pose.pose.orientation));
    if (isnan(latest_pose_.pose.pose.position.x)|| isnan(latest_pose_.pose.pose.position.y)||isnan(latest_pose_.pose.pose.position.z)){
        return;
    }
    if (!enable_csm_)
    {
        ROS_INFO("csm Not enable!! publish tf form amcl!!!");


        update_odom(pose_msg->pose.pose);
        return;

    }



    gm::Pose latest_pose = pose_msg->pose.pose;


    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");

    double p, csm_prob, fft_prob;

    gm::Pose base_pose;
    double csm_error ;
    bool csm_ok = false;
    base_pose =pose_msg->pose.pose;
    if (enable_csm_){
        // base pose to laser pose
        tf::Transform fix_base_tf_;
        gm::Pose laser_pose_;
        tf::poseMsgToTF(pose_msg->pose.pose, fix_base_tf_);
        // apply transform , get laser pose in fix frame
        tf::poseTFToMsg(fix_base_tf_ * base_laser_tf_, laser_pose_);


        sm::LaserScan scan_ref = gen_ptr->get_laser(laser_pose_);
        if (scan_ref.ranges.empty()){
            ROS_ERROR("invalid laser pose!! cancle csm!!");
            return;
        }

        // check diff between refine pose with amcl_pose


        ROS_INFO("start csm");
        csm_error = csm_wrapper->get_base_pose(scan_ref,*scan_msg,base_pose, base_laser_tf_);
        ROS_INFO("done csm");
        if (csm_error < csm_match_error_limit_ ){
            ROS_INFO("csm error : %.2f match ok!!!", csm_error);
            csm_ok = true;
        } else{
            ROS_INFO("csm error : %.2f match failure!!!", csm_error);

        }

        // check diff between refine pose with amcl_pose
        csm_prob = check_pose_prob(pose_msg->pose.pose,base_pose );

        ROS_ERROR("match csm_prob : %f",csm_prob);

        if (!csm_ok || csm_prob < match_prob_thresh ){
            ROS_ERROR("csm restore to amcl pose");

            latest_pose = pose_msg->pose.pose;
        }else if (csm_ok && csm_prob > match_prob_thresh){
            latest_pose = base_pose;
        }

    }


    // csm may fail without match failure, so the pose jumps
    // add check step, if error too big, cancle ffft, restore to amcl_pose
    bool fft_ok = false;

    if (csm_ok && enable_fft_){

        ROS_INFO("start fft");
        // feed laser pose
        double fft_error = fft_fitter_ptr->get_base_pose(*scan_msg,base_pose,base_laser_tf_);
        // return laser pose
        ROS_INFO("done fft");
        if (fft_error < fft_match_error_limit_ ){
            ROS_INFO("fft error : %.2f match ok!!!", fft_error);
            fft_ok = true;
        } else{
            ROS_INFO("fft error : %.2f match failure!!!", fft_error);

        }


        // check diff between refine pose with amcl_pose
        p = check_pose_prob(pose_msg->pose.pose,base_pose );

        ROS_ERROR("fftw match prob : %f",p);

        if (!fft_ok || p < match_prob_thresh){
            ROS_ERROR("fft restore to amcl pose");

        }else if (fft_ok && p > match_prob_thresh){
            latest_pose = base_pose;

        }
    }





    // piblish
    if (pub_pose_){
        ROS_ERROR("===========\n refine_locate_node publish pose!\n ==========");
        gm::PoseWithCovarianceStamped final_pose;
        final_pose.header.stamp = ros::Time::now();
        final_pose.header.frame_id = "map";
        final_pose.pose.pose = latest_pose;
        pose_pub_.publish(final_pose);
    }


    mutex.unlock();

    update_odom(latest_pose);

    ROS_INFO("get amcl pose [%f,%f,%f] \n get refine pose [%f,%f,%f]",
             pose_msg->pose.pose.position.x,pose_msg->pose.pose.position.y,tf::getYaw(pose_msg->pose.pose.orientation),
             latest_pose.position.x,latest_pose.position.y,tf::getYaw(latest_pose.orientation));

}

void Locate_Fusion::update_tf() {

    boost::mutex::scoped_lock
            lock(io_mutex);

    ros::Rate r(tf_update_rate_);

    if (tf_broadcast_)
    {
        while (ros::ok()){
            if(!odom_tf_update_ || !init_pose_set_){
                continue;

            }

            mutex.lock();
            ROS_INFO("thread publish tf!!!");

            // We want to send a transform that is good up until a
            // tolerance time so that odom can be used
            ros::Time tn = ros::Time::now();
            ros::Time transform_expiration = (tn +
                                              transform_tolerance_);

//            ROS_INFO("update odom by threads");
            tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                                transform_expiration,
                                                global_frame_id_, odom_frame_id_);


            tfb_->sendTransform(tmp_tf_stamped);
            mutex.unlock();
            r.sleep();


        }

    }

}

void Locate_Fusion::current_vel_cbk(const geometry_msgs::Twist::ConstPtr &msg) {
    return;
    latest_vel_ = *msg;



}

void Locate_Fusion::set_filter() {
    amcl::amcl_particles srv;
    vector<gm::Pose> s;
    srv.request.pose_array_msg.poses = s;


    if (set_filters_client_.call(srv))
    {
        ROS_INFO("Sum: %ld", (long int)srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
    }
}

void Locate_Fusion::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    ROS_INFO("========\nget init pose!!");
    odom_tf_update_ = false;
    init_pose_set_ = true;

}

void Locate_Fusion::laserReceived(const sensor_msgs::LaserScanConstPtr &laser_scan) {
    latest_scan_ = *laser_scan;
    tf::Stamped<tf::Pose> odom_pose;
    if (!init_pose_set_ || !get_amcl_pose_){
        ROS_ERROR("init pose not set!! skip scan");
        return;
    }
    if(!get_baselink(laser_scan, odom_pose))
    {
        ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
        return;
    }

    if (fabs(latest_vel_.angular.z)>vel_angular_min_){
        ROS_INFO("latest_vel_.angular.z : %.2f, disable csm",latest_vel_.angular.z );
        enable_csm_ = false;
    }else{
        ROS_INFO("latest_vel_.angular.z : %.2f, enable csm",latest_vel_.angular.z );
        enable_csm_ = true;

    };

//    queue.callAvailable(ros::WallDuration(0.01));
    gm::Pose pose_msg ;
    tf::poseTFToMsg(odom_pose,pose_msg);

    if (init_pose_set_ && !odom_tf_update_ && get_amcl_pose_){
        ros::Rate(5).sleep();
        get_baselink(laser_scan, odom_pose);
        tf::poseTFToMsg(odom_pose,pose_msg);
        update_odom(pose_msg);
        odom_tf_update_ = true;
        ROS_ERROR("get fisrt tf, publish !!!");
    }

    mutex.lock();

    latest_pose_.pose.pose = pose_msg;

    ROS_INFO("get amcl pose !!!! \n [%.3f, %.3f, %.3f]",latest_pose_.pose.pose.position.x, latest_pose_.pose.pose.position.y, tf::getYaw(latest_pose_.pose.pose.orientation));
    if (isnan(latest_pose_.pose.pose.position.x)|| isnan(latest_pose_.pose.pose.position.y)||isnan(latest_pose_.pose.pose.position.z)){
        return;
    }
    if (!enable_csm_)
    {
        ROS_INFO("csm Not enable!! publish tf form amcl!!!");


        update_odom(pose_msg);
        return;

    }



    gm::Pose latest_pose = pose_msg;


    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");

    double p, csm_prob, fft_prob;

    gm::Pose base_pose;
    double csm_error ;
    bool csm_ok = false;
    base_pose =pose_msg;
    if (enable_csm_){
        // base pose to laser pose
        tf::Transform fix_base_tf_;
        gm::Pose laser_pose_;
        tf::poseMsgToTF(pose_msg, fix_base_tf_);
        // apply transform , get laser pose in fix frame
        tf::poseTFToMsg(fix_base_tf_ * base_laser_tf_, laser_pose_);


        sm::LaserScan scan_ref = gen_ptr->get_laser(laser_pose_);
        if (scan_ref.ranges.empty()){
            ROS_ERROR("invalid laser pose!! cancle csm!!");
            return;
        }

        // check diff between refine pose with amcl_pose


        ROS_INFO("start csm");
        csm_error = csm_wrapper->get_base_pose(scan_ref,*laser_scan,base_pose, base_laser_tf_);
        ROS_INFO("done csm");
        if (csm_error < csm_match_error_limit_ ){
            ROS_INFO("csm error : %.2f match ok!!!", csm_error);
            csm_ok = true;
        } else{
            ROS_INFO("csm error : %.2f match failure!!!", csm_error);

        }

        // check diff between refine pose with amcl_pose
        csm_prob = check_pose_prob(pose_msg,base_pose );

        ROS_ERROR("match csm_prob : %f",csm_prob);

        if (!csm_ok || csm_prob < match_prob_thresh ){
            ROS_ERROR("csm restore to amcl pose");

            latest_pose = pose_msg;
        }else if (csm_ok && csm_prob > match_prob_thresh){
            latest_pose = base_pose;
        }

    }


    // csm may fail without match failure, so the pose jumps
    // add check step, if error too big, cancle ffft, restore to amcl_pose
    bool fft_ok = false;

    if (csm_ok && enable_fft_){

        ROS_INFO("start fft");
        // feed laser pose
        double fft_error = fft_fitter_ptr->get_base_pose(*laser_scan,base_pose,base_laser_tf_);
        // return laser pose
        ROS_INFO("done fft");
        if (fft_error < fft_match_error_limit_ ){
            ROS_INFO("fft error : %.2f match ok!!!", fft_error);
            fft_ok = true;
        } else{
            ROS_INFO("fft error : %.2f match failure!!!", fft_error);

        }


        // check diff between refine pose with amcl_pose
        p = check_pose_prob(pose_msg,base_pose );

        ROS_ERROR("fftw match prob : %f",p);

        if (!fft_ok || p < match_prob_thresh){
            ROS_ERROR("fft restore to amcl pose");

        }else if (fft_ok && p > match_prob_thresh){
            latest_pose = base_pose;

        }
    }





    // piblish
    if (pub_pose_){
        ROS_ERROR("===========\n refine_locate_node publish pose!\n ==========");
        gm::PoseWithCovarianceStamped final_pose;
        final_pose.header.stamp = ros::Time::now();
        final_pose.header.frame_id = "map";
        final_pose.pose.pose = latest_pose;
        pose_pub_.publish(final_pose);
    }
    mutex.unlock();



    update_odom(latest_pose);

    ROS_INFO("get amcl pose [%f,%f,%f] \n get refine pose [%f,%f,%f]",
             pose_msg.position.x,pose_msg.position.y,tf::getYaw(pose_msg.orientation),
             latest_pose.position.x,latest_pose.position.y,tf::getYaw(latest_pose.orientation));

}

bool Locate_Fusion::get_baselink(const sensor_msgs::LaserScanConstPtr &laser_scan, tf::Stamped<tf::Pose> &odom_pose) {

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                               tf::Vector3(0,0,0)), laser_scan->header.stamp, base_frame_id_);
    try
    {
        this->tf_->transformPose("map", ident, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    return true;
}

void Locate_Fusion::amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    get_amcl_pose_ = true;
}

/* target-> update map->odom transformation
 * 1,sync laser_scan with odom->baselink transformation, register lasercallback;
 * 2.listen to amcl_pose;
 * 3.listen to vel_cmd, compute latest_ave_vel
 * 4.during lasercallback, check amcl_pose, update latest_map_odom_tf;
 * 5.after icp_match, interplate latest_pose to current pose according to latest_ave_vel
 * 6.
 * */