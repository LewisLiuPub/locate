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


    // **** initlize param
    init_params();

    // ***** initilize pointer
     // initlize lib
    csm_wrapper = new Csm_Wrapper(nh_,nh_private_) ;
    gen_ptr = new Laser_Simulator(nh_, nh_private_);
    fft_fitter_ptr = new FFT_Fitter(nh_, nh_private_);

    // tf
    tf_listener_ptr = new(tf::TransformListener);
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new TransformListenerWrapper();

    // callback queue
    ros::NodeHandle nh3_;
    nh3_.setCallbackQueue(&queue);
    ros::NodeHandle nh4_;
    nh4_.setCallbackQueue(&queue2);

    // ***** sub
    // synv tf with laser
    laserscan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 1);

    laserscan_filter_ =
            new tf::MessageFilter<sensor_msgs::LaserScan>(*laserscan_sub_,
                                                          *tf_,
                                                          odom_frame_id_,
                                                          5);
    laserscan_filter_->registerCallback(boost::bind(&Locate_Fusion::laserReceived,
                                                    this, _1));

    // new callback queue
    // map-odom tf from amcl
    amclTf_sub_ =  nh3_.subscribe("amcl_tf", 2, &Locate_Fusion::amcltfReceived, this);

    // set filter service
    set_filters_client_ = nh_.serviceClient<amcl::amcl_particles>("amcl/set_particles");

    // get current velocity
    vel_sub_ = nh_.subscribe(vel_topic_,1,&Locate_Fusion::velReceived, this);

    //initialPose_sub_;
    initialPose_sub_ = nh_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);
    //    initialPose_sub_ =nh3_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);
//    initialPose_sub_ = nh_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);


    // baselink pose from amcl
    amcl_pose_sub_ = nh_.subscribe("amcl_pose", 2, &Locate_Fusion::amclPoseReceived, this);


#if 1
    // patial cloud from amcl
    partial_cloud_sub_ = nh4_.subscribe("particlecloud",1,&Locate_Fusion::amclPartialReceived, this);
#endif


    // ***** pub
    pose_pub_ = nh.advertise<gm::PoseWithCovarianceStamped>("refine_pose",1);




    // init state
    odom_tf_update_ = false;
    init_pose_set_ = false;
    get_amcl_pose_ = false;
    get_amcl_tf_ = false;
    latest_amcl_map_odom_tf_.setIdentity();
    latest_final_pose_.header.stamp = ros::Time::now();

    match_count_ = 0;

    // **** get base laser tf
    lookup_base_laser_tf(base_frame_id_,laser_frame_id_);



    // start thread piblish tf
    ROS_INFO("create thread!!");
    boost::thread thread_update_tf(&Locate_Fusion::update_tf,this);

#if 0
    sync_scan_sub_ = new message_filters::Subscriber<sm::LaserScan>(nh_, "scan", 1);

    sync_base_pose_sub_ = new message_filters::Subscriber<gm::PoseWithCovarianceStamped> (nh_, "amcl_pose",1);

    sync_ = new message_filters::Synchronizer<PS_Sync> (PS_Sync(10), *sync_scan_sub_, *sync_base_pose_sub_);



    //sync_->registerCallback(boost::bind(&locate, _1, _2));
    sync_ ->registerCallback(boost::bind(&Locate_Fusion::pose_cbk,this ,_1, _2));
#endif


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
    if(!nh_private_.getParam("update_odom_chage",update_odom_chage_))
        update_odom_chage_ = false;
    if(!nh_private_.getParam("csm_match_valid_limit",csm_match_valid_limit_))
        csm_match_valid_limit_ = 0.4;
    if (!nh_private_.getParam("update_rate",rate)){
        rate = 15.0;
    }
    if(!nh_private_.getParam("pose_change_dist_min",pose_change_dist_min_))
        pose_change_dist_min_ = 0.2;
    if (!nh_private_.getParam("pose_change_angle_min",pose_change_angle_min_)){
        pose_change_angle_min_ = 0.1;
    }
    if (tmp_tol > 1.0/rate)
        tmp_tol = 1.0/rate;


    transform_tolerance_.fromSec(tmp_tol);


}

bool Locate_Fusion::lookup_tf(string frame1, string frame2, tf::Transform &transform_matrix,ros::Time t) {
    ROS_INFO("look up  for tf,%s, %s", frame1.c_str(), frame2.c_str());
    tf::StampedTransform tf_stamp;


    try {

        tf_listener_ptr->waitForTransform(frame1, frame2, t, ros::Duration(0.1));
        tf_listener_ptr->lookupTransform(frame1, frame2, t, tf_stamp);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("lookup transformation %s to %s failure: \n %s", frame1.c_str(), frame2.c_str(), ex.what());
        return false;
    }
    transform_matrix.setIdentity();

    transform_matrix = tf::Transform(tf_stamp);
    gm::Pose check_pose;
    tf::poseTFToMsg(transform_matrix, check_pose);
    ROS_INFO("lookup transformation %s to %s  successful, get [%f,%f,%f],[%f,%f,%f,%f]", frame1.c_str(), frame2.c_str(),
             check_pose.position.x, check_pose.position.y, check_pose.position.z, check_pose.orientation.x,check_pose.orientation.y,check_pose.orientation.z,check_pose.orientation.w);
    if (!isnan(tf::getYaw(check_pose.orientation)))
        return true;
    else
        return false;
}




void Locate_Fusion::lookup_base_laser_tf(const string &base_frame, const  string &laser_frame) {
    ROS_INFO("locate node wait for tf,%s, %s", base_frame.c_str(), laser_frame.c_str());
    tf::StampedTransform base_laser_tf_stamp;

    while (ros::ok()) {
        ros::Time t = ros::Time::now();
        bool tf_ok_ = lookup_tf(base_frame, laser_frame,base_laser_tf_, t);
        if (!tf_ok_)
            continue;
        gm::Pose check_pose;
        tf::poseTFToMsg(base_laser_tf_, check_pose);
        ROS_INFO("lookup transformation %s to %s  successful, get %f,%f,%f", base_frame.c_str(), laser_frame.c_str(),
                 check_pose.position.x, check_pose.position.y, check_pose.position.z);
        if (!isnan(check_pose.position.x ))
            break;
    }
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

void Locate_Fusion::update_odom(gm::Pose base_pose,ros::Time t) {
    // subtracting base to odom from map to base and send map to odom instead
    mutex.lock();
#if 0
    tf::Transform map_base_tf;
    tf::poseMsgToTF(base_pose,map_base_tf);


    latest_map_odom_tf_ = map_base_tf*latest_odom_base_tf.inverse();
#endif
#if 1

    ROS_ERROR("refine locate node update odom!!! get lock");
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
        tf::Transform tmp_tf;

        tf::poseMsgToTF(base_pose,tmp_tf);
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              t,
                                              base_frame_id_);
        tf_->transformPose(odom_frame_id_,
                           tmp_tf_stamped,
                           odom_to_map);
    }
    catch(tf::TransformException)
    {
        ROS_DEBUG("Failed to subtract base to odom transform");
        mutex.unlock();

        return;
    }

    tf::Transform odom_map_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                              tf::Point(odom_to_map.getOrigin()));
    latest_map_odom_tf_ = odom_map_tf.inverse();
#endif


    if (tf_broadcast_ && init_pose_set_)
    {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
//        ros::Time transform_expiration = (latest_pose_.header.stamp +
//                                          transform_tolerance_);
        ros::Time tn = ros::Time::now();
        ros::Time transform_expiration = (tn +
                                          transform_tolerance_);

        tf::StampedTransform tmp_tf_stamped(latest_map_odom_tf_,
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
        update_odom(pose_msg->pose.pose, latest_pose_.header.stamp);
        odom_tf_update_ = true;
    }

    mutex.lock();
    ROS_INFO("main callback get lock!");



    // call




    latest_pose_ = *pose_msg;

    ROS_INFO("get amcl pose !!!! \n [%.3f, %.3f, %.3f]",latest_pose_.pose.pose.position.x, latest_pose_.pose.pose.position.y, tf::getYaw(latest_pose_.pose.pose.orientation));
    if (isnan(latest_pose_.pose.pose.position.x)|| isnan(latest_pose_.pose.pose.position.y)||isnan(latest_pose_.pose.pose.position.z)){
        return;
    }
    if (!enable_csm_)
    {
        ROS_INFO("csm Not enable!! publish tf form amcl!!!");


        update_odom(pose_msg->pose.pose, latest_pose_.header.stamp);
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
        int csm_corr_valid_cnt;
        csm_error = csm_wrapper->get_base_pose(scan_ref,*scan_msg,base_pose, base_laser_tf_,csm_corr_valid_cnt);
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

    update_odom(latest_pose, latest_pose_.header.stamp);

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
            if(!odom_tf_update_ ){
                r.sleep();
                ROS_INFO("thread disable publish  tf!!!");
                continue;

            }

            mutex.lock();
            ROS_INFO("thread publish tf!!! get lock");

            // We want to send a transform that is good up until a
            // tolerance time so that odom can be used
            ros::Time tn = ros::Time::now();
            ros::Time transform_expiration = (tn +
                                              transform_tolerance_);

//            ROS_INFO("update odom by threads");
            tf::StampedTransform tmp_tf_stamped(latest_map_odom_tf_,
                                                transform_expiration,
                                                global_frame_id_, odom_frame_id_);


            tfb_->sendTransform(tmp_tf_stamped);
            mutex.unlock();
            r.sleep();


        }

    }

}

void Locate_Fusion::velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    return;
    latest_vel_ = *msg;



}

void Locate_Fusion::set_filter(vector<gm::Pose> cloud) {
    amcl::amcl_particles srv;
    srv.request.pose_array_msg.poses.clear();
    srv.request.pose_array_msg.poses = cloud;
    ROS_INFO("set filter cnt: %d",int(srv.request.pose_array_msg.poses.size()) );


    if (set_filters_client_.call(srv))
    {
        ROS_INFO("Sum: %ld", (long int)srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service set_filter");
    }
}

void Locate_Fusion::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    ROS_INFO("========\nget init pose!!");
    odom_tf_update_ = false;
    init_pose_set_ = true;

}
void Locate_Fusion::update_local_tf() {
    mutex.lock();

    latest_map_odom_tf_ = temp_map_odom_tf;
    mutex.unlock();


}

void Locate_Fusion::laserReceived(const sensor_msgs::LaserScanConstPtr &laser_scan) {


    // get latest laser scan
    latest_scan_ = *laser_scan;
//    odom_tf_update_ = false;

    // add odom_base change check
    // only change > thresh then start next match



    // get latest amcl tf as map_odom_tf or use latest_map_odom_tf
    //call callbackqueue
    odom_tf_update_ = false;

    get_amcl_tf_ = false;
    // Not receive amcl_tf data, increase walltime
    queue.callAvailable(ros::WallDuration(0.02));
    if (get_amcl_tf_){

        ROS_INFO("update latest map_odom tf from amcl!! get lock");
        temp_map_odom_tf = tf::Transform(latest_amcl_map_odom_tf_);
        update_local_tf();

        if (!init_pose_set_ ){
            ROS_INFO("No latest map_odom tf from amcl!! skip");
            odom_tf_update_ = true;
            init_pose_set_ = true;
            update_local_tf();

            return;
        }
    }
    if (!init_pose_set_ ){
        ROS_INFO("No latest map_odom tf from amcl!! skip");

        return;
    }

    if (!enable_csm_)
    {
        ROS_INFO("csm Not enable!! publish tf form amcl!!!");
        odom_tf_update_ = true;
        update_local_tf();

        return;

    }
    if (fabs(latest_vel_.angular.z)>vel_angular_min_){
        ROS_INFO("latest_vel_.angular.z : %.2f, disable csm",latest_vel_.angular.z );
        odom_tf_update_ = true;
        mutex.lock();

        return;
    }else{
        ROS_INFO("latest_vel_.angular.z : %.2f, enable csm",latest_vel_.angular.z );
    };


    if (isnan(latest_map_odom_tf_.getOrigin().x())|| isnan(latest_map_odom_tf_.getOrigin().x())||isnan(tf::getYaw(latest_map_odom_tf_.getRotation()))){
        ROS_INFO("get Nan tf ; skip!!");
        return;
    }
//    if (!init_pose_set_ ){
//        ROS_INFO("No latest map_odom tf from amcl!! skip");
//
//        return;
//    }

    // get latest odom, map->odom. odom->base
    if(!lookup_odom_base_tf(latest_odom_base_tf, latest_scan_.header.stamp))
    {
        ROS_ERROR("Couldn't determine robot's pose associated with laser scan!! skip");


        return;
    }

    ROS_INFO("get map to odom [%.3f,%.3f,%.3f]",latest_map_odom_tf_.getOrigin().x(), latest_map_odom_tf_.getOrigin().y(),tf::getYaw(latest_map_odom_tf_.getRotation())  );

    ROS_INFO("get odom to baselink [%.3f,%.3f,%.3f]",latest_odom_base_tf.getOrigin().x(), latest_odom_base_tf.getOrigin().y(),tf::getYaw(latest_odom_base_tf.getRotation()));
//    if (!init_pose_set_ || !get_amcl_pose_){
//        ROS_ERROR("init pose not set!! skip scan");
//        return;
//    }

    gm::Pose map_base_pose;
    tf::poseTFToMsg(latest_map_odom_tf_*tf::Transform(latest_odom_base_tf),map_base_pose);

    ROS_INFO("get amcl pose !!!! --> [%.3f, %.3f, %.3f]",map_base_pose.position.x, map_base_pose.position.y, tf::getYaw(map_base_pose.orientation));



    latest_pose_.pose.pose = map_base_pose;
    gm::Pose latest_pose = map_base_pose;

    // check odom_base change
    // update new odom->baselink
    tf::StampedTransform tx_odom;
    lookup_tf_change(odom_frame_id_,base_frame_id_,tx_odom,latest_final_pose_.header.stamp, latest_scan_.header.stamp );
    geometry_msgs::Pose pose_chage;
    tf::poseTFToMsg(tx_odom,pose_chage);
    double ds = sqrt(pose_chage.position.x*pose_chage.position.x + pose_chage.position.y*pose_chage.position.y)  ;
    double da = fabs(tf::getYaw(pose_chage.orientation) );
    ROS_INFO("odom_base movement [%.3f,%.3f]; stop match",ds,da);
    if (ds < pose_change_dist_min_ && da< pose_change_angle_min_){
        ROS_INFO("NO movement [%.3f,%.3f]; stop match!!-- enable thread ",ds,da);
        odom_tf_update_ = true;
        ROS_INFO("ffff set odom_tf_update_");
//        update_local_tf();
        latest_final_pose_.header.stamp = latest_scan_.header.stamp;


        return;
    }

    ROS_INFO("start match!!");





//    mutex.lock();






    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");

    double csm_prob, fft_prob;

    gm::Pose base_pose;
    double csm_error ;
    bool csm_ok = false;
    base_pose =map_base_pose;
    if (enable_csm_){
        // base pose to laser pose
        tf::Transform fix_base_tf_;
        gm::Pose laser_pose_;
        tf::poseMsgToTF(base_pose, fix_base_tf_);
        // apply transform , get laser pose in fix frame
        tf::poseTFToMsg(fix_base_tf_ * base_laser_tf_, laser_pose_);


        sm::LaserScan scan_ref = gen_ptr->get_laser(laser_pose_);
        if (scan_ref.ranges.empty()){
            ROS_ERROR("invalid laser pose!! cancle csm!!");
            return;
        }

        // check diff between refine pose with amcl_pose


        ROS_INFO("start csm");
        int corr_valid_cnt;
        csm_error = csm_wrapper->get_base_pose(scan_ref,*laser_scan,base_pose, base_laser_tf_, corr_valid_cnt);
        ROS_INFO("done csm");

#if 0
        // if enough match point, update latest_map_odom
        if (corr_valid_cnt > csm_match_valid_limit_*latest_scan_.ranges.size() )
            latest_map_odom_tf_ = temp_map_odom_tf;
#endif
        if (csm_error < csm_match_error_limit_ ){
            ROS_INFO("csm error : %.4f ,valid count:%d,  match ok!!!", csm_error,corr_valid_cnt);
            csm_ok = true;
        } else{
            ROS_INFO("csm error : %.4f, valid count:%d, match failure!!!", csm_error,corr_valid_cnt);
            match_count_ =0;


        }

        // check diff between refine pose with amcl_pose
        csm_prob = check_pose_prob(map_base_pose,base_pose );

        ROS_ERROR("match csm_prob : %f",csm_prob);

        if (!csm_ok || csm_prob < match_prob_thresh ){
            ROS_ERROR("csm restore to amcl pose");
            match_count_ =0;


            latest_pose = map_base_pose;
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
            ROS_INFO("fft error : %.4f match ok!!!", fft_error);
            fft_ok = true;
        } else{
            ROS_INFO("fft error : %.4f match failure!!!", fft_error);
            match_count_ =0;


        }


        // check diff between refine pose with amcl_pose
        fft_prob = check_pose_prob(map_base_pose,base_pose );

        ROS_ERROR("fftw match prob : %f",fft_prob);

        if (!fft_ok || fft_prob < match_prob_thresh){
            ROS_ERROR("fft restore to amcl pose");
            match_count_ =0;

        }else if (fft_ok && fft_prob > match_prob_thresh){
            latest_pose = base_pose;
            match_count_ ++;

        }
    }

//    if (update_odom_chage_){
//
//        // update new odom->baselink
//        tf::StampedTransform tx_odom;
//
//        lookup_tf_change(odom_frame_id_,base_frame_id_,tx_odom);
//
//        tf::Pose new_pose;
//        tf::poseMsgToTF(latest_pose,new_pose);
//        tf::poseTFToMsg(new_pose*tx_odom,latest_pose);
//    }







    // piblish
    if (pub_pose_){
        ROS_ERROR("===========\n refine_locate_node publish pose!\n ==========");
        latest_final_pose_.header.stamp = ros::Time::now();
        latest_final_pose_.header.frame_id = "map";
        latest_final_pose_.pose.pose = latest_pose;
        pose_pub_.publish(latest_final_pose_);
    }
//    mutex.unlock();



    ros::Time tn = ros::Time::now();
    update_odom(latest_pose, latest_scan_.header.stamp);
    odom_tf_update_ = true;


    // if match many times, update amcl filter
#if 0
    if (match_count_ > 5){

        queue2.callAvailable(ros::WallDuration(0.02));
        int cnt = latest_partial_cloud_.poses.size();
        ROS_ERROR("match ok many times! update amcl partial cloud, latest cnt :%d", cnt);

        if (cnt > 0){
            double dx = latest_pose.position.x - map_base_pose.position.x ;
            double dy = latest_pose.position.y - map_base_pose.position.y;
            for (int i = 0;i< cnt;i++){
                latest_partial_cloud_.poses[i].position.x += dx;
                latest_partial_cloud_.poses[i].position.y += dy;

            }
            set_filter(latest_partial_cloud_.poses);
            init_pose_set_ = false;
        }
        match_count_ = 0;


    }
#endif


    ROS_INFO("get amcl pose [%f,%f,%f] \n get refine pose [%f,%f,%f]",
             map_base_pose.position.x,map_base_pose.position.y,tf::getYaw(map_base_pose.orientation),
             latest_pose.position.x,latest_pose.position.y,tf::getYaw(latest_pose.orientation));


}

bool Locate_Fusion::lookup_odom_base_tf(tf::Transform &odom_base_pose,ros::Time t) {


    if (lookup_tf(odom_frame_id_,base_frame_id_,odom_base_pose, t))
        return true;
    else{
        return false;
    }
}

void Locate_Fusion::amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    ROS_INFO("get amcl pose!!!");

    get_amcl_pose_ = true;
}

// receive pose as tf
void Locate_Fusion::amcltfReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    ROS_ERROR("\n==========\n get amcl tf!!!\n==========[%.3f,%.3f,%.3f]",
              msg->pose.pose.position.x,msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));

    latest_amcl_map_odom_tf_.stamp_ = msg->header.stamp;
    tf::poseMsgToTF(msg->pose.pose,latest_amcl_map_odom_tf_);
    get_amcl_tf_ = true;


}
/* target-> update map->odom transformation
 * 1,sync laser_scan with odom->baselink transformation, register lasercallback;
 * 2.listen to amcl_pose;
 * 3.listen to vel_cmd, compute latest_ave_vel
 * 4.during lasercallback, check amcl_pose, update latest_map_odom_tf;
 * 5.after icp_match, interplate latest_pose to current pose according to latest_ave_vel
 * 6.
 * */



void Locate_Fusion::lookup_tf_change(string frame1, string frame2, tf::StampedTransform &tx_odom,ros::Time t_latst, ros::Time t_now) {
    try
    {
        ros::Time now = ros::Time::now();
        // wait a little for the latest tf to become available
        tf_->waitForTransform(frame2, t_latst,
                              frame2, t_now,
                              frame1, ros::Duration(0.5));
        tf_->lookupTransform(frame2, t_latst,
                             frame2, t_now,
                             frame1, tx_odom);
    }
    catch(tf::TransformException e)
    {
        // If we've never sent a transform, then this is normal, because the
        // global_frame_id_ frame doesn't exist.  We only care about in-time
        // transformation for on-the-move pose-setting, so ignoring this
        // startup condition doesn't really cost us anything.
        ROS_WARN("lookup_tf_change Failed to transform initial pose in time (%s)", e.what());
        tx_odom.setIdentity();
    }

}

/*update amcl partial filters
 * when rotate
 * or when match error is low
 * */
void Locate_Fusion::amclPartialReceived(const geometry_msgs::PoseArrayConstPtr &msg) {
    latest_partial_cloud_ = *msg;

    return;
    double reset_partial_rotate_vel_min_ = 0.4;

    int partial_cnt = int(latest_partial_cloud_.poses.size());

    ROS_ERROR("get amcl partial cloud count %d",partial_cnt);

    bool update_partial = fabs(latest_vel_.angular.z)>reset_partial_rotate_vel_min_;


    if (update_partial){
        if (partial_cnt< 1500){
            for(int i =0;i<partial_cnt;i++){
                geometry_msgs::Pose p = latest_partial_cloud_.poses[i];
                double yaw = tf::getYaw(p.orientation);
                tf::Pose P;
                tf::poseMsgToTF(p,P);
                latest_partial_cloud_.poses.push_back(p);
                latest_partial_cloud_.poses.push_back(p);


            }
        }

        // given diffrent filter number , take diffrent update function






    } else
        return;
}
