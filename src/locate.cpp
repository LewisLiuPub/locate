//
// Created by waxz on 18-4-18.
//

#include <locate/locate.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "amcl/amcl_particles.h"
#include <boost/assign.hpp>
#include "TinyMatrix/TinyMatrix.h"
#include <cmath>
using namespace TinyMatrix;
using  namespace std;
#define use_cache false
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
    ros::NodeHandle nh5_;
    nh4_.setCallbackQueue(&queue3);

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
    initialPose_sub_ = nh5_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);
    //    initialPose_sub_ =nh3_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);
//    initialPose_sub_ = nh_.subscribe("initialpose", 2, &Locate_Fusion::initialPoseReceived, this);


    // baselink pose from amcl
    amcl_pose_sub_ = nh_.subscribe("amcl_pose", 2, &Locate_Fusion::amclPoseReceived, this);


#if 1
    // patial cloud from amcl
    partial_cloud_sub_ = nh4_.subscribe("particlecloud",1,&Locate_Fusion::amclPartialReceived, this);
#endif


    // ***** pub
    pose_pub_ = nh_.advertise<gm::PoseWithCovarianceStamped>("refine_pose",1);




    // init state
    odom_tf_update_ = false;
    init_pose_set_ = false;
    get_amcl_pose_ = false;
    get_amcl_tf_ = false;
    latest_amcl_map_odom_tf_.setIdentity();
    base_laser_tf_.setIdentity();
    latest_map_odom_tf_.setIdentity();

    latest_final_pose_.header.stamp = ros::Time::now();

    match_count_ = 0;
    nomove_cnt_ = 0;





    // start thread piblish tf
    ROS_INFO("create thread!!");
//    boost::thread thread_update_tf(&Locate_Fusion::update_tf,this);
    boost::thread(boost::bind(&Locate_Fusion::update_tf, this));

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
    if (!nh_private_.getParam("update_rate",rate))
        rate = 15.0;

    if(!nh_private_.getParam("pose_change_dist_min",pose_change_dist_min_))
        pose_change_dist_min_ = 0.2;
    if (!nh_private_.getParam("max_partial_number",max_partial_number_))
        max_partial_number_ = 5000 ;

    if (!nh_private_.getParam("point_on_circle",point_on_circle_))
        point_on_circle_ = 500;

    if (!nh_private_.getParam("radius_max",radius_max_))
        radius_max_ = 0.1 ;

    if (!nh_private_.getParam("pose_change_angle_min",pose_change_angle_min_))
        pose_change_angle_min_ = 0.1;

    if (!nh_private_.getParam("reset_amcl_filter_cnt",reset_amcl_filter_cnt_))
        reset_amcl_filter_cnt_ = 5;
    if (!nh_private_.getParam("reset_x_cov",reset_x_cov_))
        reset_x_cov_ = 0.15;
    if (!nh_private_.getParam("reset_y_cov",reset_y_cov_))
        reset_y_cov_ = 0.15;
    if (!nh_private_.getParam("reset_yaw_cov",reset_yaw_cov_))
        reset_yaw_cov_ = 0.04;

    if (!nh_private_.getParam("switch_source",switch_source_))
        switch_source_ = 0.3;

    if (tmp_tol > 1.0/rate)
        tmp_tol = 1.0/rate;


    transform_tolerance_.fromSec(tmp_tol);


}

Locate_Fusion::~Locate_Fusion() {
    delete csm_wrapper;
    delete gen_ptr;
    delete fft_fitter_ptr;
    delete tf_listener_ptr;
    delete tfb_;
    delete tf_;
    delete laserscan_sub_;
    delete laserscan_filter_;

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
    if (!std::isnan(tf::getYaw(check_pose.orientation)))
        return true;
    else
        return false;
}




bool Locate_Fusion::lookup_base_laser_tf(const string &base_frame, const  string &laser_frame) {
    ROS_INFO("locate node wait for tf,%s, %s", base_frame.c_str(), laser_frame.c_str());
    tf::StampedTransform base_laser_tf_stamp;

    ros::Time t = ros::Time::now();

    bool tf_ok_ = lookup_tf(base_frame, laser_frame,base_laser_tf_, t);

    return tf_ok_;
#if 0


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
#endif
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
#if 1
    tf::Transform map_base_tf;
    tf::poseMsgToTF(base_pose,map_base_tf);


    latest_map_odom_tf_ = map_base_tf*latest_odom_base_tf.inverse();
#endif
#if 0

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


void Locate_Fusion::update_tf() {

    boost::mutex::scoped_lock
            lock(io_mutex);

    ros::Rate r(tf_update_rate_);

    if (tf_broadcast_)
    {
        while (ros::ok()){
//            if(!odom_tf_update_ ){
//                r.sleep();
////                ROS_INFO("thread disable publish  tf!!!");
//                continue;
//
//            }

            mutex.lock();
//            ROS_INFO("thread publish tf!!! get lock");

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
    latest_vel_ = *msg;
}

void Locate_Fusion::set_filter(gm::Pose latest_pose,double x_cov = 0.5,  double y_cov = 0.5, double yaw_cov = 0.06, double cloud_yaw = 0.0) {
    amcl::amcl_particles srv;

    // add pse array
    srv.request.pose_array_msg.poses.clear();
//    srv.request.pose_array_msg.poses.push_back(latest_pose);

    // add initial pose
    geometry_msgs::PoseWithCovariance initial_pose;
    initial_pose.pose = latest_pose;

    double yaw = (cloud_yaw != 0.0) ? cloud_yaw:tf::getYaw(latest_pose.orientation);
    Matrix<double,2,2> mat_1 = {
            cos(yaw),cos(yaw+0.5*M_PI),
            sin(yaw),sin(yaw+0.5*M_PI)
    };
    Matrix<double,2,2> mat_2 = {
            x_cov,0,
            0,y_cov
    };
    Matrix<double,2,2> mat_3 = mat_1.Transpose();

    Matrix<double,2,2> mat_4 = mat_1*mat_2*mat_3;



    initial_pose.covariance= boost::assign::list_of
            (mat_4(0,0)) (mat_4(0,1))  (0)  (0)  (0)  (0)
            (mat_4(1,0))  (mat_4(1,1)) (0)  (0)  (0)  (0)
            (0)  (0)  (0) (0)  (0)  (0)
            (0)  (0)  (0)  (0) (0)  (0)
            (0)  (0)  (0)  (0)  (0) (0)
            (0) (0) (0) (0) (0) (yaw_cov);

    srv.request.initial_pose.header.stamp = ros::Time::now();
    srv.request.initial_pose.header.frame_id = "map";
    srv.request.initial_pose.pose = initial_pose;



#if 0
    int point_cnt = 0;
    int circle_num = max_partial_number_/point_on_circle_;
    gm::Pose p;

    p.orientation = latest_pose.orientation;
    double o_x = latest_pose.position.x;
    double o_y = latest_pose.position.y;

//    for (double r = 0.0;r<r_max;r+=r_max/circle_num){

    for (int  k = 0 ;k<circle_num; k++){
        double r = k*radius_max_/circle_num;

        for (int t =0;t<point_on_circle_;t++){
            p.position.x = o_x + r*cos(2*M_PI*t/point_on_circle_);
            p.position.y = o_y + r*sin(2*M_PI*t/point_on_circle_);
            srv.request.pose_array_msg.poses.push_back(p);
            point_cnt ++;
            if (point_cnt == max_partial_number_)
                break;
        }
        if (point_cnt == max_partial_number_)
            break;
    }
#endif

    ROS_INFO("set filter cnt: %d",int(srv.request.pose_array_msg.poses.size()) );


    if (set_filters_client_.call(srv))
    {
        ROS_INFO(" call service ok!: %ld", (long int)srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service set_filter");
    }
}

void Locate_Fusion::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    ROS_INFO("========\nget init pose!!");
    odom_tf_update_ = false;
    init_pose_set_ = false;
#if 0
    // test set filter
    gm::Pose test_pose;
    test_pose.position.x = 2.0;
    test_pose.position.y = 3.0;
    tf::Pose P;
    tf::Quaternion q;
    q.setRPY(0.0,0.0,0.2);
    P.setRotation(q);
    P.setOrigin(tf::Vector3(2.0,2.0,0));

    tf::poseTFToMsg(P,test_pose);

    ros::Rate(1).sleep();
    set_filter(msg->pose.pose,0.5,  0.5, 0.06);

    return;
#endif
}
void Locate_Fusion::update_local_tf() {
    mutex.lock();

    if (!std::isnan(temp_map_odom_tf.getOrigin().x()))
        latest_map_odom_tf_ = temp_map_odom_tf;
    mutex.unlock();
    odom_tf_update_ = true;



}

void Locate_Fusion::laserReceived(const sensor_msgs::LaserScanConstPtr &laser_scan) {
    // get amcl_tf, set init pose
    // when to update new tf

    //1) get init pose --> update new tf
    //2) if match point num > thresh --> update new tf

    latest_scan_ = *laser_scan;


    // 1)
    //call callbackqueue
    odom_tf_update_ = false;
    get_amcl_tf_ = false;
//    init_pose_set_ = false;
    // Not receive amcl_tf data, increase walltime
    queue.callAvailable(ros::WallDuration(0.02));
    // get latest amcl tf as map_odom_tf or use latest_map_odom_tf
    if (get_amcl_tf_){

        ROS_INFO("update latest map_odom tf from amcl!! get lock");
        temp_map_odom_tf = tf::Transform(latest_amcl_map_odom_tf_);
        nomove_cnt_ = (match_count_ < 1) ? 0: nomove_cnt_;

        // if last time not match
        // update rule 1： if long time match failure
        if (match_count_ < -reset_amcl_filter_cnt_){
            ROS_INFO("update amcl tf !! for long time no match");





            if (match_count_ < -10 *reset_amcl_filter_cnt_){
                match_count_ = -1*reset_amcl_filter_cnt_;
            }
            if (match_count_ > 10 *reset_amcl_filter_cnt_){
                match_count_ = 1*reset_amcl_filter_cnt_;
            }

        }

        queue3.callAvailable(ros::WallDuration(0.02));

        if (!init_pose_set_ ){
            ROS_INFO("update amcl tf !! for  set init pose");

            odom_tf_update_ = true;
            init_pose_set_ = true;
            // update rule 2: get initial_pose
            // if get init_pose , update amcl_tf to latest_map_odom_tf
            update_local_tf();

            return;
        }

    }


    // enable csm from rosparam
    if (!enable_csm_)
    {
        ROS_INFO("update amcl tf !! csm Not enable!!");
        odom_tf_update_ = true;
        // update rule 3: diable csm
        update_local_tf();
        return;
    }




    // update rule 4: rotate too fast
    if (fabs(latest_vel_.angular.z)>vel_angular_min_){
        ROS_INFO("update amcl tf !! latest_vel_.angular.z : %.2f, disable csm",latest_vel_.angular.z );
        odom_tf_update_ = true;
        update_local_tf();
        return;
    }else{
        ROS_INFO("latest_vel_.angular.z : %.2f, enable csm",latest_vel_.angular.z );
    };

#if 0
    if (latest_map_odom_tf_.getOrigin().x() == 0.0||isnan(latest_map_odom_tf_.getOrigin().x())|| isnan(latest_map_odom_tf_.getOrigin().x())||isnan(tf::getYaw(latest_map_odom_tf_.getRotation()))){
        ROS_INFO("get Nan tf ; skip!!");
        return;
    }
#endif
//    if (!init_pose_set_ ){
//        ROS_INFO("No latest map_odom tf from amcl!! skip");
//
//        return;
//    }

    // get latest odom, map->odom. odom->base
    if(!lookup_odom_base_tf(latest_odom_base_tf, latest_scan_.header.stamp))
    {
        ROS_ERROR("update amcl tf !! Couldn't determine robot's pose associated with laser scan!! skip");
        // update 5: lookup tf failure, csm will not run
        update_local_tf();
        return;
    }

    ROS_INFO("get map to odom [%.3f,%.3f,%.3f]",latest_map_odom_tf_.getOrigin().x(), latest_map_odom_tf_.getOrigin().y(),tf::getYaw(latest_map_odom_tf_.getRotation())  );

    ROS_INFO("get odom to baselink [%.3f,%.3f,%.3f]",latest_odom_base_tf.getOrigin().x(), latest_odom_base_tf.getOrigin().y(),tf::getYaw(latest_odom_base_tf.getRotation()));
//    if (!init_pose_set_ || !get_amcl_pose_){
//        ROS_ERROR("init pose not set!! skip scan");
//        return;
//    }



    // check odom_base change from encoder
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
        ROS_ERROR("latest match result %d, no movement cnt:%d",match_count_, nomove_cnt_);
//        update_local_tf();
        latest_final_pose_.header.stamp = latest_scan_.header.stamp;
        nomove_cnt_ ++;
        if (nomove_cnt_ > 2*reset_amcl_filter_cnt_ || match_count_ > reset_amcl_filter_cnt_){
            if (nomove_cnt_ > 10*reset_amcl_filter_cnt_)
                nomove_cnt_ = 2*reset_amcl_filter_cnt_;
            return;
        }


    }

    ROS_INFO("start match!!");
//    update_local_tf();

    // check base to laser tf
    // **** get base laser tf

    if (base_laser_tf_.getOrigin().x() == 0.0){
//            ROS_INFO("ffffff,%f",base_laser_tf_.getOrigin().x());

        ROS_INFO("lookup base_laser tf!!");
        if (!lookup_base_laser_tf(base_frame_id_,laser_frame_id_)){
            ROS_ERROR("No valid base to laser tf!! skip");
            return;
        }
    }





//    mutex.lock();






    // ** test csm fit only

//    vector<double> res = csm_wrapper->csm_fit(*map_scan_ptr,*scan_msg);
//    pre_scan = cur_scan;
//    if (!res.empty())
//        ROS_INFO("get csm fit %f, %f, %f",res[0],res[1],res[2]);
//    else
//        ROS_ERROR("csm failure!!!!");





//    latest_pose_.pose.pose = map_base_pose;









    //compare two map_odom_tf, choose the one result in more match count
    sm::LaserScan scan_ref, scan_ref_amcl, scan_ref_local;
    int corr_num_amcl = 0, coor_num_local = 0;
    gm::Pose laser_pose_amcl, laser_pose_local;


    // get_amcl_tf_
    // get pose from local
    tf::poseTFToMsg(latest_map_odom_tf_*tf::Transform(latest_odom_base_tf) * base_laser_tf_, laser_pose_local);
    ROS_INFO_STREAM("laser_pose_local"<<laser_pose_local);

    scan_ref_local = gen_ptr->get_laser(laser_pose_local);
    ROS_INFO_STREAM("discrete laser_pose_local"<<laser_pose_local);

    if (scan_ref_local.ranges.empty()){
        ROS_ERROR("laser_pose_local invalid laser pose!! cancle csm!!,[%.3f,%.3f,%.3f]",laser_pose_local.position.x,laser_pose_local.position.y,laser_pose_local.orientation.w);

        return;
    }
    // if laser simulator not update, generator old laser data ,return
    if(scan_ref_local.ranges.size()!=latest_scan_.ranges.size()){

        ROS_ERROR("laser simulator not update;skip!!");
        return;
    }
    coor_num_local = csm_wrapper->find_match_point(scan_ref_local,latest_scan_);

    if (get_amcl_tf_){
        // get pose from amcl
        tf::poseTFToMsg(temp_map_odom_tf*tf::Transform(latest_odom_base_tf) * base_laser_tf_, laser_pose_amcl);
        ROS_INFO_STREAM("laser_pose_amcl"<<laser_pose_amcl);

        scan_ref_amcl = gen_ptr->get_laser(laser_pose_amcl);
        ROS_INFO_STREAM("discrete laser_pose_amcl"<<laser_pose_amcl);

        if (scan_ref_amcl.ranges.empty()){
            ROS_ERROR("laser_pose_amcl invalid laser pose!! cancle csm!!,[%.3f,%.3f,%.3f]",laser_pose_amcl.position.x,laser_pose_amcl.position.y,laser_pose_amcl.orientation.w);
        }else{
            corr_num_amcl = csm_wrapper->find_match_point(scan_ref_amcl,latest_scan_);


        }
    }





    int scan_size = int(latest_scan_.ranges.size());

    ROS_INFO("amcl pose : scan_ref*scan_sens match point number %d/%d ",corr_num_amcl, scan_size );
    ROS_INFO("local pose : scan_ref*scan_sens match point number %d/%d ",coor_num_local, scan_size );

    if (corr_num_amcl < switch_source_*coor_num_local){
        scan_ref = scan_ref_local;
# if use_cache
        tf::Transform maplaser_tf;
        tf::poseMsgToTF(laser_pose_local,maplaser_tf);
        tf::poseTFToMsg(maplaser_tf*base_laser_tf_.inverse(),base_pose);
#endif
    }else{
        // if amcl pose match more point
        // update rule
        update_local_tf();
        scan_ref = scan_ref_amcl;
#if use_cache
        tf::Transform maplaser_tf;
        tf::poseMsgToTF(laser_pose_amcl,maplaser_tf);
        tf::poseTFToMsg(maplaser_tf*base_laser_tf_.inverse(),base_pose);
#endif

    }


    // check match point number
    // call csm function to find point

//    int corr_num = csm_wrapper->find_match_point(scan_ref,latest_scan_);
//    if (match_count_ >0 && corr_num > 0.2*scan_size){
//        ROS_INFO("update amcl tf !! \n scan_ref*scan_sens match point number %d/%d \n update amcl tf",corr_num, scan_size );
//        // update rule 6: if match point > thresh
//        update_local_tf();
//    }else{
//        ROS_INFO("Not update amcl tf !! \n scan_ref*scan_sens match point number %d/%d \n update amcl tf",corr_num, scan_size );
//
//
//    }


    // final start match csm!!!

    gm::Pose amcl_pose ;
    tf::poseTFToMsg(temp_map_odom_tf*tf::Transform(latest_odom_base_tf), amcl_pose);



    gm::Pose latest_pose, map_base_pose;
    tf::poseTFToMsg(latest_map_odom_tf_*tf::Transform(latest_odom_base_tf), latest_pose);
    gm::Pose base_pose = latest_pose;
    map_base_pose = latest_pose;
    ROS_INFO("get amcl pose !!!! --> [%.3f, %.3f, %.3f]",map_base_pose.position.x, map_base_pose.position.y, tf::getYaw(map_base_pose.orientation));





    double csm_prob, fft_prob;
    double csm_error ;
    bool csm_ok = false;
    int corr_valid_cnt = 0 ;


    if (enable_csm_){

        // fisr get baselink pose from latest_map_odom_tf


        ROS_INFO("start csm");
        csm_error = csm_wrapper->get_base_pose(scan_ref,latest_scan_,base_pose, base_laser_tf_, corr_valid_cnt);
        ROS_INFO("done csm");

#if 0
        // if enough match point, update latest_map_odom corr_valid_cnt/latest_scan_.ranges.size()
        if (corr_valid_cnt > csm_match_valid_limit_*latest_scan_.ranges.size() )
            latest_map_odom_tf_ = temp_map_odom_tf;
#endif
        if (csm_error < csm_match_error_limit_ ){
            ROS_INFO("csm error : %.4f ,valid count:%d,  match ok!!!", csm_error,corr_valid_cnt);
            csm_ok = true;
        } else{
            ROS_INFO("csm error : %.4f, valid count:%d, match failure!!!", csm_error,corr_valid_cnt);
            if (match_count_ >0)
                match_count_ = 0;
            if (match_count_ <1)
                match_count_-=1;


        }

        // check diff between refine pose with amcl_pose
        csm_prob = check_pose_prob(amcl_pose,base_pose );

        ROS_ERROR("match csm_prob : %f",csm_prob);

        if (!csm_ok || csm_prob < match_prob_thresh ){
            ROS_ERROR("csm restore to amcl pose");
            if (match_count_ >0)
                match_count_ = 0;
            if (match_count_ <1)
                match_count_-=1;


            latest_pose = map_base_pose;
        }else if (csm_ok && csm_prob > match_prob_thresh){
            latest_pose = base_pose;
            if(!enable_fft_){
                if (match_count_ >0)
                    match_count_ ++;
                if (match_count_ <1)
                    match_count_=1;
            }

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
            if (match_count_ >0)
                match_count_ = 0;
            if (match_count_ <1)
                match_count_-=1;


        }


        // check diff between refine pose with amcl_pose
        fft_prob = check_pose_prob(amcl_pose,base_pose );

        ROS_ERROR("fftw match prob : %f",fft_prob);

        if (!fft_ok || fft_prob < match_prob_thresh){
            ROS_ERROR("fft restore to amcl pose");
            if (match_count_ >0)
                match_count_ = 0;
            if (match_count_ <1)
                match_count_-=1;

        }else if (fft_ok && fft_prob > match_prob_thresh){
            latest_pose = base_pose;

            if (match_count_ >0)
                match_count_ ++;
            if (match_count_ <1)
                match_count_=1;

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
#if 1
    if (match_count_ > reset_amcl_filter_cnt_){



        if (update_odom_chage_){

            // update new odom->baselink
            tf::StampedTransform tx_odom;

            lookup_tf_change(odom_frame_id_,base_frame_id_,tx_odom,latest_scan_.header.stamp, latest_final_pose_.header.stamp);

            tf::Pose new_pose;
            tf::poseMsgToTF(latest_pose,new_pose);
            tf::poseTFToMsg(new_pose*tx_odom,latest_pose);
        }

        queue2.callAvailable(ros::WallDuration(0.02));
        int cnt = latest_partial_cloud_.poses.size();
        ROS_ERROR("match ok many times! update amcl partial cloud, latest cnt :%d", cnt);
//        latest_pose.position.z = std::max(0.5, 1.0 - double(corr_valid_cnt/latest_scan_.ranges.size()));
        double x_cov = reset_x_cov_;
        double y_cov = reset_y_cov_;
        double yaw_cov = reset_yaw_cov_;
        set_filter(latest_pose,x_cov, y_cov,yaw_cov);
//        init_pose_set_ = false;

        match_count_ = 1;


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
    latest_pose_ = *msg;

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
}
