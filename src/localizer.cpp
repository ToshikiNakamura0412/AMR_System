#include "localizer/localizer.h"

// コンストラクタ
AMCL::AMCL():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("init_x", init_x_);
    private_nh_.getParam("init_y", init_y_);
    private_nh_.getParam("init_yaw", init_yaw_);
    private_nh_.getParam("particle_num_", particle_num_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("ignore_angle_range_list", ignore_angle_range_list_);

    // --- 基本設定 ---
    // frame idの設定
    estimated_pose_.header.frame_id = "map";
    particle_cloud_.header.frame_id = "map";
    // メモリの確保
    particle_cloud_.poses.reserve(particle_num_);

    // Subscriber
    sub_map_   = nh_.subscribe("/map", 1, &AMCL::map_callback, this);
    sub_odom_  = nh_.subscribe("/roomba/odometry", 1, &AMCL::odom_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 1, &AMCL::laser_callback, this);

    // Publisher
    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose2", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud2", 1);
}

// mapのコールバック関数
void AMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_      = *msg;
    flag_map_ = true;
    initialize();
}

// odometryのコールバック関数
void AMCL::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    previous_odom_ = current_odom_;
    current_odom_  = *msg;
    flag_odom_     = true;
}

// laserのコールバック関数
void AMCL::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_      = *msg;
    flag_laser_ = true;
}

// 唯一，main文で実行する関数
void AMCL::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_map_ && flag_odom_ && flag_laser_)
            localize();    // 位置推定
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// パーティクルの初期化
void AMCL::initialize()
{
    Particle particle;
    for(int i=0; i<particle_num_; i++)
    {
        particle.x   = init_x_;
        particle.y   = init_y_;
        particle.yaw = init_yaw_;
        particles_.push_back(particle);
    }
}

// 位置推定
void AMCL::Localize()
{
    // motion_update();
    // measurement_update();
    // resampling();
    estimate_pose();
    pub_estimated_pose_.publish(estimated_pose_);
    publish_particles();
}

// 最終的にパブリッシュする位置の決定
void AMCL::estimate_pose()
{
    estimate_pose_.pose.position.x = 0.0;
    estimate_pose_.pose.position.y = 0.0;
    estimate_pose_.pose.orientation.z = 0.0;
}

// パーティクルクラウドのパブリッシュ
void AMCL::publish_particles()
{
    particle_cloud_.poses.clear();
    geometry_msgs::Pose pose;

    for(const auto& particle : particles_)
    {
        pose.position.x    = particle.x;
        pose.position.y    = particle.y;
        pose.orientation.z = particle.yaw;
        particle_cloud_.poses.push_back(pose);
    }

    pub_particle_cloud_.publish(particle_cloud_);
}

// 柱の場合、trueを返す
bool AMCL::is_ignore_angle(double angle)
{
    angle = abs(angle);

    if(ignore_angle_range_list_[0] < angle && angle < ignore_angle_range_list_[1])
        return true;
    else if(ignore_angle_range_list_[2] < angle)
        return true;
    else
        return false;
}

// 適切な角度(-M_PI ~ M_PI)を返す
double AMCL::optimize_angle(double angle)
{
    if(M_PI  < angle) angle -= 2.0*M_PI;
    if(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}