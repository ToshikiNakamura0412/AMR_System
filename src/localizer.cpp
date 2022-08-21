#include "localizer/localizer.h"

// コンストラクタ
Localizer::Localizer():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("ignore_angle_range_list", ignore_angle_range_list_);

    // frame idの設定
    estimated_pose_.header.frame_id = "map";

    // Subscriber
    sub_map_   = nh_.subscribe("/map", 1, &Localizer::map_callback, this);
    sub_odom_  = nh_.subscribe("/roomba/odometry", 1, &Localizer::odom_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 1, &Localizer::laser_callback, this);

    // Publisher
    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);
}

// mapのコールバック関数
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_      = *msg;
    flag_map_ = true;
}

// odometryのコールバック関数
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    previous_odom_ = current_odom_;
    current_odom_  = *msg;
    flag_odom_     = true;
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_      = *msg;
    flag_laser_ = true;
}

// 唯一，main文で実行する関数
void Localizer::process()
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

// 位置推定
void Localizer::Localize()
{
    initialize();
    motion_update();
    measurement_update();
    resampling();
    estimate_pose();
}

// 最終的にパブリッシュする位置の決定
void Localizer::estimate_pose()
{
    pub_estimated_pose_.publish(estimated_pose_);
}

// 柱の場合、trueを返す
bool Localizer::is_ignore_angle(double angle)
{
    angle = abs(angle);

    if(ignore_angle_range_list_[0] < angle && angle < ignore_angle_range_list_[1])
        return true;
    else if(ignore_angle_range_list_[2] < angle)
        return true;
    else
        return false;
}
