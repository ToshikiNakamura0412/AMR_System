#include "local_map/obstacle_scanner.h"

// コンストラクタ
ObstacleScanner::ObstacleScanner():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);

    // frame idの設定
    obs_poses_.header.frame_id = "base_link";

    // Subscriber
    sub_laser_ = nh_.subscribe("/scan", 1, &ObstacleScanner::laser_callback, this);

    // Publisher
    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map2/obstacle", 1);
}

// laserのコールバック関数
void ObstacleScanner::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

// 障害物検知
void ObstacleScanner::scan_obstacle()
{
    // 障害物情報のクリア
    obs_poses_.poses.clear();

    // 障害物検知
    for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        // レーザ値の距離と角度の算出
        double dist  = laser_.ranges[i];
        double angle = i * laser_.angle_increment + laser_.angle_min;

        // 柱と被るレーザ値のスキップ
        if(is_ignore_angle(angle)) continue;

        // 障害物ポーズの格納
        geometry_msgs::Pose obs_pose;
        obs_pose.position.x = dist * cos(angle);
        obs_pose.position.y = dist * sin(angle);

        // 障害物ポーズの追加
        obs_poses_.poses.push_back(obs_pose);
    }

    pub_obs_poses_.publish(obs_poses_);
}

// 柱の場合、trueを返す
bool ObstacleScanner::is_ignore_angle(double angle)
{
    angle = abs(angle);

    if((angle > M_PI*1.5/16.0) && (angle < M_PI*5.0/16.0))
        return true;
    else if(angle > M_PI*10.0/16.0)
        return true;
    else
        return false;
}

// 唯一メイン関数で実行する関数
void ObstacleScanner::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        scan_obstacle();   // 障害物検知
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}
