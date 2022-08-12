#ifndef OBSTACLE_SCANNER_H
#define OBSTACLE_SCANNER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>


// ===== クラス =====
class ObstacleScanner
{
public:
    ObstacleScanner(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数 ------
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg); // コールバック関数(laser)
    void scan_obstacle(); // 障害物検知
    bool is_ignore_angle(double angle); // 柱の除去


    // ----- 変数 -----
    int hz_;         // ループ周波数 [Hz]
    int laser_step_; // 何本ずつレーザを見るか
    std::vector<double> ignore_angle_range_list_; // 柱に関する角度範囲の配列 [rad]

    // msg受け取りフラグ
    bool flag_laser_= false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_laser_;

    // Publisher
    ros::Publisher pub_obs_poses_;

    // 各種オブジェクト
    geometry_msgs::PoseArray obs_poses_; // 障害物のポーズ配列
    sensor_msgs::LaserScan   laser_;     // レーザ値
};

#endif
