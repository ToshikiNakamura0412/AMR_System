/*
Ray casting 2D grid map
*/

#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

class LocalMapCreator
{
public:
    LocalMapCreator(); // デフォルトコンストラクタ
    void process();
private:
    // ----- 関数（引数あり) ------
    // コールバック関数
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // その他の関数

    // ----- 関数（引数なし）-----


    // ----- 変数 -----
    int hz_; // ループ周波数 [Hz]

    // msgの受け取り判断用
    bool flag_laser_poses_ = false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber

    // Publisher
    ros::Publisher pub_obs_poses_;

    // 各種オブジェクト
    geometry_msgs::PoseArray obs_poses_; // 障害物のポーズ配列
    sensor_msgs::LaserScan laser_; // レーザ値
}

#endif
