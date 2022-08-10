/*
Ray casting 2D grid map
*/

#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>


// ===== 構造体 =====
struct PrecastDB
{
    double precast_x = 0.0; // プレキャスト先のx座標 [m]
    double precast_y = 0.0; // プレキャスト先のy座標 [m]
    double dist      = 0.0; // プレキャスト先までのユークリッド距離 [m]
    double angle     = 0.0; // プレキャスト先に対する角度 [rad]
    int grid_index   = 0;   // プレキャスト先のインデックス
};

// ===== クラス =====
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
    double optimize_angle(double angle);      // 適切な角度(-M_PI ~ M_PI)を返す
    int xy_to_grid_index(double x, double y); // グリッドのインデックスを返す
    bool is_ignore_angle(double angle);       // 柱の除去


    // ----- 関数（引数なし）-----
    void init_map();   // マップの初期化
    void scan_obstacle(); // 障害物検知
    void update_map(); // マップの更新
    void precasting(); // プレキャスティング
    // std::vector<std::vector<PrecastDB>> precasting(); // プレキャスティング


    // ----- 変数 -----
    int    hz_;         // ループ周波数 [Hz]
    double map_size_;   // マップの一辺の長さ [m]
    double map_reso_;   // マップの解像度 [m/cell]
    double yaw_reso_;   // 角度解像度 [rad]
    double laser_step_; // 何本ずつレーザを見るか

    // msgの受け取り判断用
    bool flag_laser_poses_ = false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_laser_;

    // Publisher
    ros::Publisher pub_obs_poses_;
    ros::Publisher pub_local_map_;

    // 各種オブジェクト
    geometry_msgs::PoseArray obs_poses_; // 障害物のポーズ配列
    nav_msgs::OccupancyGrid local_map_;  // ローカルマップ
    sensor_msgs::LaserScan laser_;       // レーザ値
    std::vector<std::vector<PrecastDB>> precast_;
};

#endif
