#ifndef OBSTACLE_SCANNER_H
#define OBSTACLE_SCANNER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>


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
    bool is_ignore_angle(double angle);             // 柱の除去
    bool in_map(double dist, double angle);         // マップ内の場合、trueを返す
    int  get_grid_index(double dist, double angle); // グリッドのインデックスを返す
    int  xy_to_grid_index(double x, double y);      // グリッドのインデックスを返す


    // ----- 関数（引数なし）-----
    void scan_obstacle(); // 障害物検知
    void init_map();      // マップの初期化
    void update_map();    // マップの更新


    // ----- 変数 -----
    bool   is_visible_; // マップの可視化フラッグ
    int    hz_;         // ループ周波数 [Hz]
    int    laser_step_; // 何本ずつレーザを見るか
    double map_size_;   // マップの一辺の長さ [m]
    double map_reso_;   // マップの解像度 [m/cell]


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
    nav_msgs::OccupancyGrid  local_map_; // ローカルマップ
    sensor_msgs::LaserScan   laser_;     // レーザ値
};

#endif
