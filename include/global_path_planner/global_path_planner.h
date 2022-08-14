#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>


// ===== クラス =====
class AStarPlanner
{
public:
    AStarPlanner(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数 ------
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg); // コールバック関数
    void create_global_path(); // グローバルパスの生成


    // ----- 変数 -----
    int hz_; // ループ周波数 [Hz]

    // msg受け取りフラグ
    bool flag_map_ = false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_map_;

    // Publisher
    ros::Publisher pub_global_path_;

    // 各種オブジェクト
    nav_msgs::OccupancyGrid map_;         // obstacle_expanderノードから受け取るマップ
    nav_msgs::Path          global_path_; // グローバルパス
};

#endif
