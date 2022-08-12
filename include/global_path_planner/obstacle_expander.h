#ifndef OBSTACLE_EXPANDER_H
#define OBSTACLE_EXPANDER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>


// ===== クラス =====
class ObstacleExpander
{
public:
    ObstacleExpander(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数 ------
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg); // コールバック関数
    void expand_obstacle(); // 障害物の膨張


    // ----- 変数 -----
    int hz_;         // ループ周波数 [Hz]
    double target_margin_; // 車両マージン [m]

    // msg受け取りフラッグ
    bool flag_map_= false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_original_map_;

    // Publisher
    ros::Publisher pub_updated_map_;

    // 各種オブジェクト
    nav_msgs::OccupancyGrid original_map_; // map_serverから受け取るマップ
    nav_msgs::OccupancyGrid updated_map_; // 障害物を車両マージン分膨張させたマップ
};

#endif
