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
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);   // コールバック関数
    void expand_obstacle(); // 障害物の膨張
    void change_surrounding_grid_color(const int occupied_grid_index); // 周囲のグリッドの色の変更
    bool in_same_line(const int occupied_grid_index, const int index); // 占有グリッドと同じ行内か判断


    // ----- 変数 -----
    int    hz_;            // ループ周波数 [Hz]
    double target_margin_; // 車両マージン [m]

    // msg受け取りフラッグ
    bool flag_map_= false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_raw_map_;

    // Publisher
    ros::Publisher pub_updated_map_;

    // 各種オブジェクト
    nav_msgs::OccupancyGrid raw_map_;     // map_serverから受け取るマップ
    nav_msgs::OccupancyGrid updated_map_; // 障害物を車両マージン分膨張させたマップ
};

#endif
