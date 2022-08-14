/*
二つの膨張方法を実装
    mode1: 上下左右に膨張
    mode2: 円形状に膨張
*/

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

    // mode1
    void change_surrounding_grid_color1(const int occupied_grid_index); // 周囲のグリッドの色の変更(上下左右に膨張)
    bool in_same_line(const int occupied_grid_index, const int index);  // 占有グリッドと同じ行内か判断
    bool get_max_grid_index_in_same_line(const int index);              // 行のインデックスの最大値の算出
    bool get_mix_grid_index_in_same_line(const int index);              // 行のインデックスの最小値の算出

    // mode2
    void   change_surrounding_grid_color2(const int occupied_grid_index); // 周囲のグリッドの色の変更(円形状に膨張)
    void   search_rect_grid_index_list(const int center_grid_index);      // 探索対象のグリッドのインデックスを追加
    int    get_upper_left_grid_index(const int center_grid_index);        // rect_grid_index_listの左上のインデックスを検索
    int    get_lower_right_grid_index(const int center_grid_index);       // rect_grid_index_listの右下のインデックスを検索
    int    get_grid_index_x(const int index);                             // グリッドのインデックスからxのインデックスを計算
    int    get_grid_index_y(const int index);                             // グリッドのインデックスからyのインデックスを計算
    double get_dist_cell(const int index1, const int index2);             // グリッド同士の距離[cell]を計算


    // ----- 変数 -----
    int    hz_;            // ループ周波数 [Hz]
    int    mode_;          // 膨張モード [-]
    double target_margin_; // 車両マージン [m]

    // 占有グリッド周辺のインデックス格納用
    std::vector<int> rect_grid_index_list_;

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
