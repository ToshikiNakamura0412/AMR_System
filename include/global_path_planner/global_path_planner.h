#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>


// ===== 構造体 =====
struct Node
{
    int    index_x; // [cell]
    int    index_y; // [cell]
    double cost;
};

struct Motion
{
    int dx; // [cell]
    int dy; // [cell]
    double cost;
};


// ===== クラス =====
class AStarPlanner
{
public:
    AStarPlanner(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数 ------
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg); // コールバック関数
    void planning(); // グローバルパスの生成
    double calc_heuristic(const Node n1, const Node n2); // ヒューリスティック値の計算
    void get_motion_list();
    std::vector<Motion> get_motion_param(const int dx, const int dy, const double cost);
    int calc_grid_index(const int index_x, const int index_y);
    bool is_same_node(const Node n1, const Node n2);
    void get_child_node_list(const Node parent_node, std::vector<Node>& child_node_list);
    Node move(Node node, const Motion motion);

    // ----- 変数 -----
    int hz_; // ループ周波数 [Hz]
    std::vector<Motion> motion_list_;
    std::vector<Node> way_point_; // スタートとゴールを含む
    std::vector<Node> open_list_; // Openリスト
    std::vector<Node> close_list_; // Closeリスト

    // msg受け取りフラグ
    bool flag_map_ = false;
    bool is_goal_ = false;


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
