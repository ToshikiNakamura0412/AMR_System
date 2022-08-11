#ifndef LOCAL_MAP_CREATOR_H
#define LOCAL_MAP_CREATOR_H

#include <ros/ros.h>
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
    void obs_callback(const geometry_msgs::PoseArray::ConstPtr& msg);

    // その他の関数
    bool   in_map(const double dist, const double angle);           // マップ内か判断
    bool   is_ignore_grid_index(const int index);                   // 柱の除去(index ver.)
    bool   is_ignore_angle(double angle);                           // 柱の除去(angle ver.)
    int    get_grid_index(const double dist, const double angle);   // グリッドのインデックスを返す
    int    xy_to_grid_index(const double x, const double y);        // グリッドのインデックスを返す
    void   grid_index_to_xy(const int index, double& x, double& y); // グリッドのインデックスから座標を返す


    // ----- 関数（引数なし）-----
    void init_map();   // マップの初期化
    void update_map(); // マップの更新


    // ----- 変数 -----
    int    hz_;       // ループ周波数 [Hz]
    double map_size_; // マップの一辺の長さ [m]
    double map_reso_; // マップの解像度 [m/cell]


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_obs_;

    // Publisher
    ros::Publisher pub_local_map_;

    // 各種オブジェクト
    geometry_msgs::PoseArray obs_poses_; // 障害物のポーズ配列
    nav_msgs::OccupancyGrid  local_map_; // ローカルマップ
};

#endif