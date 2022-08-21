#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


// ===== 構造体 =====
struct Particle
{
    double x      = 0.0; // [m]
    double y      = 0.0; // [m]
    double yaw    = 0.0; // [rad]
    double weight = 0.0; // [-]
};


// ===== クラス =====
class Localizer
{
public:
    Localizer(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数（引数あり）------
    // コールバック関数
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // その他の関数
    bool Localizer::is_ignore_angle(double angle) // 柱か判断


    // ----- 関数（引数なし）------
    void localize();
    void initialize();
    void motion_update();
    void measurement_update();
    void resampling();
    void estimate_pose();


    // ----- 変数 -----
    int hz_;                                      // ループ周波数 [Hz]
    int laser_step_;                              // 何本ずつレーザを見るか
    Particle particle;
    std::vector<double> ignore_angle_range_list_; // 柱に関する角度範囲の配列 [rad]

    // msg受け取りフラグ
    bool flag_map_   = false;
    bool flag_odom_  = false;
    bool flag_laser_ = false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_map_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_laser_;

    // Publisher
    ros::Publisher pub_estimated_pose_;
    ros::Publisher pub_particle_cloud_;

    // 各種オブジェクト
    geometry_msgs::PoseStamped estimated_pose_; // 推定位置
    geometry_msgs::PoseArray   particle_cloud_;
    nav_msgs::OccupancyGrid    map_;            // map_serverから受け取るマップ
    nav_msgs::Odometry         current_odom_;
    nav_msgs::Odometry         previous_odom_;
    sensor_msgs::LaserScan     laser_;          // レーザ値
};

#endif