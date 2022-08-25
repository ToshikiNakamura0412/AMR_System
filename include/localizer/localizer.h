#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <random>


// ===== 構造体 =====
struct Particle
{
    double x      = 0.0; // [m]
    double y      = 0.0; // [m]
    double yaw    = 0.0; // [rad]
    double weight = 0.0; // [-]
};


// ===== クラス =====
class OdomModel
{
public:
    OdomModel(); // デフォルトコンストラクタ
    OdomModel(double ff, double fr, double rf, double rr); // コンストラクタ
    /*  引数  */
    // ff: 直進1[m]で生じる道のりのばらつきの標準偏差
    // fr: 回転1[rad]で生じる道のりのばらつきの標準偏差
    // rf: 直進1[m]で生じる向きのばらつきの標準偏差
    // rr: 回転1[rad]で生じる向きのばらつきの標準偏差

    void   show_info();
    void   set_dev(const double length, const double angle); // 標準偏差の設定
    double get_fw_noise();  // 直進に関するノイズの取得
    double get_rot_noise(); // 回転に関するノイズの取得

    OdomModel& operator =(const OdomModel& t); // 代入演算子

private:
    double fw_var_per_fw_;   // ffの分散
    double fw_var_per_rot_;  // frの分散
    double rot_var_per_fw_;  // rfの分散
    double rot_var_per_rot_; // rrの分散

    double fw_dev_;  // 直進に関するノイズの取得
    double rot_dev_; // 回転に関するノイズの取得

    // 正規分布
    std::normal_distribution<> std_norm_dist_;
    std::random_device seed_gen_;
    std::default_random_engine engine_;
};

class AMCL
{
public:
    AMCL(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数（引数あり）------
    // コールバック関数
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // その他の関数
    void   move_particle(Particle& p, double length, double direction, double rotation);
    bool   is_ignore_angle(double angle); // 柱か判断
    double normalize_angle(double angle); // 適切な角度(-M_PI ~ M_PI)を返す


    // ----- 関数（引数なし）------
    void localize();
    void initialize();
    void motion_update();
    void measurement_update();
    void resampling();
    void estimate_pose();
    void publish_particles();
    void broadcast_odom_state();


    // ----- 変数 -----
    int    hz_;                                   // ループ周波数 [Hz]
    int    laser_step_;                           // 何本ずつレーザを見るか
    double init_x_;                               // 初期位置 [m]
    double init_y_;                               // 初期位置 [m]
    double init_yaw_;                             // 初期姿勢 [rad]
    double init_dev_;                             // 正規分布の標準偏差 [m]
    double particle_num_;                         // パーティクルの個数
    OdomModel odom_model_;
    Particle  estimated_particle_;
    std::vector<Particle> particles_;
    std::vector<double> ignore_angle_range_list_; // 柱に関する角度範囲の配列 [rad]

    // msg受け取りフラグ
    bool flag_map_   = false;
    bool flag_odom_  = false;
    bool flag_laser_ = false;

    // odomモデル関連
    double ff_;
    double fr_;
    double rf_;
    double rr_;


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
    geometry_msgs::PoseArray   particle_cloud_; // パーティクルクラウド
    nav_msgs::OccupancyGrid    map_;            // map_serverから受け取るマップ
    nav_msgs::Odometry         current_odom_;   // 現在のodometry
    nav_msgs::Odometry         previous_odom_;  // 1制御周期前のodometry
    sensor_msgs::LaserScan     laser_;          // レーザ値
};

#endif
