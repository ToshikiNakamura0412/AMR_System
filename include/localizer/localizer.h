/*
emcl: mcl with expansion resetting
*/

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
    OdomModel(){} // デフォルトコンストラクタ
    OdomModel(const double ff, const double fr, const double rf, const double rr); // コンストラクタ
    /*  引数  */
    // ff: 直進1[m]で生じる道のりのばらつきの標準偏差
    // fr: 回転1[rad]で生じる道のりのばらつきの標準偏差
    // rf: 直進1[m]で生じる向きのばらつきの標準偏差
    // rr: 回転1[rad]で生じる向きのばらつきの標準偏差

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

class EMCL
{
public:
    EMCL(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数（引数あり）------
    // コールバック関数
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // 正規分布
    double norm_rv(const double mean, const double stddev);                  // ランダム変数生成関数
    double norm_pdf(const double x, const double mean, const double stddev); // 確率密度関数

    // その他の関数
    void   move_particle(Particle& p, double length, double direction, double rotation); // パーティクルの移動
    void   sort_data(std::vector<double>& data);             // 配列のデータを昇順に並び替える
    bool   is_ignore_angle(double angle);                    // 柱か判断
    bool   in_map(const int grid_index);                     // マップ内か判断
    int    xy_to_grid_index(const double x, const double y); // 座標からグリッドのインデックスを返す
    double normalize_angle(double angle);                    // 適切な角度(-M_PI ~ M_PI)を返す
    double likelihood(const Particle p);                     // 尤度関数
    double get_median(std::vector<double>& data);            // 配列の中央値を返す
    double calc_dist_to_wall(double x, double y, const double laser_angle, const double laser_range); // 壁までの距離を算出


    // ----- 関数（引数なし）------
    void   initialize();               // パーティクルの初期化
    void   reset_weight();             // 重みの初期化
    void   broadcast_odom_state();     // map座標系とodom座標系の関係を報告
    void   localize();                 // 自己位置推定
    void   motion_update();            // 動作更新
    void   observation_update();       // 観測更新
    void   estimate_pose();            // 推定位置の決定
    void   mean_pose();                // 推定位置の決定（平均）
    void   weighted_mean_pose();       // 推定位置の決定（加重平均）
    void   max_weight_pose();          // 推定位置の決定（最大の重みを有するポーズ）
    void   median_pose();              // 推定位置の決定（中央値）
    void   normalize_belief();         // 重みの正規化
    void   expansion_resetting();      // 膨張リセット
    void   resampling();               // リサンプリング
    void   publish_estimated_pose();   // 推定位置のパブリッシュ
    void   publish_particles();        // パーティクルクラウドのパブリッシュ
    double calc_marginal_likelihood(); // 周辺尤度の算出


    // ----- 変数 -----
    int       hz_;                 // ループ周波数 [Hz]
    int       particle_num_;       // パーティクルの個数 [-]
    int       reset_counter = 0;   // 連続リセットの回数 [-]
    int       reset_count_limit_;  // 連続リセットの回数の上限 [-]
    int       laser_step_;         // 何本ずつレーザを見るか [-]
    double    move_dist_th_;       // ロボットの移動開始判断用（スタートからの距離の閾値）[m]
    double    init_x_;             // 初期位置 [m]
    double    init_y_;             // 初期位置 [m]
    double    init_yaw_;           // 初期姿勢 [rad]
    double    init_x_dev_;         // 初期位置xの標準偏差 [m]
    double    init_y_dev_;         // 初期位置yの標準偏差 [m]
    double    init_yaw_dev_;       // 初期姿勢の標準偏差 [rad]
    double    alpha_th_;           // リセットに関する平均尤度の閾値 [-]
    double    expansion_x_dev_;    // 膨張リセットの位置xの標準偏差 [m]
    double    expansion_y_dev_;    // 膨張リセットの位置yの標準偏差 [m]
    double    expansion_yaw_dev_;  // 膨張リセットの姿勢の標準偏差 [rad]
    double    sensor_noise_ratio_; // 距離に対するセンサノイズ比 [-]
    Particle  particle_;           // 推定位置格納用
    OdomModel odom_model_;         // odometryのモデル

    // リスト
    std::vector<Particle> particles_;             // パーティクルクラウド（計算用）
    std::vector<double> ignore_angle_range_list_; // 柱に関する角度範囲の配列 [rad]

    // msg受け取りフラグ
    bool flag_map_   = false;
    bool flag_odom_  = false;
    bool flag_laser_ = false;

    // 各種設定フラグ
    bool flag_init_noise_; // 初期位置にノイズを加えるか
    bool flag_broadcast_;  // tf broadcastをするか
    bool is_visible_;     // パーティクルクラウドをパブリッシングするか

    // OdomModel関連
    double ff_;
    double fr_;
    double rf_;
    double rr_;

    // 正規分布用乱数
    std::random_device seed_gen_;
    std::default_random_engine engine_;


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
    nav_msgs::OccupancyGrid    map_;            // map_serverから受け取るマップ
    nav_msgs::Odometry         last_odom_;      // 最新のodometry
    nav_msgs::Odometry         prev_odom_;      // 1制御周期前のodometry
    sensor_msgs::LaserScan     laser_;          // レーザ値
    geometry_msgs::PoseStamped estimated_pose_; // 推定位置
    geometry_msgs::PoseArray   particle_cloud_; // パーティクルクラウド（パブリッシュ用）
};

#endif
