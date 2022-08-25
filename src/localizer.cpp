#include "localizer/localizer.h"

// ----- AMCL -----
// コンストラクタ
AMCL::AMCL():private_nh_("~")
{
    // パラメータの取得(AMCL)
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("particle_num", particle_num_);
    private_nh_.getParam("init_x", init_x_);
    private_nh_.getParam("init_y", init_y_);
    private_nh_.getParam("init_yaw", init_yaw_);
    private_nh_.getParam("ignore_angle_range_list", ignore_angle_range_list_);
    // パラメータの取得(OdomModel)
    private_nh_.getParam("ff", ff_);
    private_nh_.getParam("fr", fr_);
    private_nh_.getParam("rf", rf_);
    private_nh_.getParam("rr", rr_);

    // --- 基本設定 ---
    // frame idの設定
    estimated_pose_.header.frame_id = "map";
    particle_cloud_.header.frame_id = "map";
    // メモリの確保
    particle_cloud_.poses.reserve(particle_num_);
    // odomのモデルの初期化
    odom_model_ = OdomModel(ff_, fr_, rf_, rr_);

    // Subscriber
    sub_map_   = nh_.subscribe("/map", 1, &AMCL::map_callback, this);
    sub_odom_  = nh_.subscribe("/roomba/odometry", 1, &AMCL::odom_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 1, &AMCL::laser_callback, this);

    // Publisher
    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose2", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud2", 1);
}

// mapのコールバック関数
void AMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_      = *msg;
    flag_map_ = true;
    initialize();
}

// odometryのコールバック関数
void AMCL::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    previous_odom_ = current_odom_;
    current_odom_  = *msg;
    flag_odom_     = true;
}

// laserのコールバック関数
void AMCL::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_      = *msg;
    flag_laser_ = true;
}

// 唯一，main文で実行する関数
void AMCL::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_map_ && flag_odom_ && flag_laser_)
        {
            // broadcast_odom_state(); // map座標系とodom座標系の関係を報告
            localize(); // 自己位置推定
        }
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// パーティクルの初期化
void AMCL::initialize()
{
    Particle particle;
    for(int i=0; i<particle_num_; i++)
    {
        particle.x   = init_x_;
        particle.y   = init_y_;
        particle.yaw = init_yaw_;
        particles_.push_back(particle);
    }
}

// map座標系とodom座標系の関係を報告
void AMCL::broadcast_odom_state()
{
    // TF Broadcasterの実体化
    static tf2_ros::TransformBroadcaster odom_state_broadcaster;

    // map座標系からみたbase_link座標系の位置と姿勢の取得
    const double map_to_base_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
    const double map_to_base_x   = estimated_pose_.pose.position.x;
    const double map_to_base_y   = estimated_pose_.pose.position.y;

    // odom座標系からみたbase_link座標系の位置と姿勢の取得
    const double odom_to_base_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
    const double odom_to_base_x   = current_odom_.pose.pose.position.x;
    const double odom_to_base_y   = current_odom_.pose.pose.position.y;

    // map座標系からみたodom座標系の位置と姿勢の取得
    // (回転行列を使った単純な座標変換)
    const double map_to_odom_yaw = normalize_angle(map_to_base_yaw - odom_to_base_yaw);
    const double map_to_odom_x   = map_to_base_x - odom_to_base_x * cos(map_to_odom_yaw) + odom_to_base_y * sin(map_to_odom_yaw);
    const double map_to_odom_y   = map_to_base_y - odom_to_base_x * sin(map_to_odom_yaw) - odom_to_base_y * cos(map_to_odom_yaw);

    // yawからquaternionを作成
    tf2::Quaternion map_to_odom_quat;
    map_to_odom_quat.setRPY(0, 0, map_to_odom_yaw);

    // odom座標系の元となodomの位置姿勢情報格納用変数の作成
    geometry_msgs::TransformStamped odom_state;

    // 現在の時間の格納
    odom_state.header.stamp = ros::Time::now();

    // 親フレーム・子フレームの指定
    odom_state.header.frame_id = map_.header.frame_id;
    odom_state.child_frame_id  = current_odom_.header.frame_id;

    // map座標系からみたodom座標系の原点位置と方向の格納
    odom_state.transform.translation.x = map_to_odom_x;
    odom_state.transform.translation.y = map_to_odom_y;
    odom_state.transform.rotation.x    = map_to_odom_quat.x();
    odom_state.transform.rotation.y    = map_to_odom_quat.y();
    odom_state.transform.rotation.z    = map_to_odom_quat.z();
    odom_state.transform.rotation.w    = map_to_odom_quat.w();

    // tf情報をbroadcast(座標系の設定)
    odom_state_broadcaster.sendTransform(odom_state);
}

// 適切な角度(-M_PI ~ M_PI)を返す
double AMCL::normalize_angle(double angle)
{
    while(M_PI  < angle) angle -= 2.0*M_PI;
    while(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}

// 自己位置推定
void AMCL::localize()
{
    motion_update();      // 動作更新
    // measurement_update(); // 観測更新
    // resampling();         // リサンプリング
    // estimate_pose();      // 推定位置の決定
    pub_estimated_pose_.publish(estimated_pose_); // 推定位置のパブリッシュ
    publish_particles();  // パーティクルクラウドのパブリッシュ
}

// 動作更新
void AMCL::motion_update()
{
    // quaternionからyawを算出
    const double current_yaw  = tf2::getYaw(current_odom_.pose.pose.orientation);
    const double previous_yaw = tf2::getYaw(previous_odom_.pose.pose.orientation);

    // 微小移動量を算出
    const double dx   = current_odom_.pose.pose.position.x - previous_odom_.pose.pose.position.x;
    const double dy   = current_odom_.pose.pose.position.y - previous_odom_.pose.pose.position.y;
    const double dyaw = normalize_angle(current_yaw - previous_yaw);

    // 1制御周期前のロボットから見た現在位置の距離と方位を算出
    const double length    = hypot(dx, dy);
    const double direction = normalize_angle(atan2(dy, dx) - previous_yaw);

    // 全パーティルクの移動
    for(auto& particle : particles_)
        move_particle(particle, length, direction, dyaw);
}

// パーティクルの移動
void AMCL::move_particle(Particle& p, double length, double direction, double rotation)
{
    // 標準偏差を設定
    odom_model_.set_dev(length, rotation);

    // ノイズを加える
    length    += odom_model_.get_fw_noise();
    direction += odom_model_.get_rot_noise();
    rotation  += odom_model_.get_rot_noise();

    // 移動
    p.x   += length * cos(normalize_angle(direction + p.yaw));
    p.y   += length * sin(normalize_angle(direction + p.yaw));
    p.yaw  = normalize_angle(p.yaw + rotation);
}

// 推定位置の決定
void AMCL::estimate_pose()
{
}

// パーティクルクラウドのパブリッシュ
void AMCL::publish_particles()
{
    particle_cloud_.poses.clear();
    /* --- パーティクルの数が変わる場合，ここでresize() --- */
    geometry_msgs::Pose pose;

    for(const auto& particle : particles_)
    {
        pose.position.x = particle.x;
        pose.position.y = particle.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, particle.yaw);
        tf2::convert(q, pose.orientation);

        particle_cloud_.poses.push_back(pose);
    }

    pub_particle_cloud_.publish(particle_cloud_);
}

// 柱の場合、trueを返す
bool AMCL::is_ignore_angle(double angle)
{
    angle = abs(angle);

    if(ignore_angle_range_list_[0] < angle && angle < ignore_angle_range_list_[1])
        return true;
    else if(ignore_angle_range_list_[2] < angle)
        return true;
    else
        return false;
}


// ----- OdomModel -----
// コンストラクタ
OdomModel::OdomModel(double ff, double fr, double rf, double rr)
    : std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0), engine_(seed_gen_())
{
    fw_var_per_fw_   = pow(ff ,2.0);
    fw_var_per_rot_  = pow(fr ,2.0);
    rot_var_per_fw_  = pow(rf ,2.0);
    rot_var_per_rot_ = pow(rr ,2.0);
}

// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    fw_dev_  = sqrt(fabs(length)*fw_var_per_fw_  + fabs(angle)*fw_var_per_rot_);
    rot_dev_ = sqrt(fabs(length)*rot_var_per_fw_ + fabs(angle)*rot_var_per_rot_);
}

// 直進に関するノイズの取得
double OdomModel::get_fw_noise()
{
    return std_norm_dist_(engine_) * fw_dev_;
}

// 回転に関するノイズの取得
double OdomModel::get_rot_noise()
{
    return std_norm_dist_(engine_) * rot_dev_;
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& t)
{
    fw_var_per_fw_   = t.fw_var_per_fw_;
    fw_var_per_rot_  = t.fw_var_per_rot_;
    rot_var_per_fw_  = t.rot_var_per_fw_;
    rot_var_per_rot_ = t.rot_var_per_rot_;

    fw_dev_  = t.fw_dev_;
    rot_dev_ = t.rot_dev_;

    return *this;
}
