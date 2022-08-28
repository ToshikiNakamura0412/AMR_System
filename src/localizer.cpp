#include "localizer/localizer.h"

// ----- AMCL -----
// コンストラクタ
AMCL::AMCL():private_nh_("~"), engine_(seed_gen_())
{
    // パラメータの取得(AMCL)
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("particle_num", particle_num_);
    private_nh_.getParam("init_x", init_x_);
    private_nh_.getParam("init_y", init_y_);
    private_nh_.getParam("init_yaw", init_yaw_);
    private_nh_.getParam("init_dev", init_dev_);
    private_nh_.getParam("sensor_noise_ratio", sensor_noise_ratio_);
    private_nh_.getParam("reset_threshold", reset_threshold_);
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
    // odometryのモデルの初期化
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
    prev_odom_ = last_odom_;
    last_odom_ = *msg;
    flag_odom_ = true;
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
        if(flag_map_ and flag_odom_ and flag_laser_)
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

    // 初期位置近傍にパーティクルを配置
    for(int i=0; i<particle_num_; i++)
    {
        particle.x   = init_x_;
        particle.y   = init_y_;
        particle.yaw = init_yaw_;
        // particle.x   = norm_rv(init_x_,   init_dev_);
        // particle.y   = norm_rv(init_y_,   init_dev_);
        // particle.yaw = norm_rv(init_yaw_, init_dev_);
        particles_.push_back(particle);
    }

    reset_weight(); // 重みの初期化
}

// ランダム変数生成関数（正規分布）
double AMCL::norm_rv(const double mean, const double stddev)
{
    std::normal_distribution<> norm_dist(mean, stddev);
    return norm_dist(engine_);
}

// 重みの初期化
void AMCL::reset_weight()
{
    for(auto& p : particles_)
        p.weight = 1.0/particles_.size();
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
    const double odom_to_base_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
    const double odom_to_base_x   = last_odom_.pose.pose.position.x;
    const double odom_to_base_y   = last_odom_.pose.pose.position.y;

    // map座標系からみたodom座標系の位置と姿勢の取得
    // (回転行列を使った単純な座標変換)
    const double map_to_odom_yaw = normalize_angle(map_to_base_yaw - odom_to_base_yaw);
    const double map_to_odom_x   = map_to_base_x - odom_to_base_x * cos(map_to_odom_yaw)
        + odom_to_base_y * sin(map_to_odom_yaw);
    const double map_to_odom_y   = map_to_base_y - odom_to_base_x * sin(map_to_odom_yaw)
        - odom_to_base_y * cos(map_to_odom_yaw);

    // yawからquaternionを作成
    tf2::Quaternion map_to_odom_quat;
    map_to_odom_quat.setRPY(0, 0, map_to_odom_yaw);

    // odom座標系の元となodomの位置姿勢情報格納用変数の作成
    geometry_msgs::TransformStamped odom_state;

    // 現在の時間の格納
    odom_state.header.stamp = ros::Time::now();

    // 親フレーム・子フレームの指定
    odom_state.header.frame_id = map_.header.frame_id;
    odom_state.child_frame_id  = last_odom_.header.frame_id;

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
    motion_update();          // 動作更新
    observation_update();     // 観測更新（リサンプリングを含む）
    mean_pose();              // 推定位置の決定（平均）
    publish_estimated_pose(); // 推定位置のパブリッシュ
    publish_particles();      // パーティクルクラウドのパブリッシュ
}

// 動作更新
void AMCL::motion_update()
{
    // quaternionからyawを算出
    const double last_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
    const double prev_yaw = tf2::getYaw(prev_odom_.pose.pose.orientation);

    // 微小移動量を算出
    const double dx   = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy   = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    const double dyaw = normalize_angle(last_yaw - prev_yaw);

    // 1制御周期前のロボットから見た現在位置の距離と方位を算出
    const double length    = hypot(dx, dy);
    const double direction = normalize_angle(atan2(dy, dx) - prev_yaw);

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

// 観測更新（リサンプリングを含む）
void AMCL::observation_update()
{
    for(auto& particle : particles_)
        particle.weight *= likelihood(particle); // 尤度計算

    if(normalize_belief() < reset_threshold_) // 重みの正規化
        reset_weight(); // 重みの初期化
    else
        resampling(); // 系統リサンプリング
}

// 尤度関数
double AMCL::likelihood(const Particle p)
{
    double L = 0.0; // 尤度

    // センサ情報からパーティクルの姿勢を評価
    for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        const double angle = i * laser_.angle_increment + laser_.angle_min; // レーザ値の角度

        if(not is_ignore_angle(angle)) // 柱と被るレーザ値のスキップ
        {
            const double range = calc_dist_to_wall(p.x, p.y, normalize_angle(angle + p.yaw), laser_.ranges[i]);
            L += norm_pdf(range, laser_.ranges[i], laser_.ranges[i] * sensor_noise_ratio_);
        }
    }

    return L;
}

// 柱の場合、trueを返す
bool AMCL::is_ignore_angle(double angle)
{
    angle = abs(angle);

    if(ignore_angle_range_list_[0] < angle and angle < ignore_angle_range_list_[1])
        return true;
    else if(ignore_angle_range_list_[2] < angle)
        return true;
    else
        return false;
}

// 壁までの距離を算出
double AMCL::calc_dist_to_wall(double x, double y, const double laser_angle, const double laser_range)
{
    const double search_step = map_.info.resolution;
    const double search_limit = laser_range;

    for(double dist=0.0; dist<search_limit; dist+=search_step)
    {
        x += search_step * cos(laser_angle);
        y += search_step * sin(laser_angle);

        const int grid_index = xy_to_grid_index(x, y);

        if(not in_map(grid_index))
            return search_limit * 2.0;
        else if(map_.data[grid_index] == -1)
            return search_limit * 2.0;
        else if(map_.data[grid_index] == 100)
            return dist;
    }

    return search_limit * 1.3;
}

// 座標からグリッドのインデックスを返す
int AMCL::xy_to_grid_index(const double x, const double y)
{
    const int index_x = int(round((x - map_.info.origin.position.x) / map_.info.resolution));
    const int index_y = int(round((y - map_.info.origin.position.y) / map_.info.resolution));

    return index_x + (index_y * map_.info.width);
}

// マップ内の場合、trueを返す
bool AMCL::in_map(const int grid_index)
{
    if(0 <= grid_index and grid_index < map_.data.size())
        return true;
    else
        return false;
}

// 確率密度関数（正規分布）
double AMCL::norm_pdf(const double x, const double mean, const double stddev)
{
    return 1.0/sqrt(2.0 * M_PI * pow(stddev, 2.0)) * exp(-pow((x - mean), 2.0)/(2.0*pow(stddev, 2.0)));
}

// 重みの正規化
double AMCL::normalize_belief()
{
    double sum = 0.0;

    // 尤度の合計
    for(const auto& p : particles_)
        sum += p.weight;

    // 尤度の合計が小さ過ぎる場合
    if(sum < reset_threshold_)
        return sum;

    // 正規化
    for(auto& p : particles_)
        p.weight /= sum;

    return sum;
}

// 系統リサンプリング
void AMCL::resampling()
{
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;
    accum.push_back(particles_[0].weight);
    for(int i=1; i<particles_.size(); i++)
        accum.push_back(accum.back() + particles_[i].weight);

    // サンプリングのスタート位置とステップを設定
    const std::vector<Particle> old(particles_);
    const double start = (double)rand()/(RAND_MAX * particles_.size()); // 0 ~ 1/N
    const double step = 1.0 / particles_.size();

    // サンプリングするパーティクルのインデックスを保持
    std::vector<int> chosen_indexes;
    int tick=0;
    for(int i=0; i<particles_.size(); i++)
    {
        while(accum[tick] <= start + i*step)
        {
            tick++;
            if(tick == particles_.size())
            {
                ROS_ERROR("Resampling Failed"); // 配列の不正アクセス防止
                exit(1);
            }
        }
        chosen_indexes.push_back(tick);
    }

    // リサンプリング
    for(int i=0; i<particles_.size(); i++)
        particles_[i] = old[chosen_indexes[i]];

    // 重みを初期化
    reset_weight();
}

// 推定位置の決定（平均）
void AMCL::mean_pose()
{
    // 合計値
    Particle pose_sum;
    for(const auto& particle : particles_)
    {
        pose_sum.x   += particle.x;
        pose_sum.y   += particle.y;
        pose_sum.yaw += particle.yaw;
    }

    // 平均値
    particle_.x   = pose_sum.x   / particles_.size();
    particle_.y   = pose_sum.y   / particles_.size();
    particle_.yaw = pose_sum.yaw / particles_.size();
}

// 推定位置のパブリッシュ
void AMCL::publish_estimated_pose()
{
    estimated_pose_.pose.position.x = particle_.x;
    estimated_pose_.pose.position.y = particle_.y;

    // yawからquaternionを作成
    tf2::Quaternion q;
    q.setRPY(0, 0, particle_.yaw);
    tf2::convert(q, estimated_pose_.pose.orientation);

    pub_estimated_pose_.publish(estimated_pose_);
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

        // yawからquaternionを作成
        tf2::Quaternion q;
        q.setRPY(0, 0, particle.yaw);
        tf2::convert(q, pose.orientation);

        particle_cloud_.poses.push_back(pose);
    }

    pub_particle_cloud_.publish(particle_cloud_);
}



// ----- OdomModel -----
// コンストラクタ
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0)
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
