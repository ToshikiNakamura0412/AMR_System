#include "obstacle_scanner/obstacle_scanner.h"

// コンストラクタ
LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("is_visible", is_visible_);
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("map_size",   map_size_);
    private_nh_.getParam("map_reso",   map_reso_);

    // Subscriber
    sub_laser_ = nh_.subscribe("/scan", 10, &LocalMapCreator::laser_callback, this);

    // Publisher
    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map2/obstacle", 10);
    pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map2", 10);

    // --- 基本設定 ---
    // header
    obs_poses_.header.frame_id = "base_link";
    local_map_.header.frame_id = "base_link";
    // info
    local_map_.info.resolution = map_reso_;
    local_map_.info.width      = int(round(map_size_/map_reso_));
    local_map_.info.height     = int(round(map_size_/map_reso_));
    local_map_.info.origin.position.x = -map_size_/2.0;
    local_map_.info.origin.position.y = -map_size_/2.0;
    // data
    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
}

// laserのコールバック関数
void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

// 障害物検知
void LocalMapCreator::scan_obstacle()
{
    obs_poses_.poses.clear();

    for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        double dist  = laser_.ranges[i];
        double angle = i * laser_.angle_increment + laser_.angle_min;

        if(is_ignore_angle(angle)) continue;

        geometry_msgs::Pose obs_pose; // 障害物のポーズ
        obs_pose.position.x = dist * cos(angle);
        obs_pose.position.y = dist * sin(angle);

        obs_poses_.poses.push_back(obs_pose);
    }
    std::cout << "-->> Before Publish(obs)\n" << std::endl;
    pub_obs_poses_.publish(obs_poses_);
    std::cout << "<<< Publish: local_map2/obstacle >>>\n\n" << std::endl;
}

// マップの初期化(全グリッドを空きにする)
void LocalMapCreator::init_map()
{
    local_map_.data.clear();

    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
        local_map_.data.push_back(0); //「空き」にする
}

// マップの更新
void LocalMapCreator::update_map()
{
    if(!is_visible_) return; // mapの不可視化
    std::cout << "\n======== Update Map =========" << std::endl;

    init_map(); // マップの初期化

    for(const auto& obs_pose : obs_poses_.poses)
    {
        double obs_x     = obs_pose.position.x;
        double obs_y     = obs_pose.position.y;
        double obs_dist  = hypot(obs_y, obs_x);
        double obs_angle = atan2(obs_y, obs_x);

        for(double dist_from_start=obs_dist; in_map(dist_from_start, obs_angle); dist_from_start+=map_reso_)
        {
            int grid_index = get_grid_index(dist_from_start, obs_angle);
            local_map_.data[grid_index] = -1; //「未知」にする
        }

        int grid_index = xy_to_grid_index(obs_x, obs_y);
        local_map_.data[grid_index] = 100; //「占有」にする
    }

    std::cout << " [update map] OK\n" << std::endl;
    std::cout << "-->> Before Publish(map)\n" << std::endl;
    pub_local_map_.publish(local_map_);
    std::cout << "<<< Publish: local_map2 >>>" << std::endl;
}

// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(double dist, double angle)
{
    double x = dist * cos(angle);
    double y = dist * sin(angle);
    int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    if(index_x<local_map_.info.width && index_y<local_map_.info.height)
        return true;
    else
        return false;
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(double dist, double angle)
{
    double x = dist * cos(angle);
    double y = dist * sin(angle);

    return xy_to_grid_index(x, y);
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(double x, double y)
{
    int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    return index_x + (index_y * local_map_.info.width);
}

// 柱の場合、trueを返す
bool LocalMapCreator::is_ignore_angle(double angle)
{
    angle = abs(angle);

    if((angle > M_PI*1.5/16.0) && (angle < M_PI*5.0/16.0))
        return true;
    else if(angle > M_PI*10.0/16.0)
        return true;
    else
        return false;
}

// 唯一メイン関数で実行する関数
void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        scan_obstacle();
        update_map();
        ros::spinOnce(); // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}
