#include "obstacle_detector/obstacle_detector.h"

// コンストラクタ
LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("map_size", map_size_);
    private_nh_.getParam("map_reso_", map_reso_);

    // Subscriber
    sub_laser_ = nh_.subscribe("/scan", 10, &LocalMapCreator::laser_callback, this);

    // Publisher
    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 10);
    pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 10);

    // 基本設定
    obs_poses_.header.frame_id = "base_link";
    local_map_.header.frame_id = "base_link";
    local_map_.info.resolution = map_reso_;
    local_map_.info.width = map_size_/map_reso_;
    local_map_.info.height = map_size_/map_reso_;
    local_map_.info.origin.position.x = -map_size_/2.0;
    local_map_.info.origin.position.y = -map_size_/2.0;
    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
    init_map();
}

// laserのコールバック関数
void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
    flag_laser_poses_ = true;
    update_map();
}

// マップの初期化(全グリッドを空きにする)
void LocalMapCreator::init_map()
{
    local_map_.data.clear();
    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
        local_map_.data.push_back(0);
}

// マップの更新
void LocalMapCreator::update_map()
{
    std::vector<std::vector<PrecastDB>> precast = precasting();

    for(int i=0; laser_.ranges.size(); i+=laser_step_)
    {
        double dist = laser_.ranges[i];
        if(dist > map_size_/2.0) continue;
        double angle = i * laser_.angle_increment;
        int angle_id = int(floor(angle/yaw_reso_));

        std::vector<PrecastDB> gridlist = precast[angle_id];

        double x = dist*cos(angle);
        double y = dist*sin(angle);
        int index_grid = xy_to_index(x, y);

        for(const auto& grid : gridlist)
            if(grid.dist > dist)
                local_map_.data[grid.index_grid] = -1;
        
        local_map_.data[index_grid] = 100;
    }
}

// プレキャスティング
std::vector<std::vector<PrecastDB>> LocalMapCreator::precasting()
{
    std::vector<std::vector<PrecastDB>> precast;

    for(int ix=0; ix<local_map_.info.width; ix++)
    {
        for(int iy=0; iy<local_map_.info.height; iy++)
        {
            double precast_x = ix * local_map_.info.resolution + local_map_.info.origin.position.x;
            double precast_y = iy * local_map_.info.resolution + local_map_.info.origin.position.y;
            double dist = hypot(precast_x, precast_y);
            double angle = optimize_angle(atan2(precast_y, precast_x));
            int angle_id = int(floor(angle / yaw_reso_));

            PrecastDB pc;

            pc.precast_x = precast_x;
            pc.precast_y = precast_y;
            pc.dist = dist;
            pc.angle = angle;
            pc.index_grid = xy_to_index(precast_x, precast_y);

            precast[angle_id].push_back(pc);
        }
    }
    return precast;
}

// 適切な角度(-M_PI ~ M_PI)を返す
double LocalMapCreator::optimize_angle(double angle)
{
    if(M_PI  < angle) angle -= 2.0*M_PI;
    if(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}

// グリッドのインデックスを返す
int LocalMapCreator::xy_to_index(double x, double y)
{
    int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    return index_x + (index_y * local_map_.info.width);
}

// 唯一メイン関数で実行する関数
void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定
    
    while(ros::ok())
    {
        if(flag_laser_poses_)
        {
            pub_local_map_.publish(local_map_);
            pub_obs_poses_.publish(obs_poses_);
        }
        ros::spinOnce(); // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}