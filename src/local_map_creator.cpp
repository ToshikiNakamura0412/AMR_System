#include "local_map/local_map_creator.h"

// コンストラクタ
LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("map_size", map_size_);
    private_nh_.getParam("map_reso", map_reso_);

    // Subscriber
    sub_obs_ = nh_.subscribe("/local_map2/obstacle", 1, &LocalMapCreator::obs_callback, this);

    // Publisher
    pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map2", 1);

    // --- 基本設定 ---
    // header
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
void LocalMapCreator::obs_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    obs_poses_ = *msg;
}

// 唯一メイン関数で実行する関数
void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        update_map();      // マップの更新
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// マップの更新
void LocalMapCreator::update_map()
{
    init_map(); // マップの初期化

    for(const auto& obs_pose : obs_poses_.poses)
    {
        const double obs_x     = obs_pose.position.x;
        const double obs_y     = obs_pose.position.y;
        const double obs_dist  = hypot(obs_y, obs_x);
        const double obs_angle = atan2(obs_y, obs_x);

        for(double dist_from_start=obs_dist; in_map(dist_from_start, obs_angle); dist_from_start+=map_reso_)
        {
            const int grid_index = get_grid_index(dist_from_start, obs_angle);
            local_map_.data[grid_index] = -1; //「未知」にする
        }

        if(in_map(obs_dist, obs_angle))
        {
            const int grid_index = xy_to_grid_index(obs_x, obs_y);
            local_map_.data[grid_index] = 100; //「占有」にする
        }
    }

    pub_local_map_.publish(local_map_);
}

// マップの初期化
void LocalMapCreator::init_map()
{
    local_map_.data.clear();

    const int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
    {
        if(is_ignore_grid_index(i))
            local_map_.data.push_back(-1); //「未知」にする
        else
            local_map_.data.push_back(0);  //「空き」にする
    }
}

// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(const double dist, const double angle)
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    if(index_x<local_map_.info.width && index_y<local_map_.info.height)
        return true;
    else
        return false;
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);

    return xy_to_grid_index(x, y);
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y)
{
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    return index_x + (index_y * local_map_.info.width);
}

// 柱の場合、trueを返す(index ver.)
bool LocalMapCreator::is_ignore_grid_index(const int grid_index)
{
    double x, y;
    grid_index_to_xy(grid_index, x, y);
    const double angle = atan2(y, x);

    return is_ignore_angle(angle);
}

// グリッドのインデックスから座標を返す
void LocalMapCreator::grid_index_to_xy(const int index, double& x, double &y)
{
    const int index_x = index % local_map_.info.width;
    const int index_y = index / local_map_.info.width;

    x = index_x * local_map_.info.resolution + local_map_.info.origin.position.x;
    y = index_y * local_map_.info.resolution + local_map_.info.origin.position.y;
}

// 柱の場合、trueを返す(angle ver.)
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