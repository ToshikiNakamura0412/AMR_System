#include "obstacle_scanner/obstacle_scanner.h"

// コンストラクタ
LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("map_size",   map_size_);
    private_nh_.getParam("map_reso",   map_reso_);
    private_nh_.getParam("yaw_reso",   yaw_reso_);
    private_nh_.getParam("laser_step", laser_step_);

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
    std::cout << "lazer callback" << std::endl;

    laser_ = *msg;
    flag_laser_poses_ = true;

    std::cout << "  [callback] OK " << std::endl;
}

// 障害物検知
void LocalMapCreator::scan_obstacle()
{
    std::cout << "scan obstacle" << std::endl;

    obs_poses_.clear();

    for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        double dist  = laser_.ranges[i];
        double angle = i * laser_.angle_increment + laser_.angle_min;

        if(is_ignore_angle(angle)) continue;

        geometry_msgs::Pose obs_pose; // 障害物のポーズ
        obs_pose.position.x = dist*cos(angle);
        obs_pose.position.y = dist*sin(angle);

        obs_poses_.push_back(obs_pose);
    }

    std::cout << "  [scan obstacle] OK" << std::endl;
}

// マップの初期化(全グリッドを空きにする)
void LocalMapCreator::init_map()
{
    std::cout << "\tinitialize map" << std::endl;

    local_map_.data.clear();
    
    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
        local_map_.data.push_back(0); //「空き」にする

    std::cout << "\t  [initialize map] OK " << std::endl;
}

// マップの更新
void LocalMapCreator::update_map()
{
    std::cout << "\n======== Update Map =========" << std::endl;

    init_map(); // マップの初期化

    for(const auto& obs_pose : obs_poses_)
    {

    }


    // // std::vector<std::vector<PrecastDB>> precast = precasting(); // プレキャスティング
    // precasting(); // プレキャスティング

    // for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    // {
    //     // std::cout << "i: " << i << std::endl;
    //     try
    //     {
    //         double dist = laser_.ranges[i];
    //         double angle = i * laser_.angle_increment + laser_.angle_min;
    //         if(is_ignore_angle(angle)) continue;
    //         angle = optimize_angle(angle);
    //         int angle_id = int(floor(angle/yaw_reso_));

    //         // std::cout << "\n\tangle_id: " << angle_id << std::endl;
    //         std::vector<PrecastDB> &gridlist = precast_.at(angle_id);
    //         // std::cout << "\tafter precast -> gridlist" << std::endl;

    //         double x = dist*cos(angle);
    //         double y = dist*sin(angle);
    //         int grid_index = xy_to_grid_index(x, y);

    //         for(const auto& grid : gridlist)
    //         {
    //             if(grid.dist > dist)
    //             {
    //                 // std::cout << "\n\tgrid.grid_index: " << grid.grid_index << std::endl;
    //                 local_map_.data[grid.grid_index] = -1; //「未知」にする
    //                 // std::cout << "\t-1 -> data " << std::endl;
    //             }
    //         }
    //         local_map_.data[grid_index] = 100; //「占有」にする
    //         // std::cout << "\t100 -> data " << std::endl;
    //     }
    //     catch(std::bad_alloc)
    //     {
    //         ROS_WARN_STREAM("failed to create a array.");
    //         exit(0);
    //     }
    //     catch(std::out_of_range)
    //     {
    //         ROS_WARN_STREAM("failed to access a array.");
    //         exit(1);
    //     }
    //     catch(...)
    //     {
    //         ROS_WARN_STREAM("others");
    //         exit(-1);
    //     }
    // }
    std::cout << "------ Update Map -> OK -------\n" << std::endl;
}

// プレキャスティング
// std::vector<std::vector<PrecastDB>> LocalMapCreator::precasting()
void LocalMapCreator::precasting()
{
    std::cout << "\tprecasting" << std::endl;

    try
    {
        // 空の２次元動的配列precastの作成(配列の不正アクセス防止)
        // std::vector<std::vector<PrecastDB>> precast;
        precast_.clear();
        std::vector<PrecastDB> empty_pc;
        int precast_size = int(round((2*M_PI)/yaw_reso_))+10;
        precast_.reserve(precast_size);
        for(int i=0; i<precast_size; i++) precast_.push_back(empty_pc);

        // プレキャスティングの実行
        for(int ix=0; ix<local_map_.info.width; ix++)
        {
            for(int iy=0; iy<local_map_.info.height; iy++)
            {
                try
                {
                    // std::cout << "\t\tix: " << ix << ", iy: " << iy << std::endl;
                    double precast_x = ix * local_map_.info.resolution + local_map_.info.origin.position.x;
                    double precast_y = iy * local_map_.info.resolution + local_map_.info.origin.position.y;
                    double dist = hypot(precast_x, precast_y);
                    double angle = optimize_angle(atan2(precast_y, precast_x));
                    int angle_id = int(floor(angle / yaw_reso_));

                    // std::cout << "\t\tp_x: " << precast_x << ", p_y: " << precast_y << std::endl;
                    PrecastDB pc;

                    pc.precast_x  = precast_x;
                    pc.precast_y  = precast_y;
                    pc.dist       = dist;
                    pc.angle      = angle;
                    pc.grid_index = xy_to_grid_index(precast_x, precast_y);

                    // std::cout << "\t\tangle_id: " << angle_id << std::endl;
                    precast_[angle_id].push_back(pc);
                    // std::cout << "\t\t[push_back] OK " << std::endl;
                }
                catch(std::bad_alloc)
                {
                    ROS_WARN_STREAM("failed to create a array.(In for-loop)");
                    exit(0);
                }
                catch(std::out_of_range)
                {
                    ROS_WARN_STREAM("failed to access a array.(In for-loop)");
                    exit(1);
                }
                catch(...)
                {
                    ROS_WARN_STREAM("others(In for-loop)");
                    exit(-1);
                }
            }
        }

        std::cout << "\t  [precasting] OK " << std::endl;
        // return precast;
    }
    catch(std::bad_alloc)
    {
        ROS_WARN_STREAM("failed to create a array.(Out of for-loop)");
        exit(0);
    }
    catch(std::out_of_range)
    {
        ROS_WARN_STREAM("failed to access a array.(Out of for-loop)");
        exit(1);
    }
    catch(...)
    {
        ROS_WARN_STREAM("others(Out of for-loop)");
        exit(-1);
    }
}

// 適切な角度(0 ~ 2*M_PI)を返す
double LocalMapCreator::optimize_angle(double angle)
{
    if(angle < 0.0) angle += 2.0*M_PI;
    return angle;
}

// グリッドのインデックスを返す
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

    if((angle > M_PI*2.0/16.0) && (angle < M_PI*5.0/16.0))
        return true;
    else if(angle > M_PI*11.0/16.0)
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
        if(flag_laser_poses_)
        {
            update_map();
            pub_local_map_.publish(local_map_);
            std::cout << "<<< Publish: local_map2 >>>\n\n" << std::endl;
            // pub_obs_poses_.publish(obs_poses_);
            flag_laser_poses_ = false;
        }
        ros::spinOnce(); // コールバック関数の実行
        std::cout << "spinOnce()" << std::endl;
        loop_rate.sleep(); // 周期が終わるまで待つ
        std::cout << "loop_rate.sleep()" << std::endl;
    }
}
