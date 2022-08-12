#include "global_path_planner/obstacle_expander.h"

// コンストラクタ
ObstacleExpander::ObstacleExpander():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("target_margin", target_margin_);

    // frame idの設定
    updated_map_.header.frame_id = "map";

    // Subscriber
    sub_original_map_ = nh_.subscribe("/map", 1, &ObstacleExpander::map_callback, this);

    // Publisher
    pub_updated_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map/updated_map", 1);
}

// mapのコールバック関数
void ObstacleExpander::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    original_map_ = *msg;
    flag_map_ = true;
}

// 唯一メイン関数で実行する関数
void ObstacleExpander::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_map_)
            expand_obstacle(); // 障害物の膨張
        ros::spinOnce();       // コールバック関数の実行
        loop_rate.sleep();     // 周期が終わるまで待つ
    }
}

// 障害物の膨張
void ObstacleExpander::expand_obstacle()
{
    updated_map_ = original_map_;


    pub_updated_map_.publish(updated_map_);
}
