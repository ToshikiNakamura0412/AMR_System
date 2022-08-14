#include "global_path_planner/global_path_planner.h"

// コンストラクタ
AStarPlanner::AStarPlanner():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);

    // frame idの設定
    global_path_.header.frame_id = "map";

    // Subscriber
    sub_map_ = nh_.subscribe("/map/updated_map", 1, &AStarPlanner::map_callback, this);

    // Publisher
    pub_global_path_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
}

// mapのコールバック関数
void AStarPlanner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
    flag_map_ = true;
}

// 唯一メイン関数で実行する関数
void AStarPlanner::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_map_)
            create_global_path(); // グローバルパスの生成
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// グローバルパスを生成
void AStarPlanner::create_global_path()
{

}
