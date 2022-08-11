#include "local_goal_creator/local_goal_creator.h"

// コンストラクタ
LocalGoalCreator::LocalGoalCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("local_goal_dist", local_goal_dist_);

    // frame idの設定
    local_goal_.header.frame_id = "map"

    // Subscriber
    sub_estimated_pose_ = nh_.subscribe("/estimated_pose", 1, &LocalGoalCreator::estimated_pose_callback, this);
    sub_global_path_    = nh_.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);

    // Publisher
    pub_local_goal_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal2", 1);
}

// estimated_poseのコールバック関数
void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    estimated_pose_ = *msg;
    flag_estimated_pose_ = true;
}

// global_pathのコールバック関数
void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    global_path_ = *msg;
    flag_global_path_ = true;
}

// 唯一メイン関数で実行する関数
void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_global_path_ && flag_estimated_pose_)
            update_goal(); // ゴールの更新
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// ゴールの更新
void LocalGoalCreator::update_goal()
{

    pub_local_goal_.publish(local_goal_);
}
