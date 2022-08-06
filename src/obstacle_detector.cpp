#include "obstacle_detector/obstacle_detector.h"

// コンストラクタ
LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);

    // Subscriber

    // Publisher

}

// laserのコールバック関数
void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = &msg;
    flag_obs_poses_ = true;
}



// 唯一メイン関数で実行する関数
void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定
    
    while(ros::ok())
    {

        ros::spinOnce(); // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}