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
    sub_raw_map_ = nh_.subscribe("/map", 1, &ObstacleExpander::map_callback, this);

    // Publisher
    pub_updated_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map/updated_map", 1);
}

// mapのコールバック関数
void ObstacleExpander::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    raw_map_  = *msg;
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

// 障害物を膨張
void ObstacleExpander::expand_obstacle()
{
    updated_map_ = raw_map_;

    const int size = raw_map_.data.size();
    for(int index=0; index<size; index++)
        if(raw_map_.data[index] == 100) // 「占有」のとき
            change_surrounding_grid_color(index); // 周囲のグリッドの色の変更

    pub_updated_map_.publish(updated_map_);
}

// 周囲のグリッドの色を変更
void ObstacleExpander::change_surrounding_grid_color(const int occupied_grid_index)
{
    const int target_margin_cell = int(round(target_margin_/updated_map_.info.resolution)); // 車両マージン [cell]

    // 上下に膨張
    for(int i=0; i<2; i++)
    {
        int index = occupied_grid_index;
        for(int j=0; j<target_margin_cell; j++)
        {
            switch(i)
            {
                case 0 : // 上方向に膨張
                    index -= updated_map_.info.width;
                    break;
                case 1 : // 下方向に膨張
                    index += updated_map_.info.width;
                    break;
            }

            // 配列dataの範囲内のとき
            if(0 <= index && index < updated_map_.data.size())
                updated_map_.data[index] = 100;
        }
    }

    // 左右に膨張
    for(int i=0; i<2; i++)
    {
        int index = occupied_grid_index;
        for(int j=0; j<target_margin_cell; j++)
        {
            switch(i)
            {
                case 0 : // 左方向に膨張
                    index--;
                    break;
                case 1 : // 右方向に膨張
                    index++;
                    break;
            }

            // 配列dataの範囲内かつ占有グリッドと同じ行内のとき
            if((0 <= index && index < updated_map_.data.size()) && in_same_line(occupied_grid_index, index))
                updated_map_.data[index] = 100;
        }
    }
}

// 占有グリッドと同じ行内の場合、trueを返す
bool ObstacleExpander::in_same_line(const int occupied_grid_index, const int index)
{
    // 占有グリッドを含む行のインデックスの最大値・最小値の算出
    int min_index = 0;
    int max_index = updated_map_.info.width-1;
    while(max_index < occupied_grid_index)
    {
        min_index += updated_map_.info.width;
        max_index += updated_map_.info.width;
    }

    // 同じ行内か判断
    if(min_index <= index && index <= max_index)
        return true;
    else
        return false;
}
