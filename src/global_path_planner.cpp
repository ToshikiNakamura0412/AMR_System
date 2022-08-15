#include "global_path_planner/global_path_planner.h"

// コンストラクタ
AStarPlanner::AStarPlanner():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("way_point", way_point_);

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
void AStarPlanner::planning()
{
    int phase_size = way_point_.size();
    get_motion_model()

    for(int phase=0; phase<phase_size; phase++)
    {
        Node start_node = way_point_[phase];
        Node goal_node  = way_point_[phase+1];
        Node parent_node = start_node; // 親ノード
        Node std::vector<Node> child_node_list; // 子ノードのリスト
        
        open_list_.push_back(start_node);
        while(1)
        {
            if(is_same_node(parent_node, goal_node)) break;
            get_child_node_list(parent_node, child_node_list);

        }
        

    }

}

bool AStarPlanner::is_same_node(const Node n1, const Node n2)
{
    if(n1.index_x == n2.index_x && n1.index_y == n2.index_y)
        return true;
    else
        return false;
}

void AStarPlanner::get_child_node_list(const Node parent_node, std::vector<Node>& child_node_list)
{
    child_node_list.clear();
    int motion_num = motion_list_.size();

    for(int i=0; i<motion_num; i++)
    {
        Node node = move(parent_node, motion_list_[i]);
        child_node_list.push_back(node);
    }
}

Node AStarPlanner::move(Node node, const Motion motion)
{
    node.index_x += motion.dx;
    node.index_y += motion.dy;
    node.cost    += motion.cost;

    return node;
}

// ヒューリスティック値の計算
double AStarPlanner::calc_heuristic(const Node n1, const Node n2)
{
    // ヒューリスティックの重み
    const double w = 1.0; 

    // 2点間のユークリッド距離
    const double dx = double(n1.index_x - n2.index_x);
    const double dy = double(n1.index_y - n2.index_y);
    const double dist = hypot(dx, dy); 

    return w*dist;
}

int AStarPlanner::calc_grid_index(const int index_x, const int index_y)
{
    return index_x + (index_y * map_.info.width);
}

void AStarPlanner::get_motion_list()
{
    motion_list_.push_back(get_motion_param( 1,  0, 1)); // 前
    motion_list_.push_back(get_motion_param( 0,  1, 1)); // 左
    motion_list_.push_back(get_motion_param(-1,  0, 1)); // 後ろ
    motion_list_.push_back(get_motion_param( 0, -1, 1)); // 右

    motion_list_.push_back(get_motion_param(-1,-1, sqrt(2))); // 右後ろ
    motion_list_.push_back(get_motion_param(-1, 1, sqrt(2))); // 左後ろ
    motion_list_.push_back(get_motion_param( 1,-1, sqrt(2))); // 右前
    motion_list_.push_back(get_motion_param( 1, 1, sqrt(2))); // 左前
}

std::vector<Motion> AStarPlanner::get_motion(const int dx, const int dy, const double cost)
{
    Motion motion;
    motion.dx   = dx;
    motion.dy   = dy;
    motion.cost = cost;

    return motion;
}