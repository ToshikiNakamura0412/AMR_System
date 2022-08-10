#include "obstacle_scanner/obstacle_scanner.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "obstacle_scanner2"); // ノードの初期化
    LocalMapCreator local_map_creator;
    local_map_creator.process();

    return 0;
}