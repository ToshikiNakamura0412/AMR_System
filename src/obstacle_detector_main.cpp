#include "obstacle_detector/obstacle_detector.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "obstacle_detector2"); // ノードの初期化
    LocalMapCreator local_map_creator;
    local_map_creator.process();

    return 0;
}
