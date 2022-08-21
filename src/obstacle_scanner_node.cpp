#include "local_map/obstacle_scanner.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "obstacle_scanner"); // ノードの初期化
    ObstacleScanner obstacle_scanner;
    obstacle_scanner.process();

    return 0;
}
