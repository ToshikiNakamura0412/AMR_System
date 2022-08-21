#include "localizer/localizer.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "localizer"); // ノードの初期化
    Localizer localizer;
    localizer.process();

    return 0;
}
