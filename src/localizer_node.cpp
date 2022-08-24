#include "localizer/localizer.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "localizer"); // ノードの初期化
    AMCL amcl;
    amcl.process();

    return 0;
}
