#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

// ===== クラス =====
class HzChecker
{
public:
        HzChecker();
        void process();

private:
        // ----- 関数 -----
        void something_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);

        // ----- 変数 -----
        int hz_;
        int count_;
        ros::Time begin_;

        // ----- その他オブジェクト -----
        // NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        // Subscriber
        ros::Subscriber sub_something_;
};

// コンストラクタ
HzChecker::HzChecker():private_nh_("~")
{
    private_nh_.param("hz", hz_, 10);
    sub_something_ = nh_.subscribe("topic_name", 1, &HzChecker::something_callback, this);
}


// ===== 関数 =====
void HzChecker::something_callback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    ROS_INFO_STREAM("got!!");
}

void HzChecker::process()
{

    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        ros::spinOnce();
        // loop_rate.sleep();
    }
}


// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hz_checker");
    HzChecker hz_checker;
    hz_checker.process();
    return 0;
}
