#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

// ===== クラス =====
class ImageHzChecker
{
public:
        ImageHzChecker();
        void process();

private:
        // ----- 関数 -----
        void something_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);

        // ----- 変数 -----
        int  hz_;
        int  count_;
        int  count_max_;
        bool is_visible_;
        ros::Time begin_;


        // ----- その他オブジェクト -----
        // NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        // Subscriber
        ros::Subscriber sub_something_;
};

// コンストラクタ
ImageHzChecker::ImageHzChecker():private_nh_("~")
{
    private_nh_.param("hz", hz_, 100);
    private_nh_.param("count", count_, 0);
    private_nh_.param("count_max", count_max_, 50);
    private_nh_.param("is_visible", is_visible_, false);
    sub_something_ = nh_.subscribe("/topic_name", 1, &ImageHzChecker::something_callback, this);
}


// ===== 関数 =====
void ImageHzChecker::something_callback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    count_++;
    if(count_ == count_max_)
    {
        if(is_visible_)
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "] " << std::fixed << std::setprecision(3) << 50.0/(ros::Time::now().toSec()-begin_.toSec()) << " Hz");
        else
            ROS_INFO_STREAM(std::fixed << std::setprecision(3) << count_max_/(ros::Time::now().toSec()-begin_.toSec()) << " Hz");

        begin_ = ros::Time::now();
        count_ = 0;
    }
}

void ImageHzChecker::process()
{
    ros::Rate loop_rate(hz_);
    begin_ = ros::Time::now();
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_hz_checker");
    ImageHzChecker hz_checker;
    hz_checker.process();
    return 0;
}
