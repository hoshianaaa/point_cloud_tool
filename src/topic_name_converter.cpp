#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class TopicNameConverter
{
public:
    TopicNameConverter();
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    std::string input_name_, output_name_;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
};

TopicNameConverter::TopicNameConverter()
{
    std::cout << "start TopicNameConverter class" << std::endl;

    ros::NodeHandle private_nh("~");
    private_nh.param("input_name", input_name_, std::string("velodyne_points"));
    private_nh.param("output_name", output_name_, std::string("points_raw"));

    cloud_sub_ = nh_.subscribe(input_name_, 1, &TopicNameConverter::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_name_, 1, false);
}

void TopicNameConverter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{
    cloud_pub_.publish(*msgs);
}

int main(int argc, char **argv)
{
    std::cout << "start topic_name_converter node" << std::endl;
    ros::init(argc, argv, "topic_name_converter");
    TopicNameConverter tnc;
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
