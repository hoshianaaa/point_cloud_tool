#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>


int flag_ = 1;

class OneDataSub
{
public:
    OneDataSub();
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    std::string topic_name_;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
};

OneDataSub::OneDataSub()
{
    std::cout << "start OneDataSub class" << std::endl;

    ros::NodeHandle private_nh("~");
    private_nh.param("topic_name", topic_name_, std::string("ring_cloud"));
    cloud_sub_ = nh_.subscribe(topic_name_, 1, &OneDataSub::cloudCallback, this);
}

void OneDataSub::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{
    std::cout << "call back point cloud" << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
    pcl::fromROSMsg(*msgs, cloud_pcl);

    for (int i=0;i<cloud_pcl.size();i++){
        std::cout << cloud_pcl.at(i).z << std::endl;
    }
    flag_ = 0;
}

int main(int argc, char **argv)
{
    std::cout << "start ring publisher node" << std::endl;
    ros::init(argc, argv, "one_data_sub");
    OneDataSub ods;
    ros::Rate r(10);
    while(ros::ok() && flag_){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
