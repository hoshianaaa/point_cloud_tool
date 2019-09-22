#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>


class RingPublisher
{
public:
    RingPublisher();
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    std::string topic_name_;
    int ring_number_;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
};

RingPublisher::RingPublisher()
{
    std::cout << "start RingPublisher class" << std::endl;

    ros::NodeHandle private_nh("~");
    private_nh.param("topic_name", topic_name_, std::string("velodyne_points"));
    private_nh.param("ring_number", ring_number_, 0);

    cloud_sub_ = nh_.subscribe(topic_name_, 1, &RingPublisher::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ring_cloud", 1, false);
}

void RingPublisher::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{
    std::cout << "call back point cloud" << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
    cloud_pcl.width = 1;
    cloud_pcl.height = 1;
    cloud_pcl.points.resize(1);
    for (uint i=0;i<msgs->height * msgs->width;i+=1){
        int arrayPosX = i * msgs->point_step + msgs->fields[0].offset;
        int arrayPosY = i * msgs->point_step + msgs->fields[1].offset;
        int arrayPosZ = i * msgs->point_step + msgs->fields[2].offset;
        int ring = msgs->data[ i * msgs->point_step + msgs->fields[4].offset];
        if(ring == ring_number_){
            float x;
            float y;
            float z;

            memcpy(&x, &msgs->data[arrayPosX], sizeof(float));
            memcpy(&y, &msgs->data[arrayPosY], sizeof(float));
            memcpy(&z, &msgs->data[arrayPosZ], sizeof(float));

            pcl::PointXYZ p;
            p.x = x;
            p.y = y;
            p.z = z;
            cloud_pcl.push_back(p);
        }
    }
    sensor_msgs::PointCloud2 ring_cloud;
    pcl::toROSMsg(cloud_pcl, ring_cloud);
    ring_cloud.header.frame_id = "velodyne";
    cloud_pub_.publish(ring_cloud);
}

int main(int argc, char **argv)
{
    std::cout << "start ring publisher node" << std::endl;
    ros::init(argc, argv, "ring_publisher");
    RingPublisher rp;
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
