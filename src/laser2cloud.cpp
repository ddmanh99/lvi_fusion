#include "utility.h"

typedef pcl::PointXY PointType;

class ConvertCloud : public ParamServer
{
public:
    ros::Subscriber subLaser;

    ros::Publisher pubCloud;

    pcl::PointCloud<PointType> cloud;

    std_msgs::Header cloudHeader;

    sensor_msgs::PointCloud2 a;

    ConvertCloud()
    {
        subLaser = nh.subscribe<sensor_msgs::LaserScan>(laserTopic, 10, &ConvertCloud::laserHandler, this, ros::TransportHints().tcpNoDelay());

        pubCloud = nh.advertise<sensor_msgs::PointCloud2>(pointCloudTopic, 10);
    }

    void laserHandler(const sensor_msgs::LaserScanConstPtr &laserMsg)
    {
        // std::cout<<"-1"<<std::endl;
        cloud.points.resize(laserMsg->ranges.size());
        for (auto i=0; i<laserMsg->ranges.size(); i++)
        {
            float dist = laserMsg->ranges[i];
            float theta = laserMsg->angle_min + i*laserMsg->angle_increment;
            cloud.points[i].x = dist * cos(theta);
            cloud.points[i].y = dist * sin(theta);
        }
        cloudHeader = laserMsg->header;
        cloud.height = 1;
        cloud.width = laserMsg->ranges.size();

        cloud.is_dense = true;
        // std::cout<<"0"<<std::endl;
        PublishCloud();
    }

    void PublishCloud()
    {
        pcl::toROSMsg(cloud, a);
        a.header.stamp = cloudHeader.stamp;
        a.header.frame_id = cloudHeader.frame_id;
        pubCloud.publish(a);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_to_cloud");
    ConvertCloud CC;
    ROS_INFO("\033[1;32m----> Convert Laser to Cloud Started.\033[0m");
   
    ros::spin();

    return 0;
}