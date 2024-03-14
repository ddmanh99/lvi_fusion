#include "matplotlib_cpp.h"
#include "utility.h"
namespace plt = matplotlib_cpp;

class Plotting : public ParamServer
{
public:
    ros::Subscriber subImu;
    ros::Subscriber subOdom;

    nav_msgs::Odometry odom;

    std::vector<double> pos_x, pos_y;
    
    

    Plotting()
    {
        // subImu = nh.subscribe<sensor_msgs::Imu> (imuTopic, 1000, &IMUdata::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry> (odomTopic, 1000, &Plotting::odomHandler, this, ros::TransportHints().tcpNoDelay());
        if (ros::isShuttingDown())
        {
            Plotting::plot();
        }
    }

    void odomHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        odom = *odomMsg;
    }

    void process()
    {
        pos_x.push_back(odom.pose.pose.position.x);
        pos_y.push_back(odom.pose.pose.position.y);
    }

    void plot()
    {
        
        plt::fill(pos_x,pos_y,{});
        plt::show();
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_lo");

    ros::Rate rate(5);
    
    Plotting p;

    while (!ros::isShuttingDown)
    {
        p.process();
        rate.sleep();
    }
    

    // ROS_INFO("Start print");

}