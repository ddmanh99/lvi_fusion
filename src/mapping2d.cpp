#include "mapping2d.h"

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_broadcaster.h"
#include <Eigen/Eigen>
#include "lvi_fusion/cloud_info.h"

slam2d slam;
ros::Publisher pub_pose, pub_path;
ros::Publisher pub_laserscan;
ros::Publisher pub_map2d;
void publish_pose(slam2d &slam);
void publish_map2d(slam2d &slam);


void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    slam.readin_scan_data(msg);
    slam.update();
    publish_pose(slam);
    publish_map2d(slam);
}

void publish_map2d(slam2d &slam)
{
    slam.map2d.header.stamp = ros::Time(slam.timestamp);
    pub_map2d.publish(slam.map2d);
}

void odom_callback(const nav_msgs::OdometryConstPtr &odomMsg)
{
    slam.readin_odom_data(odomMsg);
}

void publish_pose(slam2d &slam)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(slam.timestamp);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "odom_lidar";
    double theta = slam.state_out.theta;
    odom.pose.pose.orientation.w = cos(0.5 * theta);
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = sin(0.5 * theta);
    odom.pose.pose.position.x = slam.state_out.t(0);
    odom.pose.pose.position.y = slam.state_out.t(1);
    odom.pose.pose.position.z = 0;
    // std::cout<<"state_out: "<<slam.state_out.t(0)<<" "<<slam.state_out.t(1)<<std::endl;
    pub_pose.publish(odom);

    static nav_msgs::Path path;
    geometry_msgs::PoseStamped pose_;
    pose_.header.frame_id = "odom";
    pose_.pose.orientation.w = cos(0.5 * theta);
    pose_.pose.orientation.x = 0;
    pose_.pose.orientation.y = 0;
    pose_.pose.orientation.z = sin(0.5 * theta);
    pose_.pose.position.x = slam.state_out.t(0);
    pose_.pose.position.y = slam.state_out.t(1);
    pose_.pose.position.z = 0;

    path.header.frame_id = "odom";
    path.poses.push_back(pose_);
    pub_path.publish(path);

    //send transfrom
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr;
    tr.header.stamp = ros::Time(slam.timestamp);
    tr.header.frame_id = "odom";
    tr.child_frame_id = "base_link";
    tr.transform.translation.x = slam.state_out.t(0);
    tr.transform.translation.y = slam.state_out.t(1);
    tr.transform.translation.z = 0;
    tr.transform.rotation.x = pose_.pose.orientation.x;
    tr.transform.rotation.y = pose_.pose.orientation.y;
    tr.transform.rotation.z = pose_.pose.orientation.z;
    tr.transform.rotation.w = pose_.pose.orientation.w;
    br.sendTransform(tr);

    static tf::TransformBroadcaster base_link2base_scan;
    static tf::Transform base_to_scan = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0.4, 0, 0));
    base_link2base_scan.sendTransform(tf::StampedTransform(base_to_scan, tr.header.stamp, "base_link", "base_scan"));
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarOdometry");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/odometry/imu_incremental", 5, odom_callback);
    ros::Subscriber sub_laserscan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserscan_callback);
    

    
    pub_pose = nh.advertise<nav_msgs::Odometry>("/mapping/lidar_odometry_incre", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 1);
    pub_map2d = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ROS_INFO("\033[1;32m----> Mapping 2D Started.\033[0m");
    ros::spin();
    return 0;
}