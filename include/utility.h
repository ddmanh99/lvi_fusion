#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <Eigen/Dense>

// using namespace Eigen;

using namespace std; 

typedef pcl::PointXYZI PointType1;

class ParamServer
{
public:
    ros::NodeHandle nh; 

    std::string robot_id;

    // Topics 
    string laserTopic; 
    string imuTopic;
    string odomTopic;
    string pointCloudTopic;

    // Frames
    string lidarFrame;
    string basefootprintFrame;
    string baselinkFrame; 
    string odometryFrame;
    string mapFrame;

    // Lidar sensor Config
    int N_SCAN;
    float diff_scan;
    float min_diff_scan;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    double mappingProcessInterval;

    float z_tollerance; 
    float rotation_tollerance;

    bool useImuHeadingInitialization;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "ttb3");

        nh.param<std::string>("lvi_fusion/laserTopic", laserTopic, "scan");
        nh.param<std::string>("lvi_fusion/imuTopic", imuTopic, "imu");
        nh.param<std::string>("lvi_fusion/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lvi_fusion/pointCloudTopic", pointCloudTopic, "points_raw");

        nh.param<std::string>("lvi_fusion/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lvi_fusion/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lvi_fusion/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lvi_fusion/mapFrame", mapFrame, "map");

        nh.param<int>("lvi_fusion/N_SCAN", N_SCAN, 180);
        nh.param<float>("lvi_fusion/diff_scan", diff_scan, 50);
        nh.param<float>("lvi_fusion/min_diff_scan", min_diff_scan, 0.07);

        nh.param<float>("lvi_fusion/imuAccNoise", imuAccNoise, 1.7e-2);
        nh.param<float>("lvi_fusion/imuGyrNoise", imuGyrNoise, 2.0e-4);
        nh.param<float>("lvi_fusion/imuAccBiasN", imuAccBiasN, 0.001);
        nh.param<float>("lvi_fusion/imuGyrBiasN", imuGyrBiasN, 0.0000008);
        nh.param<float>("lvi_fusion/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lvi_fusion/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("lvi_fusion/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lvi_fusion/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("lvi_fusion/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        nh.param<double>("lvi_fusion/mappingProcessInterval", mappingProcessInterval, 0.15);
        nh.param<float>("lvi_fusion/z_tollerance", z_tollerance, 1000);
        nh.param<float>("lvi_fusion/rotation_tollerance", rotation_tollerance, 1000);

        nh.param<bool>("lvi_fusion/useImuHeadingInitialization", useImuHeadingInitialization, false);

        usleep(100);
    }
    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2angVel(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2liAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

// template<typename T>
// sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
// {
//     sensor_msgs::LaserScan tempCloud;
//     tempCloud = thisCloud;
//     tempCloud.header.stamp = thisStamp;
//     tempCloud.header.frame_id = thisFrame;
//     if (thisPub.getNumSubscribers() != 0)
//         thisPub.publish(tempCloud);
//     return tempCloud;
// }

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}