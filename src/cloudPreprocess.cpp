#include "utility.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include "lvi_fusion/cloud_info.h"

const int queueLength = 2000;

class CloudPrepocess : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odomLock;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    ros::Subscriber subScan;

    ros::Publisher pubLaserCloudInfo;

    std::deque<sensor_msgs::LaserScan> cloudQueue;
    sensor_msgs::LaserScan curCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    
    int imuPointerCur;

    pcl::PointCloud<pcl::PointXY>::Ptr laserCloudIn;

    int deskewFlag = 0;
    double delta_time = 0.0;
    
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    lvi_fusion::cloud_info cloudInfo;

public: 
    nav_msgs::OccupancyGrid map2d;
    CloudPrepocess(): 
    deskewFlag(0)
    {
        // std::cout<<deskewFlag;
        subImu      = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &CloudPrepocess::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom     = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &CloudPrepocess::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subScan    = nh.subscribe<sensor_msgs::LaserScan>("/scan", 5, &CloudPrepocess::scanHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudInfo = nh.advertise<lvi_fusion::cloud_info> ("lvi_fusion/cloud_info", 1);
        allocateMemory();
        resetParameters();
    }

    void allocateMemory()
    {
        resetParameters();
    }

    void resetParameters()
    {
        imuPointerCur = 0;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~CloudPrepocess(){}
    
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odomLock);
        odomQueue.push_back(*odometryMsg);
    }

    void scanHandler(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
    {
        if(!cachePointCloud(scanMsg))
            return;
        
        if (!deskewInfo())
            return;

        publishClouds();
        
        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::LaserScanConstPtr& cloudMsg)
    {
        cloudQueue.push_back(*cloudMsg);
        if (cloudQueue.size() <= 2)
            return false;
        
        curCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        cloudHeader = curCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + 0.0; 

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odomLock);

        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false; 
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;
        
        imuPointerCur = 0;

        for ( int i = 0; i<(int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);
            
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;
        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;

    }
    
    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll,pitch,yaw;
        tf::Matrix3x3(orientation).getRPY(roll,pitch,yaw);

        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.scan = curCloudMsg;
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_pre_process");

    CloudPrepocess CP;

    ROS_INFO("\033[1;32m----> Cloud Pre-processing Started.\033[0m");
    
    ros::spin();

    return 0;
}