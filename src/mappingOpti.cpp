#include "utility.h"
#include "lvi_fusion/cloud_info.h"
#include "slam2d_pose_graph.h"
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace cv;

typedef pcl::PointXY PointType;

typedef struct
{
    double theta;
    Eigen::Vector2d t;

} state2d;

Eigen::Vector2d point2eigen(PointType p)
{
    Eigen::Vector2d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    return pp;
}

PointType eigen2point(Eigen::Vector2d pp)
{
    PointType p;
    p.x = pp(0);
    p.y = pp(1);
    return p;
}


class mappingOpti : public ParamServer
{

public:
    ros::Subscriber subCloud;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    ros::Publisher pubLaserOdometryIncremental;

    ros::Publisher pubLaserOdometryGlobal;

    ros::Publisher pub_path;
    ros::Publisher pub_map2d;

    lvi_fusion::cloud_info cloudInfo;

    float transformTobeMapped[6];
    float laser[180];
    float laser_pre[180];

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    std::mutex mtx;

    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    pcl::PointCloud<PointType> scan;
    pcl::PointCloud<PointType> scan_prev;

    state2d delta;
    state2d state;

   

    bool isDegenerate = false;
    bool cvmap_vis_enable = false;

    pcl::PointCloud<PointType1>::Ptr cloudKeyPoses3D;

    nav_msgs::OccupancyGrid map2d;
    Mat cvmap2d;



    // Vector2d world2map(Vector2d p);
    // cv::Point2i world2map(cv::Point2f p);



    mappingOpti()
    {
        subCloud = nh.subscribe<lvi_fusion::cloud_info>("lvi_fusion/cloud_info", 5, &mappingOpti::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom     = nh.subscribe<nav_msgs::Odometry>("/odom", 2000, &mappingOpti::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("/mapping/odometry_incremental", 5);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry> ("/mapping/odometry", 5);
        pub_path = nh.advertise<nav_msgs::Path>("/path", 100);
        pub_map2d = nh.advertise<nav_msgs::OccupancyGrid>("/map", 100);
        allocateMemory();
    }
    
    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType1>());
        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0.0;
        }
        for (int i = 0; i < 180; ++i){
            laser[i] = 0.0;
            laser_pre[i] = 0.0;
        }

        state.t = Vector2d::Zero();
        state.theta = 0;
        map2d.header.frame_id = odometryFrame;
        map2d.info.width = 500;
        map2d.info.height = 500;
        map2d.info.resolution = 0.15;
        map2d.info.origin.orientation.w = 1;
        map2d.info.origin.orientation.x = 0;
        map2d.info.origin.orientation.y = 0;
        map2d.info.origin.orientation.z = 0;
        map2d.info.origin.position.x = -0.5 * map2d.info.width * map2d.info.resolution;
        map2d.info.origin.position.y = -0.5 * map2d.info.height * map2d.info.resolution;
        map2d.info.origin.position.z = 0;
        map2d.data.resize(map2d.info.width * map2d.info.height);
        cvmap2d = Mat(map2d.info.width, map2d.info.height, CV_8SC1, -1);
        cvmap2map();
    }
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        odomQueue.push_back(*odometryMsg);
    }

    void laserCloudInfoHandler(const lvi_fusion::cloud_infoConstPtr& msgIn)
    {
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        cloudInfo = *msgIn;

        std::lock_guard<std::mutex> lock(mtx);
        static double timeLastProcessing = -1;
        if ( timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;
            std::cout<<"----------------"<<std::endl;
            

            updateInitialGuess();

            processScan();

            update();

            publishOdometry();

            publish_map();
        }
    }

    void update()
    {
        if (scan.points.size() && scan_prev.points.size())
        {
            int sum = 0;
            for (int i = 0; i < 180; ++i)
            {
                if ( abs(laser[i] - laser_pre[i]) > min_diff_scan )
                {
                    sum++;
                }
            }
            std::cout<<"sum: "<<sum<<std::endl;
            if (sum>diff_scan)
            {
                scan_match();
                update_transform();
                // std::cout<<"1"<<std::endl;
                scan_map_match_random();
            }
            else
            {
                while (!odomQueue.empty())
                {
                    if (odomQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.01)
                        odomQueue.pop_front();
                    else
                        break;
                }

                if (odomQueue.empty())
                {
                    std::cout<<"odomQueue is empty !!!"<<std::endl;
                    return;
                }
                
                // if (odomQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.015)
                // {
                //     std::cout<<odomQueue.front().header.stamp.toSec()<<" "<< timeLaserInfoCur<<std::endl;
                //     std::cout<<"odomQueue is timeout"<<std::endl;
                //     return;
                // }

                nav_msgs::Odometry odomFactor;

                for (int i=0; i<(int)odomQueue.size(); ++i)
                {
                    odomFactor = odomQueue[i];

                    if(ROS_TIME(&odomFactor)<timeLaserInfoCur)
                        continue;
                    else
                        break;
                }

                tf::Quaternion orientation;
                tf::quaternionMsgToTF(odomFactor.pose.pose.orientation, orientation);

                double roll,pitch,yaw;
                tf::Matrix3x3(orientation).getRPY(roll,pitch,yaw);

                transformTobeMapped[0] = roll;
                transformTobeMapped[1] = pitch;
                transformTobeMapped[2] = yaw;
                transformTobeMapped[3] = odomFactor.pose.pose.position.x;
                transformTobeMapped[4] = odomFactor.pose.pose.position.y + 0.25;
                transformTobeMapped[5] = odomFactor.pose.pose.position.z;
                transformUpdate();
            }
            update_map();
        
        }

        // if (scan.points.size() && scan_prev.points.size())
        // {
        //     scan_match();
        //     update_transform();
        //     // std::cout<<"1"<<std::endl;
        //     scan_map_match_random();
        //     // std::cout<<"2"<<std::endl;
        //     update_map();
        //     // std::cout<<"3"<<std::endl;
        // }

        if (scan.points.size())
        {
            scan_prev = scan;
            for (int i=0; i<180; i++)
            {
                laser_pre[i] = laser[i];
            }
        }
    }

    void publish_map()
    {
        map2d.header.stamp = timeLaserInfoStamp;
        pub_map2d.publish(map2d);
    }

    void publishOdometry()
    {
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        // std::cout<<"en "<<laserOdometryROS.pose.pose.position.x<<std::endl;
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental;
        static Eigen::Affine3f increOdomAffine;

        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            // std::cout<<laserOdometryROS.pose.pose.position.x<<std::endl;

            increOdomAffine = trans2Affine3f(transformTobeMapped);
        }
        else
        {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        // std::cout<<"end "<<laserOdomIncremental.pose.pose.position.x<<std::endl;
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
        // std::cout<<laserOdomIncremental<<std::endl;

        ////
        static nav_msgs::Path path;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = timeLaserInfoStamp;
        pose.header.frame_id = odometryFrame;
        double theta = state.theta;
        pose.pose.orientation.w = laserOdometryROS.pose.pose.orientation.w;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = laserOdometryROS.pose.pose.orientation.z;
        pose.pose.position.x = laserOdometryROS.pose.pose.position.x;
        pose.pose.position.y = laserOdometryROS.pose.pose.position.y;
        pose.pose.position.z = 0;
        // pub_pose.publish(pose);

        path.header.frame_id = odometryFrame;
        path.poses.push_back(pose);
        pub_path.publish(path);

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped tr;
        tr.header.stamp = timeLaserInfoStamp;
        tr.header.frame_id = odometryFrame;
        tr.child_frame_id = baselinkFrame;
        tr.transform.translation.x = laserOdometryROS.pose.pose.position.x;
        // std::cout<<"publish: "<<tr.transform.translation.x<<std::endl;
        tr.transform.translation.y = laserOdometryROS.pose.pose.position.y;
        tr.transform.translation.z = 0;
        tr.transform.rotation.x = laserOdometryROS.pose.pose.orientation.x;
        tr.transform.rotation.y = laserOdometryROS.pose.pose.orientation.y;
        tr.transform.rotation.z = laserOdometryROS.pose.pose.orientation.z;
        tr.transform.rotation.w = laserOdometryROS.pose.pose.orientation.w;

        // std::cout<<tr.transform.translation.x<<std::endl;
        br.sendTransform(tr);
    }

    void bresenham(Point2i p1, Point2i p2)
{
    //drawing a line from p1 to p2
    int dx = abs(p2.x - p1.x);
    int sx = (p2.x > p1.x) ? 1 : -1;
    int dy = abs(p2.y - p1.y);
    int sy = (p2.y > p1.y) ? 1 : -1;
    int err = (dx > dy ? dx : dy) / 2;
    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;

    while (x1 != x2 && y1 != y2)
    {
        if (cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) == 100)
        {
            break;
        }
        else if (cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) == -1)
        {
            cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) = 0;
        }
        int e2 = err;
        if (e2 > -dx) 
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y1 += sy;
        }
    }
}

    void cvmap2map()
    {
        for (int i = 0; i < cvmap2d.rows; i++)
    {
        for(int j = 0; j < cvmap2d.cols; j++)
        {
            map2d.data[i * map2d.info.width + j] = cvmap2d.at<int8_t>(i, j);
        }
    }
    if (cvmap_vis_enable)
    {
        imshow("cvmap2d", cvmap2d);
        waitKey(2);
    }
    }
    void update_map()
    {
        cv::Point2f tt;
    tt.x = transformTobeMapped[3];
    tt.y = transformTobeMapped[4];
    cv::Point2i origin = world2map(tt);
    if (origin.x < 0 || origin.x >= cvmap2d.cols || origin.y < 0 || origin.y >= cvmap2d.rows) return;
    Eigen::Matrix2d R;
    R(0, 0) = cos(transformTobeMapped[2]); R(0, 1) = -sin(transformTobeMapped[2]);
    R(1, 0) = sin(transformTobeMapped[2]); R(1, 1) =  cos(transformTobeMapped[2]);
    for (int i = 0; i < scan.points.size(); i++)
    {
        PointType p = scan.points[i];
        float dist = sqrtf(p.x * p.x + p.y * p.y);
        if (dist > 20) continue;
        Vector2d t(transformTobeMapped[3],transformTobeMapped[4]);
        Eigen::Vector2d pp = R * point2eigen(p) + t;
        Point2f ppp(pp(0), pp(1));

        cv:Point2i pt = world2map(ppp);

        if (pt.x < 0 || pt.x >= cvmap2d.cols || pt.y < 0 || pt.y >= cvmap2d.rows)
            return;

        bresenham(origin, pt);
        cvmap2d.at<int8_t>(pt.y * cvmap2d.cols + pt.x) = 100; //100->occupancied
    }
    cvmap2map();
    }

    int scan_map_match_score(Vector3d pose)
    {
        int score = 0;
        Eigen::Matrix2d R;
        Vector2d t(pose(1), pose(2));
        double theta = pose(0);
        R(0, 0) = cos(theta); R(0, 1) = -sin(theta);
        R(1, 0) = sin(theta); R(1, 1) =  cos(theta);
    //printf("cols: %d, rows: %d\n", cvmap2d.cols, cvmap2d.rows);
        for (int i = 0; i < scan.points.size(); i++)
        {
            Vector2d p = point2eigen(scan.points[i]);
            Vector2d pp = world2map(R * p + t);
        //cout << "pp: " << pp.transpose() << endl;
            if ((pp(0) <= 1) || (pp(0) >= cvmap2d.cols) || (pp(1) <= 1) || (pp(1) >= cvmap2d.rows))
            {
                continue;
            } else 
        {
            //get value from map
            int x = round(pp(0));
            int y = round(pp(1));
            if (cvmap2d.at<int8_t>(y * cvmap2d.cols + x) == 100)
            {
                score++;
            }
            //printf("i: %d, res:%f\n", i, residual[i]);
        }
    }
    //generate local map and compute local optimal?
    return score;
    }

    void scan_map_match_random()
    {
        Vector3d pose(transformTobeMapped[2], transformTobeMapped[3], transformTobeMapped[4]);
        double eps = 1e-5;
    //search best mattch
        int N = 200;

        for (int i = 0; i < N; i++)
        {
        //random direction
            Vector3d d = Vector3d::Random();
            d(0) /= 10.0;
            d.normalize();
            double min_len = 0;
            double max_len = 0.2;
        //search best len
            while((max_len - min_len) > eps)
            {
                int score1 = scan_map_match_score(pose + d * min_len);
                int score2 = scan_map_match_score(pose + d * max_len);
                if (score1 >= score2)
                {
                    max_len = (min_len + max_len) / 2.0;
                } else 
                {
                    min_len = (min_len + max_len) / 2.0;
                }
            }
            pose += d * min_len;
            Vector3d dx = d * min_len;
            int score = scan_map_match_score(pose);
            // printf("score: %d, min_len: %lf\n", score, min_len);
            // cout << "dx: " << dx.transpose() << endl;
    }
    //update to state
    // state.theta = pose(0);
    // state.t = pose.bottomRows(2);
    transformTobeMapped[0] += 0.0;
        transformTobeMapped[1] += 0.0;
        transformTobeMapped[2] = pose(0);
        transformTobeMapped[3] = pose(1);
        transformTobeMapped[4] = pose(2);
        transformTobeMapped[5] += 0.0;

        // std::cout<<"delta "<<delta.t(0)<<std::endl;

        // std::cout<<"3 "<<transformTobeMapped[3]<<std::endl;

        PointType1 thisPose3D;
        thisPose3D.x = transformTobeMapped[3];
        thisPose3D.y = transformTobeMapped[4];
        thisPose3D.z = transformTobeMapped[5];
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        transformUpdate();
    }

    void update_transform()
    {
        Eigen::Matrix2d dR;
        dR(0, 0) = cos(delta.theta); dR(0, 1) = -sin(delta.theta);
        dR(1, 0) = sin(delta.theta); dR(1, 1) = cos(delta.theta);


        Eigen::Vector2d dt_inv = -dR.transpose() * delta.t;
        Eigen::Matrix2d dR_inv = dR.transpose();
    

        Eigen::Matrix2d R;
        R(0, 0) = cos(transformTobeMapped[2]); R(0, 1) = -sin(transformTobeMapped[2]);
        R(1, 0) = sin(transformTobeMapped[2]); R(1, 1) =  cos(transformTobeMapped[2]);
        transformTobeMapped[2] += (-delta.theta);
        Vector2d t(transformTobeMapped[3], transformTobeMapped[4]);
        t += R * dt_inv;
        transformTobeMapped[3] = t(0);
        transformTobeMapped[4] = t(1);
    }
    // void update_transform()
    // {
    //     transformTobeMapped[0] += 0.0;
    //     transformTobeMapped[1] += 0.0;
    //     transformTobeMapped[2] += delta.theta;
    //     transformTobeMapped[3] += delta.t(0);
    //     transformTobeMapped[4] += delta.t(1);
    //     transformTobeMapped[5] += 0.0;

    //     // std::cout<<"delta "<<delta.t(0)<<std::endl;

    //     // std::cout<<"3 "<<transformTobeMapped[3]<<std::endl;

    //     PointType1 thisPose3D;
    //     thisPose3D.x = transformTobeMapped[3];
    //     thisPose3D.y = transformTobeMapped[4];
    //     thisPose3D.z = transformTobeMapped[5];
    //     thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    //     cloudKeyPoses3D->push_back(thisPose3D);

    //     transformUpdate();
    // }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
        // std::cout<<"4 "<<transformTobeMapped[3]<<std::endl;
        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    void scan_match()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        double pose[3] = {0};
        if (scan.points.size() && scan_prev.points.size())
        {

            Problem problem;
            //solve delta with ceres constraints
            pcl::KdTreeFLANN<PointType> kdtree;
            kdtree.setInputCloud(scan.makeShared());
            int K = 2; // K nearest neighbor search
            std::vector<int> index(K);
            std::vector<float> distance(K);
            //1. project scan_prev to scan

            Eigen::Matrix2d R;
            R(0, 0) = cos(delta.theta); R(0, 1) = -sin(delta.theta);
            R(1, 0) = sin(delta.theta); R(1, 1) = cos(delta.theta);
            Eigen::Vector2d dt = delta.t;
            //find nearest neighur
            for (int i = 0; i < (int)scan_prev.points.size(); i++)
            {
                PointType search_point = scan_prev.points[i];
                Eigen::Vector2d p = point2eigen(search_point);
                float a = p[0];
            
                if(abs(a)>5.0)
                {
                    continue;
                }
            
                //project search_point to current frame
                PointType search_point_predict = eigen2point(R * point2eigen(search_point) + dt);
                if (kdtree.nearestKSearch(search_point_predict, K, index, distance) == K)
                {
                    //add constraints
                    Eigen::Vector2d p = point2eigen(search_point);
                    Eigen::Vector2d p1 = point2eigen(scan.points[index[0]]);
                    Eigen::Vector2d p2 = point2eigen(scan.points[index[1]]);
                    ceres::CostFunction *cost_function = lidar_edge_error::Create(p, p1, p2);
                    problem.AddResidualBlock(cost_function,
                                         new CauchyLoss(0.5),
                                         pose);
                }
            
            }
        
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = false;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.FullReport() << "\n";

            // printf("translate(theta, x, y): %lf, %lf, %lf\n", pose[0], pose[1], pose[2]);

            delta.theta = pose[0];
            delta.t(0) = -pose[1];
            delta.t(1) = -pose[2];
            

            // std::cout<<"delta x,y : "<<delta.t(0)<<" "<<delta.t(1)<<std::endl;
        }

    }

    void processScan()
    {
        scan.points.resize(cloudInfo.scan.ranges.size());
        for (auto i = 0; i < (int)cloudInfo.scan.ranges.size(); i++)
        {
            float dist = cloudInfo.scan.ranges[i]; 
            laser[i] = dist;
            float theta = cloudInfo.scan.angle_min + i * cloudInfo.scan.angle_increment;
            scan.points[i].x = dist * cos(theta);
            scan.points[i].y = dist * sin(theta);
            
        }

        scan.width = scan.points.size();
        scan.height = 1;
        scan.is_dense = true;
    }

    cv::Point2i world2map(cv::Point2f p)
    {
        cv::Point2i m;
        m.x = roundf(p.x / map2d.info.resolution + map2d.info.width * 0.5);
        m.y = roundf(p.y / map2d.info.resolution + map2d.info.height * 0.5);
        return m;
    }

    Vector2d world2map(Vector2d p)
    {
        Vector2d m;
        m = p / map2d.info.resolution;
        m(0) += map2d.info.width * 0.5;
        m(1) += map2d.info.height * 0.5;
        return m;
    }

    void updateInitialGuess()
    {
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;

        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }

        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;

        
        // std::cout<<"1 "<<transformTobeMapped[3]<<std::endl;

        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                // std::cout<<"2.1 "<<transformTobeMapped[3]<<std::endl;
                return;
            }
        }

        if (cloudInfo.imuAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            // std::cout<<"2.2 "<<transformTobeMapped[3]<<std::endl;
            return;
        }
        
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lvi_fusion");

    mappingOpti MO;
    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    ros::spin();

    return 0;
}