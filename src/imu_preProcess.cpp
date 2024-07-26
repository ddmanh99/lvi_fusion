#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuMerger
{
public:
    ImuMerger()
    {
        // Subscribe to gyro and accel topics
        gyro_sub_ = nh_.subscribe("/camera/gyro/sample", 10, &ImuMerger::gyroCallback, this);
        accel_sub_ = nh_.subscribe("/camera/accel/sample", 10, &ImuMerger::accelCallback, this);

        // Publisher for the combined IMU data
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 10);
    }

private:
    void gyroCallback(const sensor_msgs::Imu::ConstPtr& gyro_msg)
    {
        gyro_data_ = *gyro_msg;
        publishImu();
    }

    void accelCallback(const sensor_msgs::Imu::ConstPtr& accel_msg)
    {
        accel_data_ = *accel_msg;
        publishImu();
    }

    void publishImu()
    {
        // if (gyro_data_ && accel_data_)
        // {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_link";

            imu_msg.orientation = gyro_data_.orientation;
            imu_msg.angular_velocity = gyro_data_.angular_velocity;
            imu_msg.linear_acceleration = accel_data_.linear_acceleration;

            imu_pub_.publish(imu_msg);
        // }
    }

    ros::NodeHandle nh_;
    ros::Subscriber gyro_sub_;
    ros::Subscriber accel_sub_;
    ros::Publisher imu_pub_;

    sensor_msgs::Imu gyro_data_;
    sensor_msgs::Imu accel_data_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_merger");
    ImuMerger imu_merger;
    ros::spin();
    return 0;
}
