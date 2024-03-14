import csv 
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

# file_csv = '/home/manh/tm_ws/src/data/odometer.csv'

# arr = np.loadtxt(file_csv,delimiter=",")


class pubData():
    def __init__(self):
        self.file_odom = '/home/manh/tm_ws/src/data/odometer.csv'
        self.file_scan = '/home/manh/tm_ws/src/data/laser.csv'

        self.odom = Odometry()
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.scan = LaserScan()
        self.pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=10)

        self.arr_o = np.loadtxt(self.file_odom,delimiter=",")

        self.arr_s = np.loadtxt(self.file_scan,delimiter=",")
        self.i = 0

    def match_data(self):
        self.odom.header.frame_id="odom"
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = self.arr_o[self.i][0]
        self.odom.pose.pose.position.y = self.arr_o[self.i][1]
        self.theta = self.arr_o[self.i][2]
        self.odom.pose.pose.orientation.w = math.cos(0.5 * self.theta)
        self.odom.pose.pose.orientation.x = 0
        self.odom.pose.pose.orientation.y = 0
        self.odom.pose.pose.orientation.z = math.sin(0.5 * self.theta)
        

        self.scan.header.frame_id="base_scan"
        self.scan.header.stamp = rospy.Time.now()
        self.scan.angle_min = -1.5707999467849731
        self.scan.angle_max = 1.5707999467849731
        self.scan.angle_increment = 0.004361111
        self.scan.range_min = 0.1
        self.scan.range_max = 30.0
        self.scan.ranges = self.arr_s[self.i]
        
        self.i +=1
        self.publish_topic()
    
    def publish_topic(self):
        self.pub_odom.publish(self.odom)
        self.pub_scan.publish(self.scan)


if __name__ == "__main__":
    rospy.init_node('pub_data',anonymous=False)
    rate = 10
    r = rospy.Rate(rate)

    rospy.loginfo("\033[92mInit Node Pub Data from csv...\033[0m")

    p = pubData()
    while not rospy.is_shutdown():
        p.match_data()
        r.sleep()
    print(f'Len of data: {p.i}')
    rospy.loginfo("\033[92mPub data done!!!\033[0m")





    