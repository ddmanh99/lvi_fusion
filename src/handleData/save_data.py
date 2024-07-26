# USING TO SAVE DATA FROM TOPIC TO TEXT FILE

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import os 
import yaml

yaml_file = '/home/manh/doan_ws/src/lvi_fusion/config/param.yaml'

f = open(yaml_file, "r")
data = yaml.safe_load(f)
f.close()

data_dir = data['data_dir']

dataSave_dir = data_dir

class SaveData():
    def __init__(self):

        # directory file save 
        self.data_dir = dataSave_dir
        # check file exist 
        if os.path.exists(self.data_dir):
            os.remove(self.data_dir)
        #topic 
        self.odom_topic = rospy.Subscriber("/camera/odom/sample",Odometry,self.odom_callback)
        # self.imuInc_topic  = rospy.Subscriber("/odometry/imu_incremental",Odometry,self.imuInc_callback)
        # self.mapInc_topic = rospy.Subscriber("/mapping/odometry_incremental", Odometry, self.mapInc_callback)
        self.mapOdom_topic = rospy.Subscriber("/mapping/odometry", Odometry, self.mapOdom_callback)
        self.odom     = Odometry()
        self.imuInc   = Odometry()
        self.mapInc      = Odometry()
        self.mapOdom = Odometry()


        rospy.on_shutdown(self.info)

    def odom_callback(self, msg):
        self.odom = msg

    def imuInc_callback(self, msg):
        self.imuInc = msg
    
    def mapInc_callback(self, msg):
        self.mapInc = msg
    
    def mapOdom_callback(self, msg):
        self.mapOdom = msg

    def write_data(self):
        a = open(self.data_dir, 'a')
        # a.write(str(self.odom.header.seq) +" "+ str(self.odom.pose.pose.position.x) +" "+ str(self.odom.pose.pose.position.y)
        #         +" "+ str(self.imuInc.pose.pose.position.x) +" "+ str(self.imuInc.pose.pose.position.y)
        #         +" "+ str(self.mapInc.pose.pose.position.x) +" "+ str(self.mapInc.pose.pose.position.y) 
        #         +" "+ str(self.mapOdom.pose.pose.position.x) +" "+ str(self.mapOdom.pose.pose.position.y)+"\n")
        a.write(str(self.odom.header.stamp) +" "+ str(self.mapOdom.pose.pose.position.x) +" "+ str(self.mapOdom.pose.pose.position.y) +" "+str(self.mapOdom.pose.pose.position.z) +" "
                + str(self.mapOdom.pose.pose.orientation.x) +" "+ str(self.mapOdom.pose.pose.orientation.y) +" "+ str(self.mapOdom.pose.pose.orientation.z) +" "+ str(self.mapOdom.pose.pose.orientation.w) +"\n")
        a.close()



    def info(self):
        rospy.loginfo(f'\033[94mData result saved in\033[0m {self.data_dir}')


if __name__ == "__main__":
    rospy.init_node('save_data',anonymous=False)
    rate = 1
    r = rospy.Rate(rate)

    rospy.loginfo("\033[92mInit Node Saving...\033[0m")

    s = SaveData()
    while not rospy.is_shutdown():
        s.write_data()
        r.sleep()

    rospy.loginfo("\033[92mSave data in %s!!!\033[0m",dataSave_dir)
    rospy.loginfo("\033[35mPlease run \"rosrun lvi_fusion plot_data.py\" to show Image result \033[0m")