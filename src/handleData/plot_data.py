# USING TO PLOT DATA FROM TEXT FILE
from datetime import date, datetime
import matplotlib.pyplot as plt
import rospy
import numpy as np
import math 
import os 
import yaml

yaml_file = '/home/manh/doan_ws/src/lvi_fusion/config/param.yaml'

f = open(yaml_file, "r")
data = yaml.safe_load(f)
f.close()

data_dir = data['data_dir']
print("Data from :",data_dir)

today = date.today()
now = datetime.now()
day_str = now.strftime("%Y_%m_%d")
save_fol  = '/home/manh/doan_ws/src/lvi_fusion/result/img/'+day_str
if not os.path.exists(save_fol):
    os.makedirs(save_fol)

# dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
dt_string = now.strftime("%Hh%Mm%Ss")
# print("date and time =", dt_string)

save_dir  = save_fol+'/'+dt_string+'.png'


#odom
a1 = []
a2 = []
#/odometry/imu_incremental
b1 = []
b2 = []
# /mapping/odometry_incremental
c1 = []
c2 = [] 
#/mapping/odometry
d1 = []
d2 = []
# 
e1 = []
e2 = []

topic = ["/GR", "our_method", "gmapping", "karto slam"]
m = 50

f = open(data_dir, "r")

lines = f.readlines()
for line in lines:
    numbers = line.split()
    a1.append(float(numbers[1]) + 2.0)
    a2.append(float(numbers[2]) - 3.0)
    b1.append(float(numbers[3]))
    b2.append(float(numbers[4]))
    c1.append(float(numbers[5]))
    c2.append(float(numbers[6]))
    d1.append(float(numbers[7]))
    d2.append(float(numbers[8]))

f.close()

plt.figure()

plt.plot(a1[m:len(a1)], a2[m:len(a2)], color='black',label=topic[0], linewidth = 1.0)
plt.plot(b1[m:len(b1)], b2[m:len(b2)], color='red',label=topic[1], linestyle='dotted', linewidth=1.0)
plt.plot(c1[m:len(c1)], c2[m:len(c2)], color='blue', label=topic[2], linestyle='dashdot', linewidth=1.0)
plt.plot(d1[m:len(d1)], d2[m:len(d2)], color='green', label=topic[3], linestyle=(0,(1,1)), linewidth=1.0)

plt.legend()
plt.grid(True)
plt.title("Localization using Lidar Odometry and IMU based on Factor Graph")
plt.xlabel("x [m]")
plt.ylabel('y [m]')
# print(len(self.a),len(self.b),len(self.c),len(self.d))
plt.savefig(save_dir,dpi=500)
plt.show()

print(f'\033[94mImage results saved in\033[0m {save_dir}')

