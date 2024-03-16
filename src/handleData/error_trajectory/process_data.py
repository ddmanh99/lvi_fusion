from decimal import Decimal
odom_dir = "/home/manh/doan_ws/src/lvi_fusion/result/data/compareData/odometry_.txt"
gmap_dir = "/home/manh/doan_ws/src/lvi_fusion/result/data/03_16.txt"
gmap = "/home/manh/doan_ws/src/lvi_fusion/result/data/compareData/our1.txt"

stamd = []

f = open(odom_dir, "r")
lines = f.readlines()
for line in lines:
    numbers = line.split()
    stamd.append(int(numbers[0]))
f.close()

x =[]
y =[]
z =[]
qx=[]
qy=[]
qz=[]
qw=[]
g = open(gmap_dir,"r")
glines = g.readlines()
for line in glines:
    numbers = line.split()
    
    # x.append(Decimal(numbers[0]))
    # y.append(Decimal(numbers[1]))
    # z.append(Decimal(numbers[2]))
    # qx.append(Decimal(numbers[3]))
    # qy.append(Decimal(numbers[4]))
    # qz.append(Decimal(numbers[5]))
    # qw.append(Decimal(numbers[6]))

    x.append(Decimal(numbers[1]))
    y.append(Decimal(numbers[2]))
    z.append(Decimal(numbers[3]))
    qx.append(Decimal(numbers[4]))
    qy.append(Decimal(numbers[5]))
    qz.append(Decimal(numbers[6]))
    qw.append(Decimal(numbers[7]))
g.close()

for i in range(len(stamd)):
    a=open(gmap, 'a')
    a.write(str(stamd[i]) +" "+ str(x[i]) +" "+ str(y[i]) +" "+ str(z[i])+" "
            + str(qx[i]) +" "+ str(qy[i]) +" "+ str(qz[i]) +" "+ str(qw[i]) +"\n")
    a.close()
