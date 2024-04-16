


odom_dir = ""
gmap=""
stamp =[]
x =[]
y =[]
z =[]
qx=[]
qy=[]
qz=[]
qw=[]
f = open(odom_dir, "r")
lines = f.readlines()
for line in lines:
    numbers = line.split()
    stamp.append(int(numbers[0]))
    x.append(float(numbers[1]))
    y.append(float(numbers[2])+0.25)
    z.append(float(numbers[3]))
    qx.append(float(numbers[4]))
    qy.append(float(numbers[5]))
    qz.append(float(numbers[6]))
    qw.append(float(numbers[7]))
f.close()
for i in range(len(stamp)):
    a=open(gmap, 'a')
    a.write(str(stamp[i]) +" "+ str(x[i]) +" "+ str(y[i]) +" "+ str(z[i])+" "
            + str(qx[i]) +" "+ str(qy[i]) +" "+ str(qz[i]) +" "+ str(qw[i]) +"\n")
    a.close()