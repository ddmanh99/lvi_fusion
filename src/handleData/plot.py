import matplotlib.pyplot as plt 

f = open('/home/manh/doan_ws/src/lvi_fusion/result/data/03_16.txt', "r")
lines = f.readlines()
a1=[]
a2=[]
for line in lines:
    numbers = line.split()
    a1.append(float(numbers[1]))
    a2.append(float(numbers[2]))
f.close()

g = open('/home/manh/doan_ws/src/lvi_fusion/result/data/compareData/odometry_.txt', "r")
lines = g.readlines()
b1=[]
b2=[]
for line in lines:
    numbers = line.split()
    b1.append(float(numbers[1]))
    b2.append(float(numbers[2]))
g.close()

plt.figure()
plt.plot(a1,a2,color='red', label='our method')
plt.plot(b1,b2,color='black',label='GR')
plt.legend()
plt.grid(True)
plt.show()