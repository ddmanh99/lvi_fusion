import matplotlib.pyplot as plt 

f = open('/home/manh/doan_ws/src/lvi_fusion/result/data/compareData/ourm_align.txt', "r")
lines = f.readlines()
a1=[]
a2=[]
for line in lines:
    numbers = line.split()
    a1.append(float(numbers[1]))
    a2.append(float(numbers[2])+0.25)
f.close()

gr = open('/home/manh/doan_ws/src/lvi_fusion/result/data/compareData/odometry_.txt', "r")
lines = gr.readlines()
b1=[]
b2=[]
for line in lines:
    numbers = line.split()
    b1.append(float(numbers[1]))
    b2.append(float(numbers[2]))
gr.close()

gm = open('/home/manh/doan_ws/src/lvi_fusion/result/data/compareData/gmapping_.txt', "r")
lines = gm.readlines()
c1=[]
c2=[]
for line in lines:
    numbers = line.split()
    c1.append(float(numbers[1]))
    c2.append(float(numbers[2]))
gm.close()

ka = open('/home/manh/doan_ws/src/lvi_fusion/result/data/compareData/ka_align.txt', "r")
lines = ka.readlines()
d1=[]
d2=[]
for line in lines:
    numbers = line.split()
    d1.append(float(numbers[1]))
    d2.append(float(numbers[2])+0.25)
ka.close()


plt.figure()

labels = ['Ground truth', 'Our method', 'GMapping', 'Karto SLAM']

plt.plot(b1,b2,color='black',label=labels[0])
plt.plot(a1,a2,color='red', label=labels[1])
plt.plot(c1,c2,color='blue',label=labels[2])
plt.plot(d1,d2,color='green',label=labels[3])
plt.legend()
plt.xlabel("x [m]")
plt.ylabel('y [m]')
# plt.title()
plt.grid(True)
plt.savefig('/home/manh/doan_ws/src/lvi_fusion/result/img/2024_03_18/reslults.png',dpi=600)

plt.show()