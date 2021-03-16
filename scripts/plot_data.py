#!/usr/bin/env python
# -*- coding: utf-8 -*-


import pandas as pd
import numpy as np
import matplotlib as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# planned_p 0 1 2 plan_v 3 4 5 plan_a 6 7 8

# cur_p 9 10 11 cur_v 12 13 14 cur_a 15 16 17

# delta_p 18 19 20 delta_v 21 22 23 delta_a 24 25 26

# q 27 28 29 30 euler_angle 31 32 33 thrust 34

# pre_x 35 pre_y 36 pre_z 37 pre_vx vy vz 38 39 40

plt.rc('font',family='STIXGeneral')
file = "/home/pengpeng/catkin_ws/real_drone1/fake_random_fly_record_03_14_20_18_55.csv"

data = pd.read_csv(file)


data_array=np.array(data)


print('M(18) mean: ',np.mean(abs(data_array[:,18])))

file = "/home/pengpeng/catkin_ws/record_plot_data/11.csv"
data = pd.read_csv(file)
data_array2=np.array(data)

print('M(18) mean: ',np.mean(abs(data_array2[:,18])))

print('data_array size:',data_array.shape)
print('data_array2 size:',data_array2.shape)

figure=plt.figure(figsize=(15,4),dpi=300)
left =5
right =1500
time=np.arange(right-left)/30.0

axes1=figure.add_subplot(1,1,1)
#axes1.plot(data_array[:,0],'bo--')
axes1.plot(data_array[:,1],'r')
axes1.plot(data_array[:,10],'b')
#axes1.plot3D(data_array[:,9],data_array[:,10],data_array[:,11],'r')
#axes1.plot3D(data_array2[:,9],data_array2[:,10],data_array2[:,11],'g')

# axes1.plot(time,data_array[left:right,1])
# axes1.plot(time,data_array[left:right,10])

# tmp=data_array[:,12]*data_array[:,12]+data_array[:,13]*data_array[:,13]+data_array[:,14]*data_array[:,14]

# aa=np.sqrt(tmp)
# axes1.plot(time,aa[left:right])
plt.xlabel("x")
plt.ylabel("y")



# axes1=figure.add_subplot(2,1,2)
# axes1.plot(time,data_array[left:right,3])
# axes1.plot(time,data_array[left:right,12])

# axes2=figure.add_subplot(2,1,2)
# axes2.plot(time,data_array2[left:right,1])
# axes2.plot(time,data_array2[left:right,10])




# axes2.plot(time,data_array[left:right,3])

# plt.xlabel('Time(s)', fontdict={'family' : 'STIXGeneral', 'size'   : 12})
# plt.ylabel('$\Delta{p}_{k+1}^x$(m)', fontdict={'family' : 'STIXGeneral', 'size'   : 12})

plt.grid()

# plt.legend(labels=['Real','Prediction'], ncol=2, mode="expand",prop={'family':'STIXGeneral', 'size':12},           labelspacing=0.1)

plt.xticks(fontproperties = 'STIXGeneral', size = 12)
plt.yticks(fontproperties = 'STIXGeneral', size = 12)

plt.show()
figure.savefig('pred_z.png',dpi=300,bbox_inches="tight")
print("picture has been saved  ")

