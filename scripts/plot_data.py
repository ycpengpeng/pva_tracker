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

# plt.rc('font',family='cmr10')
file = "/home/pengpeng/catkin_ws/record_plot_data/plot_data3.csv"

data = pd.read_csv(file)

data_array=np.array(data)

figure=plt.figure(figsize=(20,2),dpi=300)
left =0
right =600
time=np.arange(right-left)/30.0

# axes1=figure.add_subplot(2,1,1,projection='3d')
# axes1.plot3D(data_array[:,0],data_array[:,1],data_array[:,2])
# axes1.plot3D(data_array[:,9],data_array[:,10],data_array[:,11])
# plt.xlabel("aa")


axes2=figure.add_subplot(1,1,1)
axes2.plot(time,data_array[left:right,18])
axes2.plot(time,data_array[left:right,35])

plt.xlabel('Time(s)', fontdict={'family' : 'cmr10', 'size'   : 8})
plt.ylabel('$\Delta{p}_{k+1}^x$(m)', fontdict={'family' : 'cmr10', 'size'   : 8})
plt.grid()
plt.legend(labels=['Real','Prediction'],loc=0, ncol=1, mode="None",prop={'family':'cmr10', 'size':8})

plt.show(figure)
figure.savefig('test.png',dpi=300,bbox_inches="tight")
print("aaa")

