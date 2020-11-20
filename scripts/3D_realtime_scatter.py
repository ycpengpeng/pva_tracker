#!/usr/bin/env python
# -*- coding: utf-8 -*-








import rospy
def get_random(low,high):
 return((high-low)*random.random()+low)

from matplotlib import pyplot as plt  # 用来绘制图形
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import random
import threading
from trajectory_msgs.msg import JointTrajectoryPoint




def thread_job():
    rospy.spin()

def callback2(data):
    global x, y, z
# for i in range(30000):
    plt.clf()  # 清除之前画的图
    #fig = plt.gcf()  # 获取当前图
   # ax = fig.gca(projection='3d')

   # ax.scatter(x[:-1], y[:-1], z[:-1],c='#00CED1') #之前点为蓝色
    #ax.scatter(x[-1:], y[-1:], z[-1:],c='#DC143C',label='now:({},{},{}) before:({},{},{})'.format(round(x[-1],2),round(y[-1],2),round(z[-1],2),round(x[-2],2),round(y[-2],2),round(z[-2],2)))   #显示点的值
   # ax.legend() #增加图例
    #ax.set_xlabel('X')
   # ax.set_ylabel('Y')
   # ax.set_zlabel('Z')
    #ax.set_xlim(0, 50)
   # ax.set_ylim(0, 50)
    #ax.set_zlim(0, 10)

    #ax.view_init(elev=20, azim=-73) ## Customize the view angle so it's easier to see that the scatter points lie on the plane y=0
    #plt.pause(0.5)  # 暂停一段时间
    #plt.ioff()  # 关闭画图窗口
    ## 更新数据
    x = []
    y = []
    z = []

    a=len(data.positions)

    for t_number in range(0,a/3):
        x1 = data.positions[3*t_number]
        y1 = data.positions[3*t_number+1]
        z1 = data.positions[3*t_number+2]
        x.append(x1)
        y.append(y1)
        z.append(z1)
        print('{} {} {} '.format(x1,y1,z1))
    plt.plot(x,y)
    plt.grid()
   # plt.xlabel('x轴，单位m')
   # plt.ylabel('y轴，单位m')
    plt.show()
    sys.exit(0)
    #while(1):
		#a=1
#



def plotter():
    rospy.init_node('plotter', anonymous=True)
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    rospy.Subscriber("/pva_setpoint", JointTrajectoryPoint, callback2)
    #rospy.sleep(1)
    rospy.spin()
   
	#while(1):
      #  plt.ion()# 创建实时动态窗口
      #  plt.show() #绘制完后不让窗口关闭

if __name__ == '__main__':
    plotter()
