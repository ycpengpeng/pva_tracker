#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch



import pandas as pd
import numpy as np
import torch.nn.functional as F
import matplotlib as plt
from torch.autograd import Variable
import torch.utils.data as Data
import random
import rospy
import copy
import multiprocessing as mp

import math
import time
from pva_tracker.msg import input


def accelerate2euler(a_des):

    v1=np.array([0,0,1])

    v2=np.array([a_des[0],a_des[1],a_des[2]])#期望加速度

    quaternion=np.ones(4)
    tmp=np.cross(v1, v2)[0:3]

    quaternion[0]=tmp[0]  #x
    quaternion[1]=tmp[1]  #y
    quaternion[2]=tmp[2]  #z


    quaternion[3]=np.dot(v1,v2)+ np.linalg.norm(v1)*np.linalg.norm(v2)  #w

    quaternion_normalize=quaternion/np.linalg.norm(quaternion)

    x=quaternion_normalize[0]
    y=quaternion_normalize[1]
    z=quaternion_normalize[2]
    w=quaternion_normalize[3]

    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(2.0 * (-z * x + w * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    thrust_factor=0.058
    thrust=np.linalg.norm(v2)*thrust_factor

    euler_and_thrust=np.array([roll,pitch,yaw,thrust])

    return euler_and_thrust


def nnCallback(msg):

    time_start = time.time()
    input_arr = np.array(msg.input, dtype=float)  #input被归一化了

    a_des1=np.array([msg.a_x,msg.a_y,msg.a_z])  #a_x a_y a_z没有被归一化


    x_data=torch.from_numpy(input_arr).float()

    prediction1 = Net(x_data)

    prediction1=prediction1*max_min_vector[0:6]+min_vector[0:6] #输出反归一化

    a_des2=a_des1-np.array([0.1,0.1,0.1])*np.array(prediction1[0:3].detach())


    euler_and_thrust=accelerate2euler(a_des2)
    # euler_and_thrust=(euler_and_thrust-min_vector[-4:])/max_min_vector[-4:] #输入归一化

    # x_data[-4:]=torch.from_numpy(euler_and_thrust).float()
    # prediction2 = Net(x_data)
    # prediction2=prediction2*max_min_vector[0:6]+min_vector[0:6] #输出反归一化


    for i in range(1):

        euler_and_thrust = (euler_and_thrust - np.array(min_vector[-4:].detach()) )/ np.array(max_min_vector[-4:].detach())   # 输入归一化


        x_data[-4:] = torch.from_numpy(euler_and_thrust).float()

        prediction2 = Net(x_data)
        prediction2 = prediction2 * max_min_vector[0:6] + min_vector[0:6]  # 输出反归一化

        a_des3 = a_des2 - np.array(prediction2[0:3].detach()) * (a_des2 - a_des1) \
                        / ( np.array(prediction2[0:3].detach())   -np.array(prediction1[0:3].detach()) )

        a_des1=copy.deepcopy(a_des2)
        prediction1=prediction2.clone()

        a_des2=copy.deepcopy(a_des3)

        euler_and_thrust = accelerate2euler(a_des3)



    update_msg=input()
    update_msg.input[0]=euler_and_thrust[0]
    update_msg.input[1]=euler_and_thrust[1]
    update_msg.input[2]=euler_and_thrust[2]
    update_msg.input[3]=euler_and_thrust[3]


    tracker_up_pub.publish(update_msg)
    time_end = time.time()

    print ('spend time: ',(time_end - time_start))


rospy.init_node('tracker_nn', anonymous=True)
device = torch.device("cpu")
map_location = torch.device('cpu')
Net = torch.nn.Sequential(
    torch.nn.Linear(40, 128),
    torch.nn.Tanh(),
    torch.nn.Linear(128, 256),
    torch.nn.Tanh(),
    torch.nn.Linear(256, 256),
    torch.nn.Tanh(),
    torch.nn.Linear(256, 128),
    torch.nn.Tanh(),
    torch.nn.Linear(128,6),
    torch.nn.Tanh(),
)
Net.load_state_dict(torch.load("/home/pengpeng/catkin_ws/random_fly_new_new/net_4999.pkl", map_location='cpu'))

min_vector = torch.tensor([-0.924959, -0.911663, -0.593314, -3.45101, -2.77456, -2.49199,
                           -2.1737, -1.21307, -1.11182, 0.164217])

max_min_vector = torch.tensor([1.7747, 1.672445, 4.895684, 7.26044, 5.781, 6.11199,
                               3.79378, 2.71257, 3.22697, 1.729003])

rospy.Subscriber("/nn_compute", input, nnCallback)
tracker_up_pub = rospy.Publisher('/tracker_update', input, queue_size=1)
rospy.spin()
