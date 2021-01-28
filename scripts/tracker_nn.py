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

import time
from pva_tracker.msg import input


def nnCallback(msg):

    time_start = time.time()
    input_arr = np.array(msg.input, dtype=float)

    x_data=torch.from_numpy(input_arr).float()

    #prediction = Net(x_data)

    delta_t=0.01

    for j in range(5):
        #print("python compute")
        new_ten1=x_data.clone()
        new_ten1[36]=new_ten1[36]+delta_t    #36 37 38 39

        new_ten2=x_data.clone()
        new_ten2[36]=new_ten2[36]-delta_t   #36 37 38 39

        new_ten3=x_data.clone()
        new_ten3[37]=new_ten3[37]+delta_t  #36 37 38 39

        new_ten4=x_data.clone()
        new_ten4[37]=new_ten4[37]-delta_t   #36 37 38 39

        new_ten5=x_data.clone()
        new_ten5[38]=new_ten5[38]+delta_t   #36 37 38 39

        new_ten6=x_data.clone()
        new_ten6[38]=new_ten6[38]-delta_t   #36 37 38 39

        new_ten7=x_data.clone()
        new_ten7[39]=new_ten7[39]+delta_t   #36 37 38 39

        new_ten8=x_data.clone()
        new_ten8[39]=new_ten8[39]-delta_t  #36 37 38 39

        new_ten=torch.stack((x_data,new_ten1,new_ten2,new_ten3,new_ten4,new_ten5,new_ten6,new_ten7,new_ten8),0)

        new_prediction = Net(new_ten)

        new_prediction_square = new_prediction * new_prediction

        for i in range(4):
            first_deri = (torch.sum(new_prediction_square[2 * i + 1, 0:2]) - torch.sum(new_prediction_square[0,0 :2])) / delta_t

            second_deri = (torch.sum(new_prediction_square[2 * i + 1,0 :2]) - 2 * torch.sum(new_prediction_square[2 * i, 0:2])
                           + torch.sum(new_prediction_square[2 * i - 1, 0:2])) / delta_t / delta_t

            x_data[36 + i] = x_data[ 36 + i] - 1.5 * first_deri / second_deri


    update_msg=input()
    # update_msg.input[0]=prediction[0].item()
    # update_msg.input[1]=prediction[1].item()
    # update_msg.input[2]=prediction[2].item()
    # update_msg.input[3]=prediction[3].item()
    # update_msg.input[4]=prediction[4].item()
    # update_msg.input[5]=prediction[5].item()

    update_msg.input[0]=x_data[36]
    update_msg.input[1]=x_data[37]
    update_msg.input[2]=x_data[38]
    update_msg.input[3]=x_data[39]


    tracker_up_pub.publish(update_msg)
    time_end = time.time()

    #print ('spend time: ',(time_end - time_start))


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


rospy.Subscriber("/nn_compute", input, nnCallback)
tracker_up_pub = rospy.Publisher('/tracker_update', input, queue_size=1)
rospy.spin()
