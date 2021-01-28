//
// Created by pengpeng on 1/11/21.
//


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdint>

#include "tiny_dnn/tiny_dnn.h"


#define PI 3.1415926
#define number 10
using namespace Eigen;
using namespace std;

std::vector<float> min_vector;
std::vector<float> max_min_vector;

string Trim(string& str)
{
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
    str.erase(0,str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}

static void construct_net(tiny_dnn::network<tiny_dnn::sequential> &nn,
                          tiny_dnn::core::backend_t backend_type)
{
    using fc = tiny_dnn::layers::fc;
    using conv = tiny_dnn::layers::conv;
    using ave_pool = tiny_dnn::layers::ave_pool;
    using tanh = tiny_dnn::activation::tanh;
    using sigmoid = tiny_dnn::sigmoid_layer;

    using tiny_dnn::core::connection_table;
    using padding = tiny_dnn::padding;

    nn << fc(78, 128, true,backend_type)
       << tanh()
       << fc(128, 256, true,backend_type)
       << tanh()
       << fc(256, 256, true,backend_type)
       << tanh()
       << fc(256, 9, true,backend_type)
        ;
}



void read_data_from_csv(std::vector<tiny_dnn::vec_t> &train_data,std::vector<tiny_dnn::vec_t> &test_data,
                        std::vector<tiny_dnn::vec_t> &train_target, std::vector<tiny_dnn::vec_t> &test_target)
{

    std::vector<std::vector<float>>data;

    std::vector<std::string> file;

    file.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_1.csv");
    file.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_2.csv");
    file.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_3.csv");
    file.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_4.csv");
    file.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_5.csv");

    //string file[1]={"/home/pengpeng/catkin_ws/mpc_record_v=4.csv"};
    for (int i=0;i<file.size();i++)
    {
        //ifstream fin("/home/pengpeng/catkin_ws/mpc_record_v=4.csv"); //打开文件流操作
        ifstream fin(file[i]); //打开文件流操作
        string line;
        while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
        {
            istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
            string field;
            std::vector<float>data_vector;
            while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
            {
                float x;
                stringstream sx, sy, sz;
                sx << field;
                sx >> x;
                data_vector.push_back(x);
            }
            data.push_back(data_vector);
        }
    }


    cout<<data[0].size()<<endl;
    for(int j=0;j<data[0].size();j++)
    {
        float min=1000;
        float max=-1000;
        for(int i=0;i<data.size();i++)
        {
            max=std::max(max,data[i][j]);
            min=std::min(min,data[i][j]);
        }
        max_min_vector.push_back(max-min+0.01);
        min_vector.push_back(min);
        cout<<"列序号："<<j<<" max: "<<max<<" min:"<<min<<" max-min+0.01:"<<max-min+0.01<<endl;
    }

    vector<std::vector<float>>().swap(data);


    std::vector<std::string> file_train;

    file_train.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_1.csv");
    file_train.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_2.csv");
    file_train.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_3.csv");
    file_train.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_4.csv");
    file_train.push_back("/home/pengpeng/catkin_ws/mpc_record_random_fly_5.csv");


    cout<<"file_train.size: "<<file_train.size()<<endl;

    for (int i=0;i<file_train.size();i++)
    {
        //ifstream fin("/home/pengpeng/catkin_ws/mpc_record_v=4.csv"); //打开文件流操作
        ifstream fin(file_train[i]); //打开文件流操作
        string line;
        while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
        {
            istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
            string field;
            std::vector<float>data_vector;
            while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
            {
                float x;
                stringstream sx, sy, sz;
                sx << field;
                sx >> x;
                data_vector.push_back(x);
            }
            data.push_back(data_vector);
        }
    }
    //归一化 将数据映射到 0 到 1 之间
    for(int j=0;j<data[0].size();j++)
    {
        for(int i=0;i<data.size();i++)
        {
            data[i][j]=(data[i][j]-min_vector[j])/(max_min_vector[j]);
        }
    }
    cout<<"data.size():"<<data.size()<<endl;
    cout<<"data[0].size():"<<data[0].size()<<endl;

    for (int time=6;time<data.size()-1;time++)
    {
        tiny_dnn::vec_t train_data_vector;
        tiny_dnn::vec_t train_target_vector;

        for(int last=5;last>=0;last--)
        {
            for(int i=18;i<=26;i++)
            {
                train_data_vector.push_back(data[time-last][i]); //位置　速度　加速度差值
            }
            for(int i=31;i<=34;i++)
            {
                train_data_vector.push_back(data[time-last][i]); //给定的姿态 和 推力
            }
        }
        for(int i=18;i<=26;i++)
        {
            train_target_vector.push_back(data[time+1][i]); //下一时刻 的 位置 速度 加速度与规划值的差值
        }
        train_data.push_back(train_data_vector);
        train_target.push_back(train_target_vector);
    }
}


static void train_lenet(const std::string &data_dir_path,
                        double learning_rate,
                        int n_train_epochs,
                        int n_minibatch,
                        tiny_dnn::core::backend_t backend_type)
{

    tiny_dnn::network<tiny_dnn::sequential> nn;
    tiny_dnn::adamax optimizer;
    int train=1;

    //construct_net(nn, backend_type);
    nn.load("/home/pengpeng/catkin_ws/src/pva_tracker/cmake-build-debug/devel/lib/pva_tracker/LeNet_eluer_-model_random_fly_1");

    std::vector<tiny_dnn::vec_t> train_target, test_target;
    std::vector<tiny_dnn::vec_t> train_data, test_data;

    read_data_from_csv(train_data,test_data,train_target,test_target);

//    train_data={{1,2},{3,4}};
//    train_target={{1,1},{1,1}};

    cout<<"train_data.size():"<<train_data.size()<<endl;
    cout<<"train_data_dimension: "<<train_data[0].size()<<endl;
    cout<<"train_target.size():"<<train_target.size()<<endl;
    cout<<"train_target_dimension: "<<train_target[0].size()<<endl;



    if(train==1)
    {
        tiny_dnn::progress_display disp(train_data.size());
        tiny_dnn::timer t;


        // this lambda function will be called after each epoch
        int iEpoch              = 0;
        auto on_enumerate_epoch = [&]() {
            // compute loss and disp 1/100 of the time
            iEpoch++;
            if (iEpoch % 100) return;

            double loss = nn.get_loss<tiny_dnn::mse_pp>(train_data, train_target);
            ROS_WARN("====================================");
            std::cout << "epoch=" << iEpoch << "/" << n_train_epochs << " loss=" << loss
                      << std::endl;

            int aa = rand() % (train_data.size()-10) + 1;
            cout<<"aa: "<<aa<<endl;
            tiny_dnn::vec_t predict=nn.predict(train_data[aa]);
            cout<<"predict of train_data[aa]: "<<endl;
            for(int i=0;i<predict.size();i++)
            {
                cout<<predict[i]*max_min_vector[i+18]+min_vector[i+18]<<" ";
            }
            cout<<endl;

            cout<<"truth of target_data[aa]: "<<endl;
            for(int i=0;i<train_target[aa].size();i++)
            {
                cout<<train_target[aa][i]*max_min_vector[i+18]+min_vector[i+18]<<" ";
            }
            cout<<endl;

            if (iEpoch % 1000) return;
            nn.save("LeNet_eluer_-model_random_fly_no_tanh");

        };

        // learn
        cout<<"--------Training start----------"<<endl;

        n_minibatch=512;
        n_train_epochs=5000;

        nn.fit<tiny_dnn::mse_pp>(optimizer, train_data, train_target, n_minibatch, n_train_epochs, []() {},
                                 on_enumerate_epoch);

        std::cout << std::endl
                  << "Training finished, now computing prediction results:"
                  << std::endl;

        nn.save("LeNet_eluer_-model_random_fly_no_tanh");
    }

    else
    {
        srand(time(NULL));
        float sum_x=0;
        float sum_y=0;
        float sum_z=0;
        float kk=0;
        float jj=0;
        int count=0;
        int test_number=500;
        for(int j=0;j<test_number;j++)
        {

            int aa = rand() % (train_data.size()-10) + 1;
            cout<<"aa: "<<aa<<endl;
            tiny_dnn::vec_t predict=nn.predict(train_data[aa]);
            cout<<"predict of train_data[aa]: "<<endl;
            for(int i=0;i<predict.size();i++)
            {
                cout<<predict[i]*max_min_vector[i+18]+min_vector[i+18]<<" ";
                //cout<<predict[i]<<" ";

            }
            cout<<endl;

            cout<<"truth of target_data[aa]: "<<endl;
            for(int i=0;i<train_target[aa].size();i++)
            {
                cout<<train_target[aa][i]*max_min_vector[i+18]+min_vector[i+18]<<" ";
                //cout<<train_target[aa][i]<<" ";
            }
            cout<<endl;

            float kk=fabs(((train_target[aa][0]*max_min_vector[0+18]+min_vector[0+18])-(predict[0]*max_min_vector[0+18]+min_vector[0+18]))
                    /(train_target[aa][0]*max_min_vector[0+18]+min_vector[0+18]));
            cout<<"x kk: "<<kk<<endl;
            if(kk>1)
            {
                count++;
            } else{
                sum_x+=kk;
            }


            kk=fabs(((train_target[aa][1]*max_min_vector[1+18]+min_vector[1+18])-(predict[1]*max_min_vector[1+18]+min_vector[1+18]))
                    /(train_target[aa][1]*max_min_vector[1+18]+min_vector[1+18]));
            cout<<"y kk: "<<kk<<endl;
            sum_y+=kk;

            kk=fabs(((train_target[aa][2]*max_min_vector[2+18]+min_vector[2+18])-(predict[2]*max_min_vector[2+18]+min_vector[2+18]))
                    /(train_target[aa][2]*max_min_vector[2+18]+min_vector[2+18]));
            cout<<"z kk: "<<kk<<endl;
            sum_z+=kk;
            ROS_WARN("----------");
        }
        cout<<"count :"<<count<<endl;
        cout<<"average error"<<sum_x/(test_number-count)<<endl;
        cout<<"average error"<<sum_y/test_number<<endl;
        cout<<"average error"<<sum_z/test_number<<endl;

        cout<<"train_data.size():"<<train_data.size()<<endl;
        cout<<"train_data_dimension: "<<train_data[0].size()<<endl;
        cout<<"train_target.size():"<<train_target.size()<<endl;
        cout<<"train_target_dimension: "<<train_target[0].size()<<endl;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiny_dnn_test2");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);
    ROS_INFO("train");

    double learning_rate                   = 1;
    double epochs                             = 1;
    std::string data_path                  = "";
    int minibatch_size                     = 1;
    tiny_dnn::core::backend_t backend_type = tiny_dnn::core::default_engine();


    train_lenet(data_path, learning_rate, epochs, minibatch_size, backend_type);


    // wait for FCU connection
    while(ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();

    }
}
