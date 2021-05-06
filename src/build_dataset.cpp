//
// Created by pengpeng on 1/24/21.
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
    std::ofstream outFile;

    std::string data_file="/home/pengpeng/catkin_ws/real_drone1/dataset_fake_random_fly_record_03_14_20_18_55_for_test.csv";  //生成的数据集

    int LAST=3;

/*    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_11_22_30_05.csv");  //用来构建数据集的原始数据
    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_11_22_39_20.csv");
    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_12_11_46_49.csv");
    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_12_11_51_06.csv");
    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_12_11_59_11.csv");
    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_12_12_15_05.csv");
    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_12_12_30_33.csv");
    file.push_back("/home/pengpeng/catkin_ws/real_drone1/random_fly_record_03_12_12_37_47.csv");*/


    file.push_back("/home/pengpeng/catkin_ws/real_drone1/fake_random_fly_record_03_14_20_18_55.csv"); //用来构建数据集的原始数据






    for(int i=0;i<file.size();i++)
    {
        ifstream fin(file[i]);
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

    //把pitch反过来
    for(int i=0;i<data.size();i++)
    {
        data[i][32]=data[i][32];
    }




/* //归一化 将数据映射到到 0 到 1 之间
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
        cout<<"列序号："<<j<<" max: "<<max<<" min:"<<min<<" max-min+0.01:"<<max-min+0.01<<endl;
        for(int i=0;i<data.size();i++)
        {
            data[i][j]=(data[i][j]-min)/(max-min+0.01);
        }
    }
*/


    outFile.open(data_file, std::ios::out);
    cout<<"build dataset to "<<data_file<<endl;
    outFile.trunc;


    for (int time=6;time<data.size()-1;time++)
    {
        tiny_dnn::vec_t train_data_vector;
        tiny_dnn::vec_t train_target_vector;

        for(int last=LAST;last>=0;last--)
        {
            for(int i=18;i<=23;i++)
            {
                train_data_vector.push_back(data[time-last][i]); //位置　速度　加速度差值
            }
            for(int i=31;i<=34;i++)
            {
                train_data_vector.push_back(data[time-last][i]); //给定的姿态 和 推力
            }
        }
        for(int i=18;i<=23;i++)
        {
            train_target_vector.push_back(data[time+1][i]); //下一时刻 的 位置 速度 加速度与规划值的差值
        }
        for(int i=0;i<train_data_vector.size();i++)
        {
            outFile<<train_data_vector[i]<<",";
        }
        train_data.push_back(train_data_vector);

        for(int i=0;i<train_target_vector.size()-1;i++)
        {
            outFile<<train_target_vector[i]<<",";
        }
        outFile<<train_target_vector[train_target_vector.size()-1]<<"\n";
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
    int train=0;

    construct_net(nn, backend_type);
    //nn.load("/home/pengpeng/catkin_ws/src/pva_tracker/cmake-build-debug/devel/lib/pva_tracker/LeNet_eluer_-model_random_fly_1");

    std::vector<tiny_dnn::vec_t> train_target, test_target;
    std::vector<tiny_dnn::vec_t> train_data, test_data;

    read_data_from_csv(train_data,test_data,train_target,test_target);

    cout<<"train_data.size():"<<train_data.size()<<endl;
    cout<<"train_data_dimension: "<<train_data[0].size()<<endl;
    cout<<"train_target.size():"<<train_target.size()<<endl;
    cout<<"train_target_dimension: "<<train_target[0].size()<<endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "build_dataset");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);
    ROS_INFO("train");

    double learning_rate                   = 1;
    double epochs                             = 1;
    std::string data_path                  = "";
    int minibatch_size                     = 1;
    tiny_dnn::core::backend_t backend_type = tiny_dnn::core::default_engine();
    train_lenet(data_path, learning_rate, epochs, minibatch_size, backend_type);

    while(ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();

    }
}
