//
// Created by pengpeng on 1/15/21.
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

string Trim(string& str)
{
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
    str.erase(0,str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}

void read_data_from_csv(std::vector<tiny_dnn::vec_t> &train_data,std::vector<tiny_dnn::vec_t> &test_data,
                        std::vector<tiny_dnn::vec_t> &train_target, std::vector<tiny_dnn::vec_t> &test_target)
{

    std::vector<std::vector<float>>data;


    ifstream fin("/home/pengpeng/catkin_ws/src/pva_tracker/mpc_record_test.csv"); //打开文件流操作
    string line;
    while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
    {
        //cout <<"原始字符串："<< line << endl; //整行输出
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
//        vector<string> fields; //声明一个字符串向量
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
//        string name = Trim(fields[0]); //清除掉向量fields中第一个元素的无效字符，并赋值给变量name
//        string age = Trim(fields[1]); //清除掉向量fields中第二个元素的无效字符，并赋值给变量age
//        string birthday = Trim(fields[2]); //清除掉向量fields中第三个元素的无效字符，并赋值给变量birthday
//        cout <<"处理之后的字符串："<< name << "\t" << age << "\t" << birthday << endl;
    }
    //cout<<"data[0][0]:"<<data[0][0]<<endl;

//    train_data={{1,2},{3,4}};
//    test_labels={10,200};


    //归一化 将数据映射到到 0 到 1 之间
    for(int j=0;j<data[0].size();j++)
    {
        float min=1000;
        float max=-1000;
        for(int i=0;i<data.size();i++)
        {
            max=std::max(max,data[i][j]);
            min=std::min(min,data[i][j]);
        }
        cout<<"列序号："<<j<<"max: "<<max<<" min:"<<min<<endl;
        for(int i=0;i<data.size();i++)
        {
            data[i][j]=(data[i][j]-min)/(max-min+0.01);
        }

    }




    cout<<"data.size():"<<data.size()<<endl;
    cout<<"data[0].size():"<<data[0].size()<<endl;

    for (int time=1;time<data.size()-1;time++)
    {
        tiny_dnn::vec_t train_data_vector;
        tiny_dnn::vec_t train_target_vector;
        for(int i=0;i<data[0].size();i++)
        {
            if(i>=27&&i<=30)
            {
                train_data_vector.push_back(data[time-1][i]);//上一时刻的给定姿态四元数
            }
            if(i==34)
            {
                train_data_vector.push_back(data[time-1][i]);//上一时刻的给定推力
            }
        }
        for(int i=0;i<data[0].size();i++)
        {
            if(i>=18&&i<=30)
            {
                train_data_vector.push_back(data[time][i]);//这一时刻的位置 速度 加速度与规划值的差值 姿态四元数
            }
            if(i==34)
            {
                train_data_vector.push_back(data[time][i]);//这一时刻的给定推力
            }

            if(i>=18&&i<=26)
            {
                train_target_vector.push_back(data[time+1][i]); //下一时刻 的 位置 速度 加速度与规划值的差值
            }

        }
        train_data.push_back(train_data_vector);
        train_target.push_back(train_target_vector);

    }

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

    nn << fc(19, 64, true,backend_type)
       << sigmoid()
       << fc(64, 256, true,backend_type)
       << sigmoid()
       << fc(256, 256, true,backend_type)
       << sigmoid()
       << fc(256, 9, true,backend_type)
       << sigmoid();


}



static void train_lenet(const std::string &data_dir_path,
                        double learning_rate,
                        int n_train_epochs,
                        int n_minibatch,
                        tiny_dnn::core::backend_t backend_type)
{

    tiny_dnn::network<tiny_dnn::sequential> nn;
    tiny_dnn::adamax optimizer;
    //construct_net(nn, backend_type);
    nn.load("/home/pengpeng/catkin_ws/src/pva_tracker/cmake-build-debug/devel/lib/pva_tracker/LeNet-model");
    std::vector<tiny_dnn::vec_t> train_target, test_target;
    std::vector<tiny_dnn::vec_t> train_data, test_data;

    read_data_from_csv(train_data,test_data,train_target,test_target);

//    train_data={{1,2},{3,4}};
//    train_target={{1,1},{1,1}};

    cout<<"train_data.size():"<<train_data.size()<<endl;
    cout<<"train_data_dimension: "<<train_data[0].size()<<endl;
    cout<<"train_target.size():"<<train_target.size()<<endl;
    cout<<"train_target_dimension: "<<train_target[0].size()<<endl;

    for (int i = 0; i < nn.depth(); i++) {
        cout << "#layer:" << i << "\n";
        cout << "layer type:" << nn[i]->layer_type() << "\n";
        cout << "input:" << nn[i]->in_size() << "(" << nn[i]->in_shape() << ")\n";
        cout << "output:" << nn[i]->out_size() << "(" << nn[i]->out_shape() << ")\n";
    }

    tiny_dnn::progress_display disp(train_data.size());
    tiny_dnn::timer t;



    // this lambda function will be called after each epoch
    int iEpoch              = 0;
    auto on_enumerate_epoch = [&]() {
        // compute loss and disp 1/100 of the time
        iEpoch++;
        if (iEpoch % 100) return;

        double loss = nn.get_loss<tiny_dnn::mse>(train_data, train_target);
        ROS_WARN("====================================");
        std::cout << "epoch=" << iEpoch << "/" << n_train_epochs << " loss=" << loss
                  << std::endl;

        tiny_dnn::vec_t predict=nn.predict(train_data[100]);
        cout<<"predict of train_data[100]: "<<endl;
        for(int i=0;i<predict.size();i++)
        {
            cout<<predict[i]<<" ";
        }
        cout<<endl;

        cout<<"truth of target_data[100]: "<<endl;
        for(int i=0;i<train_target[100].size();i++)
        {
            cout<<train_target[100][i]<<" ";
        }
        cout<<endl;

    };

    // learn
    cout<<"--------Training start----------"<<endl;

    n_minibatch=train_data.size();
    n_train_epochs=5000;

    nn.fit<tiny_dnn::mse>(optimizer, train_data, train_target, n_minibatch, n_train_epochs, []() {},
                          on_enumerate_epoch);

    std::cout << std::endl
              << "Training finished, now computing prediction results:"
              << std::endl;

    nn.save("LeNet-model2");

/*    tiny_dnn::vec_t predict=nn.predict(train_data[100]);
    cout<<"predict of train_data[100]: "<<endl;
    for(int i=0;i<predict.size();i++)
    {
        cout<<predict[i]<<" ";
    }
    cout<<endl;

    cout<<"truth of target_data[100]: "<<endl;
    for(int i=0;i<train_target[100].size();i++)
    {
        cout<<train_target[100][i]<<" ";
    }
    cout<<endl;*/



    // nn.test(test_data, test_labels).print_detail(std::cout);
    // save network model & trained weights



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
