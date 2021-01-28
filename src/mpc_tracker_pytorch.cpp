

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <pva_tracker/PVA_TrackerConfig.h>
#include <nav_msgs/Odometry.h>
#include "pva_tracker/input.h"

#include "torch/script.h"
#include "torch/torch.h"

#define GRAVITATIONAL_ACC 9.81

using namespace Eigen;

// Coefficients
Vector3d position_error_p;
Vector3d position_error_d;
Vector3d position_error_i;
Vector3d velocity_error_p;
Vector3d velocity_error_d;
Vector3d velocity_error_i;

double p_i_acc_error_limit;
double v_i_acc_error_limit;

// Global Variables
Vector3d planned_p;
Vector3d planned_v;
Vector3d planned_a;
double planned_yaw;
Vector3d current_p;
Vector3d current_v;
Vector3d current_a;
Vector3d mpc_k;
Quaterniond current_att;
ros::Publisher att_ctrl_pub, odom_sp_enu_pub;
double thrust_factor;
std::vector<std::vector<float>> input_queue;

ros::Time last_time;
Vector3d last_a;
Vector3d last_current_v;

ros::Publisher pre_pub;
ros::Publisher nn_pub;

pva_tracker::input  predict_msg;

pva_tracker::input  input_msg;

float mpc_kpx,mpc_kpy,mpc_kvx,mpc_kvy, mpc_kaxy,mpc_kpz, mpc_kvz, mpc_kaz;

torch::jit::script::Module module;

using namespace std;

float max_min[]={1.7747, 1.6724, 4.8957, 7.2604, 5.7810, 6.1120,3.79378,  2.71257 , 3.22697,  1.729003};
float min_arr[]={-0.924959 ,-0.911663 ,-0.593314 ,-3.45101  ,-2.77456 , -2.49199 , -2.1737, -1.21307 , -1.11182 ,  0.164217};

Vector3d a_des;
Quaterniond att_des_q;
double thrust_des;

mavros_msgs::AttitudeTarget att_setpoint;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    /// ENU frame
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    current_att.w() = msg->pose.orientation.w;
    current_att.x() = msg->pose.orientation.x;
    current_att.y() = msg->pose.orientation.y;
    current_att.z() = msg->pose.orientation.z;
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    /// ENU frame
    current_v << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;

    current_a=(current_v-last_current_v)/(ros::Time::now().toSec()-last_time.toSec());


    if(current_a(0)>100||current_a(0)<-100)
    {
        ROS_INFO("Accelerate TOO BIG");
        current_a=last_a;
    }
//    ROS_INFO("QQQQQ  a0  :%f   a1:%f    a2:%f ",current_a(0),current_a(1),current_a(2));
    last_time=ros::Time::now();
    last_current_v=current_v;
    last_a=current_a;
}

Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw)
{
    Eigen::Quaterniond temp;
    temp.x() = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y() = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z() = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    temp.w() = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);

    return temp;
}

Eigen::Vector3d quaternion2euler_eigen(float x, float y, float z, float w)
{
    Eigen::Vector3d temp;//roll pitch yaw
    temp.x() = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp.y() = asin(2.0 * (z * x - w * y));
    temp.z() = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}


Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2)
{
    Vector3d result;
    result << v1(0)*v2(0), v1(1)*v2(1), v1(2)*v2(2);
    return result;
}

void vector3dLimit(Vector3d &v, double limit)  ///limit should be positive
{
    if(limit > 0){
        for(int i=0; i<3; i++){
            v(i) = fabs(v(i)) > limit ? (v(i) > 0 ? limit : -limit) : v(i);
        }
    }
}
template <typename T>
void store_data3d(T &store_data, Vector3d vector3d)
{
    for (int i=0;i<3;i++)
    {
        store_data.push_back(vector3d[i]);
    }
}

void accel2quater(Vector3d a_des,Quaterniond &att_des_q, double &thrust_des)
{
    Vector3d att_des_norm = a_des / a_des.norm();

    Quaterniond z_w_quat(0, 0, 0, 1.0);
    Quaterniond att_current_vector_quat = current_att * z_w_quat * current_att.inverse();
    Vector3d current_att_vector(att_current_vector_quat.x(), att_current_vector_quat.y(), att_current_vector_quat.z());
    Vector3d z_w_norm(0, 0, 1.0);
//    Quaterniond att_des_q = Quaterniond::FromTwoVectors(current_att_vector, att_des_norm);
    att_des_q = Quaterniond::FromTwoVectors(z_w_norm, att_des_norm);

    //add yaw
    Quaterniond yaw_quat(cos(planned_yaw/2.0), att_des_norm(0)*sin(planned_yaw/2.0),
                         att_des_norm(1)*sin(planned_yaw/2.0),att_des_norm(2)*sin(planned_yaw/2.0));
    att_des_q = yaw_quat * att_des_q;

    //Calculate thrust
    thrust_des = a_des.norm() * thrust_factor;  //a_des.dot(att_current_vector) * THRUST_FACTOR

}




void pvaCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{



    planned_p << msg->positions[0], msg->positions[1], msg->positions[2];
    planned_yaw = msg->positions[3];
    planned_v << msg->velocities[0], msg->velocities[1], msg->velocities[2];
    planned_a << msg->accelerations[0], msg->accelerations[1], msg->accelerations[2];


    /// Publish to record in rosbag
    nav_msgs::Odometry odom_sp_enu;
    odom_sp_enu.header.stamp = ros::Time::now();
    odom_sp_enu.pose.pose.position.x = planned_p(0);
    odom_sp_enu.pose.pose.position.y = planned_p(1);
    odom_sp_enu.pose.pose.position.z = planned_p(2);
    odom_sp_enu.twist.twist.linear.x = planned_v(0);
    odom_sp_enu.twist.twist.linear.y = planned_v(1);
    odom_sp_enu.twist.twist.linear.z = planned_v(2);

    odom_sp_enu.pose.pose.orientation.x=current_p(0)-planned_p(0);
    odom_sp_enu.pose.pose.orientation.y=current_p(1)-planned_p(1);
    odom_sp_enu.pose.pose.orientation.z=current_p(2)-planned_p(2);

    odom_sp_enu_pub.publish(odom_sp_enu);

    /// Calculate desired thrust and attitude
    Vector3d p_error = planned_p - current_p;
    Vector3d v_error = planned_v - current_v;

    static Vector3d p_error_last;
    static Vector3d v_error_last;
    static Vector3d p_error_accumulate;
    static Vector3d v_error_accumulate;
    static bool if_init = true;

    if(if_init){
        if_init = false;
        p_error_last = p_error;
        v_error_last = v_error;
        p_error_accumulate = p_error;
        v_error_accumulate = v_error;
        return;
    }


    /**Core code**/
    Vector3d delt_p_error = p_error - p_error_last;
    Vector3d delt_v_error = v_error - v_error_last;

    p_error_accumulate += p_error;
    v_error_accumulate += v_error;
    vector3dLimit(p_error_accumulate, p_i_acc_error_limit);
    vector3dLimit(v_error_accumulate, v_i_acc_error_limit);

    Vector3d a_fb =   /// PID
            vectorElementMultiply(p_error, position_error_p) + vectorElementMultiply(v_error, velocity_error_p) +
            vectorElementMultiply(delt_p_error, position_error_d) + vectorElementMultiply(delt_v_error, velocity_error_d) +
            vectorElementMultiply(p_error_accumulate, position_error_i) + vectorElementMultiply(v_error_accumulate, velocity_error_i);

    p_error_last = p_error;
    v_error_last = v_error;



    // Set a maximum acceleration feed forward value given by position and velocity error.
    for(int i=0; i<3; i++){
        if(fabs(a_fb(i)) > 5.0) a_fb(i) = 5.0 * a_fb(i) / fabs(a_fb(i));
    }
    Vector3d z_w_norm(0, 0, 1.0);
    a_des = a_fb + planned_a + GRAVITATIONAL_ACC * z_w_norm;
    accel2quater(a_des,att_des_q,thrust_des);


    //ROS_INFO("first:  %f  %f  %f",a_des(0),a_des(1),a_des(2));
    Vector3d euler_angle=quaternion2euler_eigen(att_des_q.x(),att_des_q.y(),att_des_q.z(),att_des_q.w());

    std::vector<float> nn_input;

    if(input_queue.size()>6)
    {
        for(int last=3;last>=1;last--)
        {
            for(int i=9;i<=14;i++)
            {
                nn_input.push_back(input_queue[input_queue.size()-last][i]-input_queue[input_queue.size()-last-1][i-9]);
            }
            for(int i=18;i<=21;i++)
            {
                nn_input.push_back(input_queue[input_queue.size()-last][i]);
            }
        }

        std::vector<float> current_pva;
        store_data3d(current_pva,current_p);
        store_data3d(current_pva,current_v);
        store_data3d(current_pva,current_a);
        for(int i=0;i<=5;i++)
        {
            nn_input.push_back(current_pva[i]-input_queue.back()[i]);
        }
        nn_input.push_back(euler_angle(0));
        nn_input.push_back(euler_angle(1));
        nn_input.push_back(euler_angle(2));
        nn_input.push_back(thrust_des);

        ROS_INFO("original roll: %f pitch:  %f yaw: %f  thrust_des: %f",nn_input[36],nn_input[37]
        ,nn_input[38],nn_input[39]);

        //输入归一化
        for(int i=0;i<nn_input.size();i++)
        {
            int temp =i%10;
            nn_input[i]=(nn_input[i]-min_arr[temp])/max_min[temp]; //归一化
        }

        for(int i=0;i<nn_input.size();i++)
        {
            input_msg.input[i]=nn_input[i];
        }



        nn_pub.publish(input_msg);
/*        float input_arr[nn_input.size()];
        for(int i=0;i<nn_input.size();i++)
        {
            input_arr[i]=nn_input[i];
        }
        std::vector<torch::jit::IValue> inputs;
        ros::Time last_request=ros::Time::now();

        inputs.emplace_back(torch::from_blob(input_arr,{1,40}));


        at::Tensor outputs = module.forward(inputs).toTensor();
        //ROS_INFO("ann spend: %f",ros::Time::now().toSec()-last_request.toSec());

        std::vector<float> predict(outputs.data_ptr<float>(), outputs.data_ptr<float>()+outputs.numel());

        //输出反归一化
        for(int i=0;i<predict.size();i++)
        {
            predict[i]=predict[i]*max_min[i]+min[i];
        }

        ROS_INFO_THROTTLE(2,"predict: %f  %f   %f  %f  %f  %f    ",predict[0],predict[1],predict[2],predict[3],predict[4]
        ,predict[5]);


        predict_msg.pred_x=predict[0];
        predict_msg.pred_y=predict[1];
        predict_msg.pred_z=predict[2];
        predict_msg.pred_vx=predict[3];
        predict_msg.pred_vy=predict[4];
        predict_msg.pred_vz=predict[5];
        pre_pub.publish(predict_msg);

        a_des(0)=a_des(0)-(mpc_kpxy*predict[0]+mpc_kvxy*predict[3]+mpc_kaxy*predict[6]);
        a_des(1)=a_des(1)-(mpc_kpxy*predict[1]+mpc_kvxy*predict[4]+mpc_kaxy*predict[7]);
        a_des(2)=a_des(2)-(mpc_kpz*predict[2]+mpc_kvz*predict[5]+mpc_kaz*predict[8]);
        accel2quater(a_des,att_des_q,thrust_des);

        //ROS_INFO("second:  %f  %f  %f",a_des(0),a_des(1),a_des(2));*/

    }
    else
    {
        att_setpoint.header.stamp = ros::Time::now();
        att_setpoint.orientation.w = att_des_q.w();
        att_setpoint.orientation.x = att_des_q.x();
        att_setpoint.orientation.y = att_des_q.y();
        att_setpoint.orientation.z = att_des_q.z();
        att_setpoint.thrust = thrust_des;

        std::vector<float> store_data;
        store_data3d(store_data,planned_p);  // 0 1 2
        store_data3d(store_data,planned_v); // 3 4 5
        store_data3d(store_data,planned_a);  // 6 7 8
        store_data3d(store_data,current_p);  //9 10 11
        store_data3d(store_data,current_v);  // 12 13 14
        store_data3d(store_data,current_a);  //15 16 17

        euler_angle=quaternion2euler_eigen(att_des_q.x(),att_des_q.y(),att_des_q.z(),att_des_q.w());

        store_data.push_back(euler_angle(0));  //18
        store_data.push_back(euler_angle(1));  // 19
        store_data.push_back(euler_angle(2));  //20
        store_data.push_back(thrust_des);    //21

        input_queue.push_back(store_data);
        if(input_queue.size()>10)
        {
            input_queue.erase(input_queue.begin());
        }

/*    ROS_INFO_THROTTLE(2.0, "Attitude Quaternion Setpoint is w=%f, x=%f, y=%f, z=%f, thrust=%f", att_setpoint.orientation.w,
                      att_setpoint.orientation.x, att_setpoint.orientation.y, att_setpoint.orientation.z, att_setpoint.thrust);*/

        att_ctrl_pub.publish(att_setpoint);
    }


}


void tracker_update_Callback(const pva_tracker::input::ConstPtr& msg)
{

/*    //反归一化
    //ROS_INFO_THROTTLE(2,"get message from python");
    std::vector<float> predict;
    for(int i=0;i<6;i++)
    {
        predict.push_back(msg->input[i] * max_min[i] + min_arr[i]);
    }
    predict_msg.pred_x=predict[0];
    predict_msg.pred_y=predict[1];
    predict_msg.pred_z=predict[2];
    predict_msg.pred_vx=predict[3];
    predict_msg.pred_vy=predict[4];
    predict_msg.pred_vz=predict[5];
    pre_pub.publish(predict_msg);

    ROS_INFO_THROTTLE(2,"predict: %f  %f   %f  %f  %f  %f    ",predict[0],predict[1],predict[2],predict[3],predict[4]
    ,predict[5]);


    a_des(0)=a_des(0)-(mpc_kpx*predict[0]+mpc_kvx*predict[3]);
    a_des(1)=a_des(1)-(mpc_kpy*predict[1]+mpc_kvy*predict[4]);
    a_des(2)=a_des(2)-(mpc_kpz*predict[2]+mpc_kvz*predict[5]);
    accel2quater(a_des,att_des_q,thrust_des);

    accel2quater(a_des,att_des_q,thrust_des);*/


    float roll=msg->input[0];
    float pitch=msg->input[1];
    float yaw=msg->input[2];
    float thrust_raw=msg->input[3];


    //fan gui yi hua
    roll=roll*max_min[6]+min_arr[6];
    pitch=pitch*max_min[7]+min_arr[7];
    yaw=yaw*max_min[8]+min_arr[8];
    float thrust_python=thrust_raw*max_min[9]+min_arr[9];
    ROS_INFO("get from python roll: %f pitch:  %f yaw: %f  thrust_des: %f",roll,pitch,yaw,thrust_python);


    Eigen::Quaterniond att_des_python=euler2quaternion_eigen(roll,pitch,yaw);

    att_setpoint.header.stamp = ros::Time::now();
    att_setpoint.orientation.w = att_des_python.w();
    att_setpoint.orientation.x = att_des_python.x();
    att_setpoint.orientation.y = att_des_python.y();
    att_setpoint.orientation.z = att_des_python.z();
    att_setpoint.thrust = thrust_python;

    std::vector<float> store_data;
    store_data3d(store_data,planned_p);  // 0 1 2
    store_data3d(store_data,planned_v); // 3 4 5
    store_data3d(store_data,planned_a);  // 6 7 8
    store_data3d(store_data,current_p);  //9 10 11
    store_data3d(store_data,current_v);  // 12 13 14
    store_data3d(store_data,current_a);  //15 16 17

    Vector3d euler_angle;
    euler_angle=quaternion2euler_eigen(att_des_python.x(),att_des_python.y(),att_des_python.z(),att_des_python.w());

    ROS_INFO("after compute roll: %f pitch:  %f yaw: %f  ",euler_angle(0),euler_angle(1),euler_angle(2));



    store_data.push_back(euler_angle(0));  //18
    store_data.push_back(euler_angle(1));  // 19
    store_data.push_back(euler_angle(2));  //20
    store_data.push_back(thrust_python);    //21

    input_queue.push_back(store_data);
    if(input_queue.size()>10)
    {
        input_queue.erase(input_queue.begin());
    }

    //ROS_INFO_THROTTLE(1.0, "Attitude Quaternion Setpoint is w=%f, x=%f, y=%f, z=%f, thrust=%f", att_setpoint.orientation.w,
                     // att_setpoint.orientation.x, att_setpoint.orientation.y, att_setpoint.orientation.z, att_setpoint.thrust);

    att_ctrl_pub.publish(att_setpoint);

}



void configureCallback(tracker::PVA_TrackerConfig &config, uint32_t level) {
    position_error_p << config.position_p_xy, config.position_p_xy, config.position_p_z;
    position_error_d << config.position_d_xy, config.position_d_xy, config.position_d_z;
    position_error_i << config.position_i_xy, config.position_i_xy, config.position_i_z;
    p_i_acc_error_limit = config.p_i_acc_error_limit;

    velocity_error_p << config.velocity_p_xy, config.velocity_p_xy, config.velocity_p_z;
    velocity_error_d << config.velocity_d_xy, config.velocity_d_xy, config.velocity_d_z;
    velocity_error_i << config.velocity_i_xy, config.velocity_i_xy, config.velocity_i_z;
    v_i_acc_error_limit = config.v_i_acc_error_limit;

    thrust_factor = config.hover_thrust_factor;

    mpc_kpx=config.mpc_k_px;
    mpc_kpy=config.mpc_k_py;
    mpc_kvx=config.mpc_k_vx;
    mpc_kvy=config.mpc_k_vy;
    mpc_kaxy=config.mpc_k_axy;
    mpc_kpz=config.mpc_k_pz;
    mpc_kvz=config.mpc_k_vz;
    mpc_kaz=config.mpc_k_az;



}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_tracker_pytorch");

    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig> server;
    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig>::CallbackType f;
    f = boost::bind(&configureCallback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle nh;

    module = torch::jit::load("/home/pengpeng/PycharmProjects/pythonProject/python_test/net_4999.pt");
    module.eval();

    ros::Subscriber position_sub = nh.subscribe("/mavros/local_position/pose", 1, positionCallback);
    ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/velocity_local", 1, velocityCallback);
    ros::Subscriber pva_sub = nh.subscribe("/pva_setpoint", 1, pvaCallback);

    att_ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    odom_sp_enu_pub = nh.advertise<nav_msgs::Odometry>("/odom_sp_enu", 1);

    pre_pub=nh.advertise<pva_tracker::input>("/pred", 1);

    nn_pub=nh.advertise<pva_tracker::input>("/nn_compute", 1);

    ros::Subscriber tracker_update_sub = nh.subscribe("/tracker_update", 1, tracker_update_Callback);





    ros::spin();
    return 0;
}
