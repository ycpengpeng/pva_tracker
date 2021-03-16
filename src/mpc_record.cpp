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

#define PI 3.1415926
#define number 10
using namespace Eigen;
using namespace std;

mavros_msgs::State current_state;

ros::Time last_time;

Vector3d current_p;
Vector3d current_v,last_current_v;
Vector3d current_a,last_a;

Vector3d last_current_p;


Vector3d planned_p;
Vector3d planned_v;
Vector3d planned_a;

Vector3d last_planned_p(0,0,0);
Vector3d last_planned_v(0,0,0);
Vector3d last_planned_a(0,0,0);



double planned_yaw;
Vector4d set_orientation;
double set_thrust;

std::ofstream outFile;
//
//void state_cb(const mavros_msgs::State::ConstPtr& msg){
//    current_state = *msg;
//}

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
    temp.y() = asin(2.0 * (-z * x + w * y));
    temp.z() = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}



void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}


void velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    //ROS_INFO("velocity sub cb!!!");
    current_v << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    //ROS_INFO("VX %f VY %f VZ  %f",current_v(0),current_v(1),current_v(2));
    current_a=(current_v-last_current_v)/(ros::Time::now().toSec()-last_time.toSec());


    if(current_a(0)>100||current_a(0)<-100)
    {
        //ROS_INFO("A TOO BIG");
        current_a=last_a;
    }
//    ROS_INFO("QQQQQ  a0  :%f   a1:%f    a2:%f ",current_a(0),current_a(1),current_a(2));
    last_time=ros::Time::now();
    last_current_v=current_v;
    last_a=current_a;

}


void pvaCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{

    planned_p << msg->positions[0], msg->positions[1], msg->positions[2];
    planned_yaw = msg->positions[3];
    planned_v << msg->velocities[0], msg->velocities[1], msg->velocities[2];
    planned_a << msg->accelerations[0], msg->accelerations[1], msg->accelerations[2];
}
void att_ctrl_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{

    if(current_state.mode == "OFFBOARD")
    {

        if(last_current_p(0)!=current_p(0)||last_current_p(1)!=current_p(1))
        {


            set_orientation <<msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w;
            //roll pitch yaw
            Eigen::Vector3d euler=quaternion2euler_eigen(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
            set_thrust=msg->thrust;
//    ROS_INFO("VX %f VY %f VZ  %f",current_v(0),current_v(1),current_v(2));
//规划位置1*3，规划速度1*3，规划加速度1*3，实际位置1*3，实际速度1*3，实际加速度1*3，
// 位置差值1*3，速度差值1*3，加速度差值1*3， 给定si yuan shu 1*4，gei ding ou la jiao1*3,给定推力1*1


            outFile <<planned_p(0)<< ','<< planned_p(1) << ','<<planned_p(2)
                    << ','<<planned_v(0)<< ','<< planned_v(1) << ','<<planned_v(2)
                    << ','<<planned_a(0)<< ','<< planned_a(1) << ','<<planned_a(2)
                    << ','<<current_p(0) << ','<<current_p(1)<<','<<current_p(2)
                    << ','<<current_v(0)<< ','<<current_v(1)<< ','<<current_v(2)
                    << ','<<current_a(0)<< ','<<current_a(1)<< ','<<current_a(2)

                    << ','<<current_p(0)-last_planned_p(0)<< ','<<current_p(1)-last_planned_p(1)<<','<<current_p(2)-last_planned_p(2)
                    << ','<<current_v(0)-last_planned_v(0)<< ','<<current_v(1)-last_planned_v(1)<< ','<<current_v(2)-last_planned_v(2)
                    << ','<<current_a(0)-last_planned_a(0)<< ','<<current_a(1)-last_planned_a(1)<< ','<<current_a(2)-last_planned_a(2)

                    << ','<<set_orientation(0)<< ','<<set_orientation(1)<< ','<<set_orientation(2)<<','<<set_orientation(3)

                    << ','<<euler(0)<< ','<<euler(1)<< ','<<euler(2)
                    << ','<<set_thrust
                    << '\n';
            last_planned_p=planned_p;
            last_planned_v=planned_v;
            last_planned_a=planned_a;


        }


    }
    last_current_p=current_p;



}


/*
void set_zuan_quan_point(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{

    trajectory_msgs::JointTrajectoryPoint zuan_quan_setpoint;

    zuan_quan_setpoint.positions.push_back(p(0)); //x
    zuan_quan_setpoint.positions.push_back(p(1)); //y
    zuan_quan_setpoint.positions.push_back(p(2)); //z
    zuan_quan_setpoint.positions.push_back(yaw);
    //ROS_INFO("P(0)  %f P(1) %f P(2) %f",p(0),p(1),p(2));

    zuan_quan_setpoint.velocities.push_back(v(0));
    zuan_quan_setpoint.velocities.push_back(v(1));
    zuan_quan_setpoint.velocities.push_back(v(2));

    zuan_quan_setpoint.accelerations.push_back(a(0));
    zuan_quan_setpoint.accelerations.push_back(a(1));
    zuan_quan_setpoint.accelerations.push_back(a(2));

    zuan_quan_point_pub.publish(zuan_quan_setpoint);
}

*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_record");
    ros::NodeHandle nh;

    std::string file="real_drone1/test2.csv";

    outFile.open(file, std::ios::out);
    cout<<"record data to "<<file<<endl;

    outFile.trunc;

/*    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");*/

//    zuan_quan_point_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/zuan_quan_setpoint", 1);
    ros::Rate rate(30.0);
    //ROS_INFO("1111");
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vicon_pose", 1, positionCallback);
    ros::Subscriber pva_sub = nh.subscribe("/pva_setpoint", 1, pvaCallback);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vicon_vel", 1, velocity_sub_cb);
    ros::Subscriber att_ctrl_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1,att_ctrl_cb);


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);

    // wait for FCU connection
    while(ros::ok() )
    {

        ros::spinOnce();
        //rate.sleep();
    }



}
