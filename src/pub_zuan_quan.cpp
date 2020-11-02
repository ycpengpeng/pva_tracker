//
// Created by pengpeng on 11/2/20.
//

//
// Created by pengpeng on 10/29/20.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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
using namespace Eigen;

mavros_msgs::State current_state;

Matrix<double, 4, 3> planned_p;

Matrix<double, 4, 3> planned_v;

Vector3d planned_a(0,0,0);
Eigen::Vector4d planned_yaw;
ros::Publisher zuan_quan_point_pub;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_zuan_quan");
    ros::NodeHandle nh;

    //ROS_INFO("QQQQ");

    planned_p << -1.5,1.0,1.8,
            -2,1.0,1.8,
            -4.2,2.5,1.8,
            -6.2,2.5,1.8;

    //ROS_INFO("planned_p(0):%f",planned_p(0));
    planned_v<<-1,0,0,
            -1,0,0,
            -1,0,0,
            0,0,0;

    planned_yaw<<180.0/180.0*PI,180.0/180*PI,180.0/180*PI,180.0/180*PI;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    zuan_quan_point_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/zuan_quan_setpoint", 10);

    ros::Rate rate(40.0);


    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z =0.5;
    double theta=180.0/180.0*PI;
    pose.pose.orientation.w=cos(theta/2);
    pose.pose.orientation.x=0;
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=sin(theta/2);

    int quan_num=0;
    nh.param<int>("int_param",quan_num, 0);
    nh.setParam("int_param", 0);

    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

       // static unsigned int count=0;

//        count++;
//        if(count>1000)
//        {
//            break;
//        }
        //local_pos_pub.publish(pose);

        nh.getParam("int_param", quan_num);
       // ROS_INFO("quan_num:%d",quan_num);

        set_zuan_quan_point(planned_p.row(quan_num),planned_v.row(quan_num),planned_a,planned_yaw(quan_num));

        ros::spinOnce();
        rate.sleep();

        //local_pos_pub.publish(pose);
    }


}
