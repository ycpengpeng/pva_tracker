//
// Created by pengpeng on 11/4/20.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

using namespace Eigen;

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    //ROS_INFO("X:%f Y:%f  Z:%f",-current_p(0),current_p(1),current_p(2));
}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;

}

void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;

    pva_setpoint.positions.push_back(p(0)); //x
    pva_setpoint.positions.push_back(p(1)); //y
    pva_setpoint.positions.push_back(p(2)); //z
    pva_setpoint.positions.push_back(yaw);

    pva_setpoint.velocities.push_back(v(0));
    pva_setpoint.velocities.push_back(v(1));
    pva_setpoint.velocities.push_back(v(2));

    pva_setpoint.accelerations.push_back(a(0));
    pva_setpoint.accelerations.push_back(a(1));
    pva_setpoint.accelerations.push_back(a(2));

    pva_pub.publish(pva_setpoint);

//    ROS_INFO_THROTTLE(1.0, "P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
//    ROS_INFO_THROTTLE(1.0, "V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
//    ROS_INFO_THROTTLE(1.0, "A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);
//
//    ROS_INFO("P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
//    ROS_INFO("V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
//    ROS_INFO("A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_take_off_on_up");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

    const int LOOPRATE = 40;
    ros::Rate loop_rate(LOOPRATE);

    double take_off_height = 1.5;
    double take_off_acc = 0.1;
    double take_off_time=sqrt(take_off_height/take_off_acc)*2.0;
    double delt_t = 1.0 / LOOPRATE;
    double take_off_send_times = take_off_time / delt_t;
    int counter = 0;
    int take_off_init=1;
    Vector3d take_off_position;
    ros::Time last_time=ros::Time::now();
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    geometry_msgs::PoseStamped pose;

    double yaw_set = 3.1415926;
    int mode=0;
    double z_sp=0;
    double vz_sp=0;
    ROS_INFO("Arm and ta1111keoff");

    while(ros::ok())
    {

        if(current_state.mode != "OFFBOARD" || !current_state.armed)
        {
            ros::spinOnce();
            loop_rate.sleep();
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z =0;
            double theta=0/180.0*3.14;
            pose.pose.orientation.w=cos(theta/2);
            pose.pose.orientation.x=0.0;
            pose.pose.orientation.y=0;
            pose.pose.orientation.z=sin(theta/2);
            local_pos_pub.publish(pose);
            continue;
        }

        if(mode==0)
        {

            if(take_off_init==1)
            {
                ros::spinOnce();
                take_off_position=current_p;
                take_off_height = 1.5;
                take_off_acc = 0.1;
                take_off_time=sqrt(take_off_height/take_off_acc)*2.0;
                take_off_send_times = take_off_time / delt_t;
                counter=0;
                take_off_init=0;
                last_time=ros::Time::now();

            }

            if(counter < take_off_send_times / 2.0)
            {

                z_sp = 0.5*take_off_acc*counter*delt_t*counter*delt_t;
                vz_sp = counter*delt_t*take_off_acc;
                Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
                Vector3d v_sp(0, 0, vz_sp);
                setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);

            }
            else if(counter < take_off_send_times)
            {

                double t_this = (counter-take_off_send_times/2.0)*delt_t;
                //ROS_INFO("THIS:%f",t_this);
                //z_sp = take_off_send_times/2*delt_t*take_off_acc*t_this - 0.5*take_off_acc*t_this*t_this;

                vz_sp = take_off_send_times/2.0*delt_t*take_off_acc - take_off_acc*t_this;

                z_sp=1.0/2.0*take_off_acc*(take_off_time/2.0)*(take_off_time/2.0)+(vz_sp+take_off_time/2.0*take_off_acc)*t_this/2.0;

                //ROS_INFO("Z_SP:%f",z_sp);
                Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
                Vector3d v_sp(0, 0, vz_sp);
                setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);

            }
            else
            {
                Vector3d p_sp(take_off_position(0), take_off_position(1), take_off_height);
                setPVA(p_sp, Vector3d::Zero(), Vector3d::Zero(), yaw_set);
//                counter --;
            }
            counter++;
            if(ros::Time::now() - last_time > ros::Duration(20))
            {
                mode=1;
                take_off_init=1;
                ROS_INFO("Land Mode--------------");
            }
        }
        else if(mode==1)
        {
            if(take_off_init==1)
            {
                ros::spinOnce();
                take_off_position=current_p;
               // ROS_INFO("Z:%f",take_off_position(2));
                take_off_acc=0.1;
                take_off_time=sqrt(take_off_position(2)/take_off_acc)*2.0;
                take_off_send_times=take_off_time/delt_t;
               // ROS_INFO("aaaaaaaatake_off_send_times:%f",take_off_send_times);
                counter=0;
                take_off_init=0;
                last_time=ros::Time::now();
            }
            //ROS_INFO("take_off_send_times:%f",take_off_send_times);
            //ROS_INFO("counter:%d",counter);

            if(counter < take_off_send_times / 2)
            {
                //ROS_INFO("DDDDDDD");
                z_sp = take_off_position(2)-0.5*take_off_acc*counter*delt_t*counter*delt_t;
                vz_sp = -counter*delt_t*take_off_acc;
                ROS_INFO("z_sp:%f",z_sp);
                Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
                Vector3d v_sp(0, 0, vz_sp);
                setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);

            }
            else if(counter < take_off_send_times)
            {
                //ROS_INFO("SSSSSS");

                double t_this = (counter-take_off_send_times/2.0)*delt_t;
                vz_sp = -(take_off_send_times/2.0*delt_t*take_off_acc - take_off_acc*t_this);
                z_sp = take_off_position(2)-(1.0/2.0*take_off_acc*(take_off_time/2.0)*(take_off_time/2.0)+(-vz_sp+take_off_time/2.0*take_off_acc)*t_this/2.0);
               // ROS_INFO("vz_sp:%f",vz_sp);
                //ROS_INFO("z_sp:%f",z_sp);
                Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
                Vector3d v_sp(0, 0, vz_sp);
                setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);

            }
            else
            {
                Vector3d p_sp(take_off_position(0), take_off_position(1), 0);
                setPVA(p_sp, Vector3d::Zero(), Vector3d::Zero(), yaw_set);
            }
            counter++;
            if(ros::Time::now() - last_time > ros::Duration(20))
            {
                mode=0;
                take_off_init=1;
                ROS_INFO("TAKE OFF Mode--------------");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

