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
Quaterniond current_att;
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
    ros::init(argc, argv, "hover_test_on_up");
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
    double into_offb=0;
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
            take_off_position=current_p;
            into_offb=0;
/*            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z =0;
            double theta=0/180.0*3.14;
            pose.pose.orientation.w=cos(theta/2);
            pose.pose.orientation.x=0.0;
            pose.pose.orientation.y=0;
            pose.pose.orientation.z=sin(theta/2);
            local_pos_pub.publish(pose);*/
            setPVA(current_p,Vector3d::Zero(),Vector3d::Zero(),yaw_set);
            continue;
        }
        Vector3d p_sp(take_off_position(0), take_off_position(1), take_off_position(2));
        Vector3d v_sp(0, 0, 0);
        setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);
        ros::spinOnce();
        loop_rate.sleep();




/*         Vector3d recorded_position = current_p;
         while(ros::ok()){
             if(current_state.mode != "OFFBOARD" || !current_state.armed){
                 setPVA(current_p, Vector3d::Zero(), Vector3d::Zero(), yaw_set);
                 recorded_position = current_p;
             }else{
                 setPVA(recorded_position, Vector3d::Zero(), Vector3d::Zero(), yaw_set);
             }

             loop_rate.sleep();
             ros::spinOnce();
         }*/


    }
    return 0;
}

