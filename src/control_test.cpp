//
// Created by cc on 2020/8/5.
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

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /// ENU frame to NWU
    current_p << msg->pose.position.y, -msg->pose.position.x, msg->pose.position.z;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    ros::Publisher pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    const int LOOPRATE = 40;
    ros::Rate loop_rate(LOOPRATE);

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


    /// Take off with constant acceleration
    double take_off_height = 1.0;
    double take_off_acc = 1.0;

    double take_off_time_half = sqrt(take_off_height/take_off_acc);
    double delt_t = 1.0 / LOOPRATE;
    double take_off_send_times = take_off_time_half / delt_t * 2;
    int counter = 0;
    Vector3d recorded_takeoff_position(current_p(0), current_p(1), current_p(2));

    ROS_INFO("Arm and takeoff");
    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        trajectory_msgs::JointTrajectoryPoint pva_setpoint;

        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            pva_setpoint.positions.push_back(current_p(0)); //x
            pva_setpoint.positions.push_back(current_p(1)); //y
            pva_setpoint.positions.push_back(current_p(2)); //z
            pva_setpoint.positions.push_back(0.0); /// TODO: Yaw should also use current value

            pva_setpoint.velocities.push_back(0);
            pva_setpoint.velocities.push_back(0);
            pva_setpoint.velocities.push_back(0);

        }else{
            counter ++;
            if(counter < take_off_send_times / 2){
                pva_setpoint.positions.push_back(recorded_takeoff_position(0)); //x
                pva_setpoint.positions.push_back(recorded_takeoff_position(1)); //y
                pva_setpoint.velocities.push_back(0);
                pva_setpoint.velocities.push_back(0);

                pva_setpoint.positions.push_back(0.5*take_off_acc*counter*delt_t*counter*delt_t); //z
                pva_setpoint.velocities.push_back(counter*delt_t*take_off_acc);  //vz
            }else if(counter < take_off_send_times){
                pva_setpoint.positions.push_back(recorded_takeoff_position(0)); //x
                pva_setpoint.positions.push_back(recorded_takeoff_position(1)); //y
                pva_setpoint.velocities.push_back(0);
                pva_setpoint.velocities.push_back(0);

                double t_this = (counter-take_off_send_times/2)*delt_t;
                pva_setpoint.positions.push_back(take_off_send_times/2*delt_t*take_off_acc*t_this - 0.5*take_off_acc*t_this*t_this); //z
                pva_setpoint.velocities.push_back(take_off_send_times/2*delt_t*take_off_acc - take_off_acc*t_this);  //vz
            }else{
                pva_setpoint.positions.push_back(5); //x
                pva_setpoint.positions.push_back(-5); //y
                pva_setpoint.velocities.push_back(0);
                pva_setpoint.velocities.push_back(0);

                pva_setpoint.positions.push_back(take_off_height); //z
                pva_setpoint.velocities.push_back(0.0);  //vz
                counter --;
            }
        }

        pva_setpoint.positions.push_back(0.0); //yaw
        pva_setpoint.accelerations.push_back(0);
        pva_setpoint.accelerations.push_back(0);
        pva_setpoint.accelerations.push_back(0);
        pva_pub.publish(pva_setpoint);

//        if(current_p(2) > take_off_height-0.05){
//            ROS_WARN("Takeoff Complete!");
//            break;
//        }

        ROS_INFO_THROTTLE(1.0, "P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
        ROS_INFO_THROTTLE(1.0, "V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
        ROS_INFO_THROTTLE(1.0, "A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
