//
// Created by cc on 2020/8/4.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <pva_tracker/PVA_TrackerConfig.h>

#define GRAVITATIONAL_ACC 9.81
#define THRUST_FACTOR 0.058   // When THRUST_FACTOR is 1, it means thrust is desired acceleration

using namespace Eigen;

// Coefficients
Vector3d position_error_p;
Vector3d velocity_error_p;

// Global Variables
Vector3d planned_p;
Vector3d planned_v;
Vector3d planned_a;
double planned_yaw;
Vector3d current_p;
Vector3d current_v;
Quaterniond current_att;
ros::Publisher att_ctrl_pub;

Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2)
{
    Vector3d result;
    result << v1(0)*v2(0), v1(1)*v2(1), v1(2)*v2(2);
    return result;
}

void pvaCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{
    mavros_msgs::AttitudeTarget att_setpoint;

    /// NWU frame to ENU frame
    planned_p << -msg->positions[1], msg->positions[0], msg->positions[2];
    planned_yaw = msg->positions[3];
    planned_v << -msg->velocities[1], msg->velocities[0], msg->velocities[2];
    planned_a << -msg->accelerations[1], msg->accelerations[0], msg->accelerations[2];

    /// Calculate desired thrust and attitude
    Vector3d p_error = planned_p - current_p;
    Vector3d v_error = planned_v - current_v;

    Vector3d a_fb =   /// TODO: USE PID controller and add maximum error?
            vectorElementMultiply(p_error, position_error_p) + vectorElementMultiply(v_error, velocity_error_p);

    Vector3d z_w_norm(0, 0, 1.0);
    Vector3d a_des = a_fb + planned_a + GRAVITATIONAL_ACC * z_w_norm;
    Vector3d att_des_norm = a_des / a_des.norm();

    Quaterniond att_des = Quaterniond::FromTwoVectors(z_w_norm, att_des_norm);

    //add yaw
    Quaterniond yaw_quat(cos(planned_yaw/2.0), att_des_norm(0)*sin(planned_yaw/2.0),
            att_des_norm(1)*sin(planned_yaw/2.0),att_des_norm(2)*sin(planned_yaw/2.0));
    att_des = yaw_quat * att_des;

    //Calculate thrust
    Quaterniond z_w_quat(0, 0, 0, 1.0);
    Quaterniond att_current_vector_quat = current_att * z_w_quat * current_att.inverse();
    Vector3d att_current_vector(att_current_vector_quat.x(), att_current_vector_quat.y(),
                                att_current_vector_quat.z());
    double thrust_des = a_des.dot(att_current_vector) * THRUST_FACTOR;

    att_setpoint.header.stamp = ros::Time::now();
    att_setpoint.orientation.w = att_des.w();
    att_setpoint.orientation.x = att_des.x();
    att_setpoint.orientation.y = att_des.y();
    att_setpoint.orientation.z = att_des.z();
    att_setpoint.thrust = thrust_des;

    ROS_INFO_THROTTLE(1.0, "Attitude Quaternion Setpoint is w=%f, x=%f, y=%f, z=%f, thrust=%f", att_setpoint.orientation.w,
            att_setpoint.orientation.x, att_setpoint.orientation.y, att_setpoint.orientation.z, att_setpoint.thrust);

    att_ctrl_pub.publish(att_setpoint);
}


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
}

void configureCallback(tracker::PVA_TrackerConfig &config, uint32_t level) {
    position_error_p << config.position_error_p_xy, config.position_error_p_xy, config.position_error_p_z;
    velocity_error_p << config.velocity_error_p_xy, config.velocity_error_p_xy, config.velocity_error_p_z;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker");

    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig> server;
    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig>::CallbackType f;
    f = boost::bind(&configureCallback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle nh;

    ros::Subscriber pva_sub = nh.subscribe("/pva_setpoint", 1, pvaCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, velocityCallback);

    /// TODO: check if the topic is right.
    att_ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

    ros::spin();
    return 0;
}