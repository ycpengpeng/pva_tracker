//
// Created by pengpeng on 1/25/21.
//



//
// Created by cc on 2020/8/5.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

using namespace Eigen;
using namespace std;

Vector3d current_p,current_v,current_a,last_current_v,last_a;
mavros_msgs::State current_state;
ros::Publisher pva_pub;
ros::Time last_time;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /// ENU frame to NWU
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
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

/** This function is to generate state to state trajectory **/
void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
                       Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d af, double v_max, double delt_t,
                       Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t)
{
    // % Choose the time as running in average velocity
    // double decay_parameter = 0.5;
    // double T = 0.2;

    double j_limit = 1;
    double a_limit = 1;
    double v_limit = v_max;


//    double T1 = fabs(af(0)-a0(0))/j_limit > fabs(af(1)-a0(1))/j_limit ? fabs(af(0)-a0(0))/j_limit : fabs(af(1)-a0(1))/j_limit;
//    T1 = T1 > fabs(af(2)-a0(2))/j_limit ? T1 : fabs(af(2)-a0(2))/j_limit;
    double T2 = fabs(vf(0)-v0(0))/a_limit > fabs(vf(1)-v0(1))/a_limit ? fabs(vf(0)-v0(0))/a_limit : fabs(vf(1)-v0(1))/a_limit;
    T2 = T2 > fabs(vf(2)-v0(2))/a_limit ? T2 : fabs(vf(2)-v0(2))/a_limit;
    double T3 = fabs(pf(0)-p0(0))/v_limit > fabs(pf(1)-p0(1))/v_limit ? fabs(pf(0)-p0(0))/v_limit : fabs(pf(1)-p0(1))/v_limit;
    T3 = T3 > fabs(pf(2)-p0(2))/v_limit ? T3 : fabs(pf(2)-p0(2))/v_limit;

//    double T = T1 > T2 ? T1 : T2;
//    T = T > T3 ? T : T3;
    double T = T2;
    T = T > T3 ? T : T3;
    T = T < 0.3 ? 0.3 : T;

//    ROS_INFO_THROTTLE(2, "T=%lf", T);

    int times = T / delt_t+0.5;  //四舍五入

    p = Eigen::MatrixXd::Zero(times, 3);
    v = Eigen::MatrixXd::Zero(times, 3);
    a = Eigen::MatrixXd::Zero(times, 3);
    t = Eigen::VectorXd::Zero(times);

    // % calculate optimal jerk controls by Mark W. Miller
    for(int ii=0; ii<3; ii++)
    {
        double delt_a = af(ii) - a0(ii);
        double delt_v = vf(ii) - v0(ii) - a0(ii)*T;
        double delt_p = pf(ii) - p0(ii) - v0(ii)*T - 0.5*a0(ii)*T*T;

        //%  if vf is not free
        double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
        double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
        double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);

        for(int jj=0; jj<times; jj++)
        {
            double tt = (jj + 1)*delt_t;
            t(jj) = tt;
            p(jj,ii) = alpha/120*pow(tt,5) + beta/24*pow(tt,4) + gamma/6*pow(tt,3) + a0(ii)/2*pow(tt,2) + v0(ii)*tt + p0(ii);
            v(jj,ii) = alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0(ii)*tt + v0(ii);
            a(jj,ii) = alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0(ii);
        }
    }
}

void compute_circular_traj(const double r, const double vel, const Eigen::Vector3d p0, const double t,
                           Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a)
//@requires r > 0 && vel > 0 && t >= 0;
{
    const double theta = vel*t/r;

    p(0) = r*cos(theta) + p0(0) - r;
    p(1) = r*sin(theta) + p0(1);
    p(2) = p0(2);

    v(0) = -vel*sin(theta);
    v(1) = vel*cos(theta);
    v(2) = 0.0;

    a(0) = -vel*vel/r*cos(theta);
    a(1) = -vel*vel/r*sin(theta);
    a(2) = 0.0;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_random_fly");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, velocity_sub_cb);

    const int LOOPRATE = 30;
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
    double take_off_height = 4;
    double take_off_acc = 1.0;

    double take_off_time_half = sqrt(take_off_height/take_off_acc);
    double delt_t = 1.0 / LOOPRATE;
    double take_off_send_times = take_off_time_half / delt_t * 2;
    int counter = 0;
    Vector3d recorded_takeoff_position(current_p(0), current_p(1), current_p(2));

    double yaw_set = 0.0;

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
            setPVA(current_p, Vector3d::Zero(), Vector3d::Zero(), yaw_set);
        }else{
            counter ++;
            double z_sp, vz_sp;
            if(counter < take_off_send_times / 2){
                z_sp = 0.5*take_off_acc*counter*delt_t*counter*delt_t;
                vz_sp = counter*delt_t*take_off_acc;
                Vector3d p_sp(recorded_takeoff_position(0), recorded_takeoff_position(1), z_sp);
                Vector3d v_sp(0, 0, vz_sp);
                setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);

            }else if(counter < take_off_send_times){
                double t_this = (counter-take_off_send_times/2)*delt_t;
                z_sp = take_off_send_times/2*delt_t*take_off_acc*t_this - 0.5*take_off_acc*t_this*t_this;
                vz_sp = take_off_send_times/2*delt_t*take_off_acc - take_off_acc*t_this;

                Vector3d p_sp(recorded_takeoff_position(0), recorded_takeoff_position(1), z_sp);
                Vector3d v_sp(0, 0, vz_sp);
                setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);

            }else{
                Vector3d p_sp(recorded_takeoff_position(0), recorded_takeoff_position(1), take_off_height);
                setPVA(p_sp, Vector3d::Zero(), Vector3d::Zero(), yaw_set);
                counter --;
            }
        }

        if(current_p(2) > take_off_height-0.05){
            ROS_INFO_ONCE("Takeoff Complete!");
            break;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


    Vector3d eulerAngle(20,10,30); //yaw pitch roll

    eulerAngle=eulerAngle/180.0*3.14;
    Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=yawAngle*pitchAngle*rollAngle;
    
   const int number_of_points=7;
    
    Matrix<double,number_of_points,3> point_raw_matrix;
    float side=4.0;
    float fly_height=10;
    point_raw_matrix<<1.2,2.5,1.2,
            -3.2,2.1,2.5,
            -2.9,0.2,3.1,
            -1.5,2.0,4.2,
            2.0,-0.2,2.1,
            -1.0,1.2,3.4,
            1.2,-0.9,4.3        
            ;
    Matrix<double,3,number_of_points>  point_matrix;
    point_matrix = rotation_matrix*point_raw_matrix.transpose();
    point_matrix(2,0)+=fly_height;
    point_matrix(2,1)+=fly_height;
    point_matrix(2,2)+=fly_height;
    point_matrix(2,3)+=fly_height;

    cout<<"point_matrix； "<<point_matrix<<endl;


    /** Take off complete. Go to a point with minimum jerk trajectory **/
    double circle_radius = 1.8;

    MatrixXd p_t, v_t, a_t;
    Eigen::VectorXd t_vector;
    Vector3d v0(0.0, 0.0, 0.0);
    Vector3d a0(0.0, 0.0, 0.0);

    Vector3d pf(circle_radius, -circle_radius, take_off_height);
    Vector3d vf(0, 0, 0);
    Vector3d af(0, 0, 0);


    int point_number=0;
    while(ros::ok())
    {
        motion_primitives(current_p, current_v, a0, point_matrix.col(point_number), vf, af, 2.0, delt_t, p_t, v_t, a_t, t_vector);
        cout<<"t_vector.size: "<<t_vector.size()<<endl;
        cout<<"p_t-------"<<endl;
        cout<<p_t<<endl;
        cout<<"-------"<<endl;
        cout<<"v_t-------"<<endl;
        cout<<v_t<<endl;
        cout<<"-------"<<endl;
        cout<<"a_t-------"<<endl;
        cout<<a_t<<endl;
        cout<<"-------"<<endl;
        int i=0;
        while((current_p-point_matrix.col(point_number)).norm()>=0.2)
        {

            setPVA(p_t.row(i), v_t.row(i), a_t.row(i), yaw_set);// a_t.row(i));
            loop_rate.sleep();
            ros::spinOnce();
            i++;
            if(i>=t_vector.size()-1)
            {
                i=t_vector.size()-1;
            }
        }
        point_number++;
        if(point_number==number_of_points)
        {
            point_number=0;
        }
        cout<<"target: "<<point_matrix.col(point_number)<<endl;
        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
