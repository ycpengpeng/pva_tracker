//
// Created by cc on 2020/8/5.
//

#include <plan_zuan_quan.h>

using namespace Eigen;

int COUNT_QUAN=1;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

euler ang;

pva_table *table = (pva_table*)malloc(sizeof(pva_table));

MatrixXd p_t, v_t, a_t;
VectorXd yaw_t;
Vector3d current_p(0,0,0);
Vector3d current_v(0,0,0);
Vector3d last_current_v(0,0,0);
Vector3d current_a(0,0,0);
Vector3d last_a(0,0,0);
Vector3d planned_p,planned_v,planned_a;
Vector3d last_planned_p(0,0,0);
double planned_yaw;

Quaterniond current_att;
int I=0;

//Quaterniond _q(0,0.7071,0.7071,0);  //从北东地到东北天的四元数
Quaterniond _q(1,0,0,0);

ros::Time last_time;


mavros_msgs::AttitudeTarget att_setpoint;
ros::Publisher att_ctrl_pub;
ros::Publisher odom_sp_enu_pub;
ros::Publisher path_pub;

unsigned int t_number=0;     //离散点的数量


void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
/*    /// ENU frame to NWU
    current_p << msg->pose.position.y, -msg->pose.position.x, msg->pose.position.z;*/
    static ros::Time position_cb_time=ros::Time::now();


    //ROS_INFO("duration:   %f",ros::Time::now().toSec()-position_cb_time.toSec());
    position_cb_time=ros::Time::now();
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    current_att.w() = msg->pose.orientation.w;
    current_att.x() = msg->pose.orientation.x;
    current_att.y() = msg->pose.orientation.y;
    current_att.z() = msg->pose.orientation.z;

    current_att=_q*current_att*_q.inverse();



//ROS_INFO("X:%f Y:%f  Z:%f",-current_p(0),current_p(1),current_p(2));
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
    //ROS_INFO("QQQQQ  a0  :%f   a1:%f    a2:%f ",current_a(0),current_a(1),current_a(2));
    last_time=ros::Time::now();
    last_current_v=current_v;
    last_a=current_a;

}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;

}
void motion_primitives_with_table(Vector3d p0,Vector3d v0,Vector3d a0,Vector3d pf,Vector3d vf,Vector3d af,unsigned int &t_num,
                                  double yawf);

void zuan_quan_set_point_cb(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{
    //ros::Time tmp=ros::Time::now();


    planned_p << msg->positions[0], msg->positions[1], msg->positions[2];


    if(last_planned_p(0)==planned_p(0)&&last_planned_p(1)==planned_p(1)&&last_planned_p(2)==planned_p(2))
    {
        return;
    }

    //ROS_INFO("planned_p(0)  %f  planned_p(1)  %f  planned_p(2)   %f",planned_p(0),planned_p(1),planned_p(2));
    planned_yaw = msg->positions[3];
    planned_v << msg->velocities[0], msg->velocities[1], msg->velocities[2];
    planned_a << msg->accelerations[0], msg->accelerations[1], msg->accelerations[2];
    //ROS_INFO("planned_a %f %f %f",planned_a(0),planned_a(1),planned_a(2));
    static bool init=1;
    if(init==0)
    {
        current_p=p_t.row(t_number-1);
        current_v=v_t.row(t_number-1);
        current_a=a_t.row(t_number-1);

    }
    init=0;

    nav_msgs::Odometry path_position;
    path_position.header.stamp = ros::Time::now();
    path_position.pose.pose.position.y=current_p(0);
    path_pub.publish(path_position);

    motion_primitives_with_table(current_p,current_v,current_a,planned_p,planned_v,planned_a,t_number,planned_yaw);
    last_planned_p=planned_p;
    I=0;
    COUNT_QUAN=-COUNT_QUAN;
   // ROS_INFO("planned_p(0)  %f  planned_p(1)  %f  planned_p(2)   %f",planned_p(0),planned_p(1),planned_p(2));
    //ROS_INFO("duration:   %f",ros::Time::now().toSec()-tmp.toSec());
}

void motion_primitives_with_table(Vector3d p0,Vector3d v0,Vector3d a0,Vector3d pf,Vector3d vf,Vector3d af,unsigned int &t_num,
                                  double yawf)
{
    double T1, T2, T3, T;
    double delt_x, delt_y, delt_z;
    //ROS_INFO("pf(0):  %f,pf(1):   %f,pf(2):   %f",pf(0),pf(1),pf(2));
    delt_x=pf(0)-p0(0);
    delt_y=pf(1)-p0(1);
    delt_z=pf(2)-p0(2);
    T1 = table->query_pva_table(delt_x, v0(0), vf(0), a0(0));
    T2 = table->query_pva_table(delt_y, v0(1), vf(1), a0(1));
    T3 = table->query_pva_table(delt_z, v0(2), vf(2), a0(2));
    T = T1 > T2 ? T1 : T2;
    T = T > T3 ? T : T3;
    T = T < 0.5 ? 0.5 : T;
    if(T==-1)
    {
        ROS_INFO("T=-1////////////////");
    }

    //ROS_INFO("T:%f",T);
    t_num=T/delta_t;  //number of dots
    //ROS_INFO("computer:t_number:%d",t_num);
    //ROS_INFO("t_num:%d   qqq",t_num);
    p_t = Eigen::MatrixXd::Zero(t_num, 3);
    v_t = Eigen::MatrixXd::Zero(t_num, 3);
    a_t = Eigen::MatrixXd::Zero(t_num, 3);
    //t = Eigen::VectorXd::Zero(t_num);
    for (int column = 0; column < 3; column++)
    {
        double delt_a = af(column) - a0(column);
        double delt_v = vf(column) - v0(column) - a0(column)*T;
        double delt_p = pf(column) - p0(column) - v0(column)*T - 0.5*a0(column)*T*T;

        // % if vf is not free
        double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
        double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
        double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);


        for (int times = 0; times < t_num; times++)
        {

            //ROS_INFO("times=%d",times);
            double tt = (times + 1)*delta_t;
            p_t(times, column) = alpha / 120 * pow(tt, 5) + beta / 24 * pow(tt, 4) + gamma / 6 * pow(tt, 3) +
                                 a0(column) / 2 * pow(tt, 2) +v0(column) * tt + p0(column);
            v_t(times, column) = alpha / 24 * pow(tt, 4) + beta / 6 * pow(tt, 3) + gamma / 2 * pow(tt, 2) +
                                 a0(column) * tt + v0(column);
            a_t(times, column) = alpha / 6 * pow(tt, 3) + beta / 2 * pow(tt, 2) + gamma * tt + a0(column);

        }
        //ROS_INFO("p_t(t_num-1,0):  %f,p_t(t_num-1,1):  %f,p_t(t_num-1,2):  %f",p_t(t_num-1,0),p_t(t_num-1,1),p_t(t_num-1,2));

    }
    //ROS_INFO("p_t(t_num-1,0):  %f,p_t(t_num-1,1):  %f,p_t(t_num-1,2):  %f",p_t(t_num-1,0),p_t(t_num-1,1),p_t(t_num-1,2));

    yaw_t = Eigen::VectorXd::Zero(t_num);
    for(int times=0;times<t_num;times++)
    {
        yaw_t(times)=yawf;
    }


}


void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;
    static double last_send_px=0;

    if(p(0)-last_send_px>0.5)
    {
        ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        ROS_INFO("now p(0):%f",p(0));
        ROS_INFO("last_send_px:%f",last_send_px);
        ROS_INFO("planned_p(0):%f",planned_p(0));
        ROS_INFO("NOW I:%d",I);
        ROS_INFO("NOW T_NUMBBER:%d",t_number);
//        ROS_INFO("current_p(0):%f",current())
        ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");

    }

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



    /// Publish to record in rosbag
    nav_msgs::Odometry odom_sp_enu;
    odom_sp_enu.header.stamp = ros::Time::now();
    odom_sp_enu.pose.pose.position.x = p(0);
    odom_sp_enu.pose.pose.position.y = p(1);
    odom_sp_enu.pose.pose.position.z = p(2);
    odom_sp_enu.twist.twist.linear.x = v(0);
    odom_sp_enu.twist.twist.linear.y = v(1);
    odom_sp_enu.twist.twist.linear.z = v(2);
    odom_sp_enu_pub.publish(odom_sp_enu);

//    nav_msgs::Odometry path_position;
//    path_position.header.stamp = ros::Time::now();
//    path_position.pose.pose.position.x=current_p(0);
//
//    path_pub.publish(path_position);


    pva_pub.publish(pva_setpoint);
    last_send_px=p(0);

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
    ros::init(argc, argv, "plan_zuan_quan");
    ros::NodeHandle nh;
    table->csv2pva_table("/home/pengpeng/Desktop/p3_v1-5_a3_res0-1.csv");

    ros::Rate loop_rate(LOOPRATE);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, velocity_sub_cb);

    ros::Subscriber traj_point_sub = nh.subscribe<trajectory_msgs::JointTrajectoryPoint>("/zuan_quan_setpoint", 1, zuan_quan_set_point_cb);

    ros::Publisher velocity_pub=nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    odom_sp_enu_pub = nh.advertise<nav_msgs::Odometry>("/odom_sp_enu", 1);
    path_pub=nh.advertise<nav_msgs::Odometry>("/path", 1);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

//    mavros_msgs::SetMode offb_set_mode;
//    offb_set_mode.request.custom_mode = "OFFBOARD";
//
//    mavros_msgs::CommandBool arm_cmd;
//    arm_cmd.request.value = true;
//    ros::Time last_request = ros::Time::now();

    ROS_INFO("START_TAKE OFF!!------------------------------");
    while(ros::ok())
    {
//        if( current_state.mode != "OFFBOARD" &&
//            (ros::Time::now() - last_request > ros::Duration(5.0))){
//            if( set_mode_client.call(offb_set_mode) &&
//                offb_set_mode.response.mode_sent){
//                ROS_INFO("Offboard enabled");
//            }
//            last_request = ros::Time::now();
//        } else {
//            if( !current_state.armed &&
//                (ros::Time::now() - last_request > ros::Duration(5.0))){
//                if( arming_client.call(arm_cmd) &&
//                    arm_cmd.response.success){
//                    ROS_INFO("Vehicle armed");
//                }
//                last_request = ros::Time::now();
//            }
//        }

        static unsigned int count=0;
        count++;
        if(count>1000)
        {
            break;
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z =0.8;

        double theta=180.0/180.0*3.14;
        pose.pose.orientation.w=cos(theta/2);
        pose.pose.orientation.x=0;
        pose.pose.orientation.y=0;
        pose.pose.orientation.z=sin(theta/2);
        local_pos_pub.publish(pose);


        ros::spinOnce();
        loop_rate.sleep();

    }

    ROS_INFO("START_ZUAN_QUAN!!------------------------------");

    while(ros::ok())
    {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        //ROS_INFO("X:%f",current_p(0));
        static unsigned char quan_num = 0;
        static unsigned int last_t_number=0;
        for(I=0;I<t_number-1;I++)
        {
            //ROS_INFO("QQQQ");
            //ROS_INFO("X:%f",current_p(0));
            setPVA(p_t.row(I), v_t.row(I), a_t.row(I), yaw_t(I));//a_t.row(last_index));
            //ROS_INFO("planned_p(0)  %f  planned_p(1)  %f  planned_p(2)   %f",planned_p(0),planned_p(1),planned_p(2));

            //ros::spinOnce();
            loop_rate.sleep();
        }
        if(I==t_number-1&&t_number!=0)
        {
            quan_num++;
            if(quan_num==10)
            {
                break;
            }
            //ROS_INFO("next_quan_num:%d",quan_num);
            nh.setParam("int_param", quan_num);
        }
        last_t_number=t_number;
        ros::spinOnce();
        unsigned int qu=0;
        ros::Time tmp=ros::Time::now();
        while(t_number!=0&&I!=0)
        {
//            Vector3d p_tmp;
//            p_tmp=p_t.row(t_number)+v_t.row(t_number);
//            geometry_msgs::Twist velocity;
//            velocity.linear.x=v_t(t_number-1,0)/2;
//            velocity.linear.y=v_t(t_number-1,1)/2;
//            velocity.linear.z=v_t(t_number-1,2)/2;
//            velocity_pub.publish(velocity);
           // ROS_INFO("X:%f",current_p(0));
            qu++;
            if(t_number<50)
            {
               // ROS_INFO("IUYYYYWUYUWYEUWYYYEEE%%%%%%%%%YYYYYYY");

            }
           // ROS_INFO("WWWW");
            setPVA(p_t.row(t_number-1), v_t.row(t_number-1), a_t.row(t_number-1), yaw_t(t_number-1));//a_t.row(last_index));
            //ROS_INFO("------!!!!!!!!!!!!!!!!!!!-----------------------------------------------");
            ros::spinOnce();
        }
        //ROS_INFO("duration:   %f",ros::Time::now().toSec()-tmp.toSec());
        //ROS_INFO("QU:%d",qu);


        //loop_rate.sleep();
    }

    ROS_INFO("---------START HOVER----------------------------------");
    while(ros::ok())
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = planned_p(0);
        pose.pose.position.y = planned_p(1);
        pose.pose.position.z =planned_p(2);

        double theta=150.0/180.0*3.14;
        pose.pose.orientation.w=cos(theta/2);
        pose.pose.orientation.x=-0.0;
        pose.pose.orientation.y=0;
        pose.pose.orientation.z=sin(theta/2);
        //        ROS_INFO("theta:%f",theta);
//        ROS_INFO("W:%f",pose.pose.orientation.w);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
