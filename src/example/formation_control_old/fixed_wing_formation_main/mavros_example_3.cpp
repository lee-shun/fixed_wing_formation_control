/******************************************************
 * mavros_example_3.cpp                                *
 *                                                     *
 * Author: Qyp                                         *
 *                                                     *
 * Time: 2018.3.25                                     *
 *                                                     *
 * 说明: mavros示例程序3                                 *
 *      1. 给飞控发送位置控制指令                          *
 *      2. 读取飞控回传的信息(位置设定值,姿态设定值)
        3. 切换offboard模式                             *
 *      4. 纯做演示用,无实际用途                          *
 *                                                     *
 ******************************************************/

// ros程序必备头文件
#include <ros/ros.h>

//mavros相关头文件
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

using namespace std;

mavros_msgs::State current_state;                       //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

mavros_msgs::PositionTarget pos_setpoint;               //即将发给无人机的控制指令

mavros_msgs::PositionTarget pos_target;                 //从无人机回传的 vehicle_local_position_setpoint
mavros_msgs::AttitudeTarget att_target;                 //从无人机回传的 vehicle_attitude_setpoint [四元数形式]
sensor_msgs::Imu imu;
float PIX_Euler_target[3];                              //无人机 期望欧拉角(从飞控中读取)
float PIX_Euler[3];                                         //无人机当前欧拉角(从飞控中读取)
float Thrust_target;                                    //期望推力


float get_ros_time(ros::Time begin);                                                 //获取ros当前时间
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    pos_target = *msg;
}

void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    att_target = *msg;

    float q[4];
    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;
    quaternion_2_euler(q, PIX_Euler_target);

    Thrust_target = msg->thrust;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu = *msg;
    float q[4];
    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;

    quaternion_2_euler(q, PIX_Euler);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_example_3");
    ros::NodeHandle nh;

    // 频率 [50Hz]
    ros::Rate rate(30.0);

    // 【订阅】无人机当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 【订阅】无人机imu信息
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, imu_cb);

    // 服务 修改系统模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode mode_cmd;

    // 设定位置速度加速度姿态等指令定义在/home/nuc/mavros/src/mavros/mavros/src/plugins/setpoint_raw.cpp中

    // 【发布】位置控制期望值 对应的mavlink消息类型是 SET_POSITION_TARGET_LOCAL_NED ( #84 )
    ros::Publisher pos_sp_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    // 【订阅】无人机位置期望值[飞控中读取]
    ros::Subscriber pos_target_sub = nh.subscribe<mavros_msgs::PositionTarget>("mavros/setpoint_raw/target_local", 10, pos_target_cb);

    // 【订阅】无人机姿态期望值[飞控中读取]
    ros::Subscriber att_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/target_attitude", 10, att_target_cb);

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

    while(ros::ok())
    {
        // 当前时间
        float current_time = get_ros_time(begin_time);

        //如果当前模式不是OFFBOARD模式, 则切换为OFFBOARD模式
        if(current_state.mode != "OFFBOARD")
        {
           mode_cmd.request.custom_mode = "OFFBOARD";
           set_mode_client.call(mode_cmd);
        }

        //对即将发给飞控的指令进行赋值
        pos_setpoint.type_mask = (2 << 10) | (7 << 6) | (7 << 3);  //100 111 111 000  位置 + yaw
        //pos_setpoint.type_mask = (1 << 10) | (7 << 6) | (7 << 3);  //010 111 111 000  位置 + yaw_rate
        //pos_setpoint.type_mask = (2 << 10) | (7 << 3) | (7 << 0);  // 100 000 111 111   加速度 + yaw
        pos_setpoint.position.x = 10;
        pos_setpoint.position.y = 10;
        pos_setpoint.position.z = 0;
        pos_setpoint.velocity.x = 0;
        pos_setpoint.velocity.y = 0;
        pos_setpoint.velocity.z = 0;
        pos_setpoint.acceleration_or_force.x = 0;
        pos_setpoint.acceleration_or_force.y = 0;
        pos_setpoint.acceleration_or_force.z = 0;
        pos_setpoint.yaw = 0*3.1415/180.0;               //注意 : 单位为弧度
        pos_setpoint.yaw_rate = 0*3.1415/180.0;          //注意 : 单位为弧度
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "pos_setpoint: [X Y Z] : " << " " << pos_setpoint.position.x << " [m] "<< pos_setpoint.position.y<<" [m] "<<pos_setpoint.position.z<<" [m] "<<endl;

        cout << "pos_setpoint: [yaw] : " << " " << pos_setpoint.yaw * 180/3.1415 << " [°] " <<endl;

        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

        cout << "Time: " << current_time <<" [s] "<<endl;

        cout << "Mode : [ " << current_state.mode<<" ]" <<endl;

        cout << "pos_target: [X Y Z] : " << " " << pos_target.position.x << " [m] "<< pos_target.position.y<<" [m] "<< pos_target.position.z<<" [m] "<<endl;

        cout << "Attitude: [roll pitch yaw] " << " " << PIX_Euler[0]*180/3.1415 << " ° "<< PIX_Euler[1]*180/3.1415 <<" ° "<< PIX_Euler[2]*180/3.1415 << " ° " <<endl;

        cout << "Attitude_target: [roll pitch yaw] : " << PIX_Euler_target[0] * 180/3.1415 <<" [°] "<<PIX_Euler_target[1] * 180/3.1415 << " [°] "<< PIX_Euler_target[2] * 180/3.1415<<" [°] "<<endl;

        cout << "Thrust_target[0 - 1] : " << Thrust_target <<endl;

        //发布
        pos_sp_pub.publish(pos_setpoint);
        //回调
        ros::spinOnce();
        //挂起一段时间(rate为 50HZ)
        rate.sleep();
    }
        return 0;

}
// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
}


//获取当前时间 单位：秒
float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
