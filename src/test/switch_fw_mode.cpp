/*
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description: 本程序的作用是将飞控的模式切换为需要的模式 
 */

// ros程序必备头文件
#include <ros/ros.h>
#include <iostream>
//mavros相关头文件
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

using namespace std;

mavros_msgs::State current_state;    //无人机当前状态[包含上锁状态 模式] (从飞控中读取)
float get_ros_time(ros::Time begin); //获取ros当前时间
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "switch_fw_mode");
    ros::NodeHandle nh;
    ros::Rate rate(5.0);                     // 频率 [5Hz]
    ros::Time begin_time = ros::Time::now(); // 记录启控时间
    mavros_msgs::SetMode mode_cmd;           //模式容器

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);     // 【订阅】无人机当前状态
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"); // 【服务】 修改系统模式

    int mode_type = 0;    //模式类型
    int times_out = 100;  //次数限制
    int counters = 0;     //计数器
    bool outflag = false; //退出标志位

    while (ros::ok() && (!outflag))
    {
        float current_time = get_ros_time(begin_time); // 当前时间

        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>固定翼模式<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        cout << "时刻: " << current_time << " [s] " << endl;
        cout << "模式 : [ " << current_state.mode << " ]" << endl;
        cout << "更换模式：1 offboard, 2 mission, 3 out" << endl;

        cin >> mode_type;
        if (mode_type == 1)
        {
            mode_cmd.request.custom_mode = "OFFBOARD";
        }
        else if (mode_type == 2)
        {
            mode_cmd.request.custom_mode = "AUTO.MISSION";
        }
        else if (mode_type == 3)
        {
            outflag = true;
            break;
        }

        while ((mode_cmd.request.custom_mode != current_state.mode) && (counters <= times_out))
        { //如果当前模式与设定模式不一致，切换，（有次数限制）
            counters++;
            set_mode_client.call(mode_cmd);
        }

        ros::spinOnce(); //挂起一段时间，等待切换结果
        rate.sleep();

        if ((counters <= times_out) && (mode_cmd.request.custom_mode == current_state.mode))
        { //成功切换，而且没有问题
            cout << "切换成功" << endl;
        }
        else
        {
            cout << "切换失败" << endl;
        }
        counters = 0;
    }
    return 0;
}

float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
