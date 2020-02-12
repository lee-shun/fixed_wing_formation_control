/*
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * 本程序实现的飞机任务状态，飞行状态，控制状态的全监控并发送消息以及记录log
 */

#ifndef _CONTROL_MONITOR_HPP_
#define _CONTROL_MONITOR_HPP_

#include <ros/ros.h>
#include <iostream>
#include "../fixed_wing_lib/syslib.hpp"
#include "../fixed_wing_lib/mathlib.hpp"
#include "fixed_wing_formation_control/FWstates.h"
#include "fixed_wing_formation_control/FWcmd.h"
#include "fixed_wing_formation_control/Leaderstates.h"
#include "fixed_wing_formation_control/Formation_control_states.h"
#include "fixed_wing_formation_control/Fwmonitor.h"

using namespace std;
class CONTROL_MONITOR
{
public:
    struct _s_monitor_param
    {
        /**
         * 此处的要求时间，从飞机达标开始计时，如果中间断开，则需要重新开始计时
        */
        float STAND_TIME{20.0}; //编队控制任务要求时间<s>
        /**
         * 此处要求的距离，在标准点附近都是可以的，因而在领机的圆柱体结构内都是可以的
        */
        float STAND_POS_XB{5.0}; //编队控制任务要求机体前向分量距离<m>
        float STAND_POS_YB{1.0}; //编队控制任务要求机体横向分量距离<m>
        float STAND_POS_ZB{1.0}; //编队控制任务要求机体高度分量距离<m>

        float form_start_time{0}; //编队控制任务达标时刻
    };
    void run();

private:
    int planeID{2}; //飞机编号

    ros::NodeHandle nh;                  //ros句柄
    ros::Time begin_time;                //监控程序开始时间
    float current_time;                  //当前时间
    float get_ros_time(ros::Time begin); //获取当前时间

    ros::Publisher fwmonitor_pub;                 // 【发布】任务状态的flags
    ros::Subscriber fw_states_sub;                // 【订阅】固定翼全部状态量
    ros::Subscriber leader_states_sub;            // 【订阅】领机全部状态量
    ros::Subscriber formation_control_states_sub; // 【订阅】编队控制状态量
    ros::Subscriber fw_cmd_sub;                   // 【订阅】飞机四通道控制量

    fixed_wing_formation_control::Fwmonitor fwmonitor_flag;                          //自定义--任务状态的flags
    fixed_wing_formation_control::FWstates fwstates;                                 //自定义--飞机打包的全部状态
    fixed_wing_formation_control::FWcmd fw_4cmd;                                     //自定义--飞机四通道控制量
    fixed_wing_formation_control::Leaderstates leaderstates;                         //自定义--领机状态
    fixed_wing_formation_control::Formation_control_states formation_control_states; //自定义--编队控制状态量

    void ros_sub_pub();
    void formation_control_states_cb(const fixed_wing_formation_control::Formation_control_states::ConstPtr &msg); //编队状态callback
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg);                                 //本机状态callback
    void leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg);                        //领机状态callback
    void fw_4cmd_cb(const fixed_wing_formation_control::FWcmd::ConstPtr &msg);                                     //本机指令callback

    _s_monitor_param monitor_params; //一些监控常量

    bool is_connected(); //判断飞机是否失联
    bool is_underctrl(); //判断是否失控，或者说进入危险状态
};
#endif