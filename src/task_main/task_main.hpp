/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  比赛任务主程序
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime : 2020-02-13 10:23:51
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _TASK_MAIN_HPP_
#define _TASK_MAIN_HPP_

#include <ros/ros.h>
#include <iostream>
#include "../fixed_wing_lib/syslib.hpp"
#include "fixed_wing_formation_control/FWstates.h"
#include "fixed_wing_formation_control/FWcmd.h"
#include "fixed_wing_formation_control/Leaderstates.h"
#include "fixed_wing_formation_control/Formation_control_states.h"
#include "fixed_wing_formation_control/Fwmonitor.h"
#include "../formation_controller/formation_controller.hpp"

using namespace std;
class TASK_MAIN
{
private:
    int planeID{2}; //飞机编号
    //void print_data(const struct _s_fw_states *p); //测试数据通断

    ros::NodeHandle nh; //ros句柄
    ros::Time begin_time;
    float current_time;
    float get_ros_time(ros::Time begin); //获取当前时间

    ros::Subscriber fwmonitor_sub;               // 【订阅】任务状态的flags
    ros::Subscriber fw_states_sub;               // 【订阅】固定翼全部状态量
    ros::Subscriber leader_states_sub;           // 【订阅】领机全部状态量
    ros::Publisher formation_control_states_pub; // 【发布】编队控制状态量
    ros::Publisher fw_cmd_pub;                   // 【发布】飞机四通道控制量

    fixed_wing_formation_control::FWstates fwstates;                                 //自定义--飞机打包的全部状态
    fixed_wing_formation_control::FWcmd fw_4cmd;                                     //自定义--飞机四通道控制量
    fixed_wing_formation_control::Leaderstates leaderstates;                         //自定义--领机状态
    fixed_wing_formation_control::Formation_control_states formation_control_states; //自定义--编队控制状态量
    fixed_wing_formation_control::Fwmonitor fwmonitor_flag;                          //自定义--任务状态的flags

    void ros_sub_pub();                                                                     //ros消息订阅与发布
    void fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg);     //任务状态flagscallback
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg);          //本机状态callback
    void leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg); //领机状态callback

    bool fw_is_ok{true};  //飞机失联、失控以及炸鸡标志位
    bool need_task{true}; //需要进行任务标志位

    bool need_take_off{false}; //起飞标志位

    void control_formation();               //编队控制主函数
    bool need_control_formation{true};      //需要编队控制的标志位
    FORMATION_CONTROL formation_controller; //编队控制器
    string fw_col_mode_current{"MANUAL"};   //当前模式
    string fw_col_mode_last{"MANUAL"};      //上一时刻模式

    struct FORMATION_CONTROL::_s_fw_model_params fw_params;                   //飞机模型参数
    struct FORMATION_CONTROL::_s_formation_params form_ctrller_params;        //编队控制器参数
    struct FORMATION_CONTROL::_s_tecs_params tecs_params;                     //编队控制器内部TECS控制器参数
    struct FORMATION_CONTROL::_s_lateral_controller_params later_ctrl_params; //编队控制器内部横侧向控制器参数
    void input_params();                                                      //将外部的文件之中的参数加载到相应的函数当中去

    struct FORMATION_CONTROL::_s_leader_states leader_states;       //领机信息
    struct FORMATION_CONTROL::_s_fw_states thisfw_states;           //本机信息
    struct FORMATION_CONTROL::_s_4cmd formation_cmd;                //四通道控制量
    struct FORMATION_CONTROL::_s_fw_error formation_error;          //编队误差以及偏差
    struct FORMATION_CONTROL::_s_formation_params formation_params; //编队控制器混合误差产生参数,编队控制器参数
    struct FORMATION_CONTROL::_s_fw_sp formation_sp;                //编队控制运动学期望值
    void formation_states_pub();                                    //发布编队控制器控制状态

    bool need_landing{false}; //需要降落标志位

public:
    void run();
};

#endif