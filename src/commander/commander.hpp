/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-19 11:28:44
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-23 11:17:17
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * 本程序是一个状态机，管理飞机比赛任务切换问题，订阅来自监
 * 控节点的监控消息，获取飞机的四个阶段的执行状况；根据飞机
 * 当前的状态以及任务的需要，产生飞机期望模式msg，交由
 * task_main使用
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _COMMANDER_HPP_
#define _COMMANDER_HPP_

#include <iostream>
#include <ros/ros.h>

#include "fixed_wing_formation_control/Fw_current_mode.h" /*飞机当前任务模式*/
#include "fixed_wing_formation_control/Fwmonitor.h"
#include "fixed_wing_formation_control/Fw_cmd_mode.h"

using namespace std;

#define COMMANDER_INFO(a) cout << "[COMMANDER_INFO]:" << a << endl;
class COMMANDER
{

private:
    int planeID{2}; /*飞机编号*/

    /**
     * ros初始化
    */

    ros::NodeHandle nh;                  /*ros句柄*/
    ros::Time begin_time;                /*监控程序开始时间*/
    float current_time;                  /*当前时间*/
    float get_ros_time(ros::Time begin); /*获取当前时间*/

    /**
     * ros订阅发布
    */

    ros::Subscriber fwmonitor_sub;       /* 【订阅】任务状态的flags订阅*/
    ros::Subscriber fw_current_mode_sub; /*【订阅】飞机当前模式订阅*/
    ros::Publisher fw_cmd_mode_pub;      /*【发布】飞机期望模式发布*/

    fixed_wing_formation_control::Fw_current_mode fw_current_mode; /*自定义--当前模式容器*/
    fixed_wing_formation_control::Fwmonitor fw_monitor_flags;      /*自定义--当前监控容器*/
    fixed_wing_formation_control::Fw_cmd_mode fw_cmd_mode;         /*自定义--期望模式容器*/

    /**
     * ros消息函数
    */

    void ros_sub_pub();
    void fw_current_mode_cb(const fixed_wing_formation_control::Fw_current_mode::ConstPtr &msg);
    void fw_monitor_flags_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg);

    /**
     * 模式判断函数
    */

    void in_idel_mode();      /*空闲模式状态机*/
    void in_takeoff_mode();   /*起飞模式状态机*/
    void in_formation_mode(); /*编队模式状态机*/
    void in_land_mode();      /*降落模式状态机*/
    void in_protect_mode();   /*保护模式状态机*/

public:
    void run();
};

#endif
