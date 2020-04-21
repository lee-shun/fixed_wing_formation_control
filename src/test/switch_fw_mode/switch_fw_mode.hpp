/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * 本程序的作用是将飞控的模式切换为需要的模式 
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-23 14:11:14
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _SWITCH_FW_MODE_HPP_
#define _SWITCH_FW_MODE_HPP_

/*  ros程序必备头文件 */
#include <iostream>
#include <ros/ros.h>
/* mavros相关头文件 */
#include "fixed_wing_formation_control/Fw_cmd_mode.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "../../fixed_wing_lib/syslib.hpp"

using namespace std;
class SWITCH_FW_MODE
{
    public:
      void set_planeID(int id);
      void run(int argc, char **argv);

    private:
        int planeID{1};

        string uavID{"uav1/"};

      /* 无人机当前状态[包含上锁状态 模式] (从飞控中读取) */
      mavros_msgs::State current_state;

      float get_ros_time(ros::Time begin);

      void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

      ros::NodeHandle nh;
      ros::Time begin_time = ros::Time::now(); /* 记录启控时间 */
      mavros_msgs::SetMode mode_cmd;           /*  模式容器 */
      fixed_wing_formation_control::Fw_cmd_mode task_cmd_mode; /* 任务模式切换 */

      int mode_type = 0;    /* 模式类型 */
      int times_out = 100;  /* 次数限制 */
      int counters = 0;     /* 计数器 */
      bool outflag = false; /* 退出标志位 */
};
#endif
