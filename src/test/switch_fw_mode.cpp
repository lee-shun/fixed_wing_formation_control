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

/*  ros程序必备头文件 */
#include <iostream>
#include <ros/ros.h>
/* mavros相关头文件 */
#include "fixed_wing_formation_control/Fw_cmd_mode.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

mavros_msgs::State
    current_state; /* 无人机当前状态[包含上锁状态 模式] (从飞控中读取) */
float get_ros_time(ros::Time begin); /* 获取ros当前时间 */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函
 * 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函
 * 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
int main(int argc, char **argv) {
  ros::init(argc, argv, "switch_fw_mode");
  ros::NodeHandle nh;
  ros::Rate rate(5.0);                     /*  频率 [5Hz] */
  ros::Time begin_time = ros::Time::now(); /* 记录启控时间 */
  mavros_msgs::SetMode mode_cmd;           /*  模式容器 */
  fixed_wing_formation_control::Fw_cmd_mode task_cmd_mode; /* 任务模式切换 */

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      "mavros/state", 10, state_cb); /* 【订阅】无人机当前状态 */

  ros::Publisher task_cmd_mode_pub =
      nh.advertise<fixed_wing_formation_control::Fw_cmd_mode>(
          "fixed_wing_formation_control/fw_cmd_mode", 10);

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
      "mavros/set_mode"); /* 【服务】 修改系统模式 */

  int mode_type = 0;    /* 模式类型 */
  int times_out = 100;  /* 次数限制 */
  int counters = 0;     /* 计数器 */
  bool outflag = false; /* 退出标志位 */

  while (ros::ok() && (!outflag)) {
    float current_time = get_ros_time(begin_time); /* 当前时间 */

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>固定翼模式<<<<<<<<<<<<<<<<<<<<<<<<<<"
            "<<<"
         << endl;
    cout << "时刻: " << current_time << " [s] " << endl;
    cout << "模式 : [ " << current_state.mode << " ]" << endl;
    cout << "更换模式：1 offboard, 2 mission, 3 out" << endl;

    cin >> mode_type;
    if (mode_type == 1) {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_formation = true;
      task_cmd_mode_pub.publish(task_cmd_mode);
    } else if (mode_type == 2) {
      mode_cmd.request.custom_mode = "AUTO.MISSION";
    } else if (mode_type == 3) {
      outflag = true;
      break;
    }

    while (
        (mode_cmd.request.custom_mode != current_state.mode) &&
        (counters <=
         times_out)) { /*  如果当前模式与设定模式不一致，切换，（有次数限制） */
      counters++;
      set_mode_client.call(mode_cmd);
    }

    ros::spinOnce(); /*  挂起一段时间，等待切换结果 */
    rate.sleep();

    counters = 0;
  }
  return 0;
}

float get_ros_time(ros::Time begin) {
  ros::Time time_now = ros::Time::now();
  float currTimeSec = time_now.sec - begin.sec;
  float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimeSec + currTimenSec);
}
