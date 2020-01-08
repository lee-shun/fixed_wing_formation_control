/*
比赛任务主程序
*/
#include <ros/ros.h>
#include <iostream>
#include "../fixed_wing_lib/syslib.hpp"
#include "fixed_wing_formation_control/FWstates.h"
#include "fixed_wing_formation_control/FWcmd.h"
#include "../formation_controller/formation_controller.hpp"
#include "../pack_fw_states/fixed_wing_sub_pub.hpp"

using namespace std;
class TASK_MAIN
{
private:
    ros::NodeHandle nh; //ros句柄
    ros::Time begin_time;
    float current_time;
    float get_ros_time(ros::Time begin); //获取当前时间

    _FIXED_WING_SUB_PUB fixed_wing_sub_pub;                                        //ros消息收发的中间对象
    fixed_wing_formation_control::FWstates fwstates;                               //自定义--飞机打包的全部状态
    fixed_wing_formation_control::FWcmd fw_4cmd;                                   //自定义--飞机四通道控制量
    ros::Subscriber fw_states_sub;                                                 // 【订阅】固定翼全部状态量
    ros::Publisher fw_cmd_pub;                                                     //发布飞机四通道控制量
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg); //
    void ros_sub_pub();                                                            //订阅领机从机的信息

    void control_formation(); //编队控制主函数
    bool need_control_formation{true};
    FORMATION_CONTROL formation_controller;                                               //编队控制器
    string fw_col_mode_current{"MANUAL"};                                                 //当前模式
    string fw_col_mode_last{"MANUAL"};                                                    //上一时刻模式
    struct FORMATION_CONTROL::_s_leader_states leader_states;                             //领机信息
    struct FORMATION_CONTROL::_s_fw_states thisfw_states;                                 //本机信息
    struct FORMATION_CONTROL::_s_4cmd formation_cmd;                                      //四通道控制量
    struct FORMATION_CONTROL::_s_formation_controller_states formation_controller_states; //编队控制器状态

public:
    void run();
};