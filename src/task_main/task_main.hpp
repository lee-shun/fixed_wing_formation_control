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
    ros::NodeHandle nh;

    ros::Time begin_time;

    _FIXED_WING_SUB_PUB fixed_wing_sub_pub;

    fixed_wing_formation_control::FWstates fwstates;
    fixed_wing_formation_control::FWcmd fw_4cmd;

    ros::Subscriber fw_states_sub; // 【订阅】固定翼全部状态量
    ros::Publisher fw_cmd_pub;     //发布飞机四通道控制量

    float current_time;

    float get_ros_time(ros::Time begin);

    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg);

public:
    void run();
};