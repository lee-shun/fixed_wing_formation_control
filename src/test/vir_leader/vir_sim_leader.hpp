/*
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * 本程序是虚拟领机，领机获取的是仿真环境的数据，速度以及航线不是固定不变的
 * 订阅打包程序的飞机状态，打包成虚拟领机的状态发布。
 */

#ifndef VIR_SIM_LEADER_HPP
#define VIR_SIM_LEADER_HPP

#include <ros/ros.h>
#include <fixed_wing_formation_control/FWstates.h>
#include <fixed_wing_formation_control/Leaderstates.h>
#include "../../fixed_wing_lib/syslib.hpp"
class VIR_SIM_LEADER
{
public:
    void run(int argc, char **argv);

private:
    ros::NodeHandle nh;
    ros::Publisher vir_leader_pub;
    ros::Subscriber fw_states_sub;                           // 【订阅】固定翼全部状态量
    fixed_wing_formation_control::Leaderstates leaderstates; //即将要发布的领机的状态
    fixed_wing_formation_control::FWstates fwstates;         //自定义--飞机打包的全部状态

    void ros_sub_pub();
    void show_vir_leader_status();
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg);

    long begin_time;
    float current_time;
    float last_time;
};

#endif
