/*
 * @Author: your name
 * @Date: 2020-01-09 11:24:53
 * @LastEditTime : 2020-02-10 11:01:42
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fixed_wing_formation_control/src/fw_control_monitor/control_monitor.cpp
 */
/*
control_monitor主要完成的功能是对于飞机飞行状态，连接状态，飞机编队任务执行状态的监控
*/
#include "control_monitor.hpp"

float CONTROL_MONITOR::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void CONTROL_MONITOR::formation_control_states_cb(const fixed_wing_formation_control::Formation_control_states::ConstPtr &msg)
{
    formation_control_states = *msg;
}

void CONTROL_MONITOR::fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    fwstates = *msg;
}

void CONTROL_MONITOR::leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg)
{
    leaderstates = *msg;
}

void CONTROL_MONITOR::fw_4cmd_cb(const fixed_wing_formation_control::FWcmd::ConstPtr &msg)
{
    fw_4cmd = *msg;
}

void CONTROL_MONITOR::ros_sub_pub()
{

    fw_states_sub = nh.subscribe ////1.【订阅】固定翼全部状态量
                    <fixed_wing_formation_control::FWstates>("fixed_wing_formation_control/fw_states", 10, &CONTROL_MONITOR::fw_state_cb, this);

    leader_states_sub = nh.subscribe ////2. 【订阅】领机信息
                        <fixed_wing_formation_control::Leaderstates>("fixed_wing_formation_control/leader_states", 10, &CONTROL_MONITOR::leader_states_cb, this);

    fw_cmd_sub = nh.subscribe ////3.【订阅】固定翼四通道控制量
                 <fixed_wing_formation_control::FWcmd>("/fixed_wing_formation_control/fw_cmd", 10, &CONTROL_MONITOR::fw_4cmd_cb, this);

    formation_control_states_sub = nh.subscribe ////4.【订阅】编队控制状态
                                   <fixed_wing_formation_control::Formation_control_states>("/fixed_wing_formation_control/formation_control_states", 10, &CONTROL_MONITOR::formation_control_states_cb, this);

    fwmonitor_pub = nh.advertise ////5.【发布】监控节点飞机以及任务状态
                    <fixed_wing_formation_control::Fwmonitor>("fixed_wing_formation_control/fwmonitor_flags", 10);
}

void CONTROL_MONITOR::run()
{
    ros::Rate rate(50.0);
    begin_time = ros::Time::now(); // 记录启控时间
    ros_sub_pub();
    cout << "current_time::" << current_time << "\t"
         << "in_the_monitor_run" << endl;
    while (ros::ok())
    {
        current_time = get_ros_time(begin_time);

        if ((abs_num(formation_control_states.err_PXb) < monitor_params.STAND_POS_XB) &&
            (abs_num(formation_control_states.err_PYb) < monitor_params.STAND_POS_YB) &&
            (abs_num(formation_control_states.err_PZb) < monitor_params.STAND_POS_ZB))
        {
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "control_monitor");
    CONTROL_MONITOR _ctrl_monitor;
    if (true)
    {
        _ctrl_monitor.run();
    }
    return 0;
}