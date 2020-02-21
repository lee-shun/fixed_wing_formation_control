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
 * @LastEditTime: 2020-02-21 15:30:12
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * 本程序是一个状态机，管理飞机比赛任务切换问题，订阅来自监
 * 控节点的监控消息，获取飞机的四个阶段的执行状况；根据飞机
 * 当前的状态以及任务的需要，产生飞机期望模式msg，交由
 * task_main使用
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "commander.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description: 获取当前时刻
 */
float COMMANDER::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void COMMANDER::fw_current_mode_cb(const fixed_wing_formation_control::Fw_current_mode::ConstPtr &msg)
{
    fw_current_mode = *msg;
}

void COMMANDER::fw_monitor_flags_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg)
{
    fw_monitor_flags = *msg;
}

/**
 * @Input: void
 * @Output: void
 * @Description: ros订阅发布声明函数
 */
void COMMANDER::ros_sub_pub()
{
    fw_current_mode_sub = nh.subscribe<fixed_wing_formation_control::Fw_current_mode> //【订阅】当前飞机模式
                          ("fixed_wing_formation_control/fw_current_mode", 10, &COMMANDER::fw_current_mode_cb, this);

    fwmonitor_sub = nh.subscribe<fixed_wing_formation_control::Fwmonitor> //【订阅】任务状态监控器
                    ("fixed_wing_formation_control/fwmonitor_flags", 10, &COMMANDER::fw_monitor_flags_cb, this);

    fw_cmd_mode_pub = nh.advertise //【发布】固定翼期望模式
                      <fixed_wing_formation_control::Fw_cmd_mode>("fixed_wing_formation_control/fw_cmd_mode", 10);
}

/**
 * @Input: void
 * @Output: void
 * @Description: commander主循环
 */
void COMMANDER::run()
{
    ros::Rate rate(20.0);
    begin_time = ros::Time::now(); // 记录启控时间
    ros_sub_pub();

    while (ros::ok())
    {
        current_time = get_ros_time(begin_time);
        cout << "current_time::" << current_time << "\t"
             << "in_the_commander" << endl;

        if (fw_monitor_flags.fw_is_wellctrlled && fw_monitor_flags.fw_is_connected)
        {
            switch (fw_current_mode.mode)
            {
            case (fixed_wing_formation_control::Fw_current_mode::FW_IN_IDEL):
                cout << "FW_IN_IDEL" << endl;
                if (true) //TODO:外部起飞指令激活
                {
                    fw_cmd_mode.need_take_off = true;
                }
                break;
            case (fixed_wing_formation_control::Fw_current_mode::FW_IN_TAKEOFF):
                cout << "FW_IN_TAKEOFF" << endl;
                if (fw_monitor_flags.fw_complete_takeoff)
                {
                    fw_cmd_mode.need_formation = true;
                }
                break;
            case (fixed_wing_formation_control::Fw_current_mode::FW_IN_FORMATION):
                cout << "FW_IN_FORMATION" << endl;
                if (fw_monitor_flags.formation_time_complete && fw_monitor_flags.formation_distance_complete)
                {
                    fw_cmd_mode.need_land = true;
                }
                break;
            case (fixed_wing_formation_control::Fw_current_mode::FW_IN_LANDING):
                cout << "FW_IN_LANDING" << endl;
                if (fw_monitor_flags.fw_complete_landed)
                {
                    fw_cmd_mode.need_idel = true;
                }
                break;
            default:
                cout << "in_commander, the current_mode is not declared. Check the \"task_main current_mode\". " << endl;
                break;
            }
        }
        else //需要进入保护模式
        {
            fw_cmd_mode.need_protected = true;

            cout << "in commander, need protect" << endl;
            if (!fw_monitor_flags.fw_is_wellctrlled)
            {
                cout << "in commander, fw_is_badctrlled" << endl;
            }
            else if (!fw_monitor_flags.fw_is_connected)
            {
                cout << "in commander, fw_is_unconnected" << endl;
            }
        }

        fw_cmd_mode_pub.publish(fw_cmd_mode);

        ros::spinOnce();
        rate.sleep();
    }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "commander");
    COMMANDER _commander;
    if (true)
    {
        _commander.run();
    }
    return 0;
}