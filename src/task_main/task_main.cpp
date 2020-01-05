/*
比赛任务主程序
*/
#include <ros/ros.h>
#include <iostream>
#include "../fixed_wing_lib/syslib.hpp"
#include "fixed_wing_formation_control/FWstates.h"
#include "../formation_controller/formation_controller.hpp"

using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "task_main");
    ros::NodeHandle nh;
    // 频率 [50Hz]
    ros::Rate rate(50.0);

    double time_now = get_sys_time();
    double last_time = get_sys_time();

    while (ros::ok())
    {
        time_now = get_sys_time();

        cout << fixed << setprecision(4)<< "DT===" << (time_now - last_time)/1000 << endl;

        last_time = time_now;

        ros::spinOnce();
        //挂起一段时间(rate为 50HZ)
        rate.sleep();
    }

    return 0;
}