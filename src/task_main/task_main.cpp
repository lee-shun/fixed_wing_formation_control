/*
比赛任务主程序
*/
#include <ros/ros.h>
#include <iostream>
#include "fixed_wing_formation_control/FWstates.h"
#include "../formation_controller/formation_controller.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_main");
    FORMATION_CONTROL test;
    while(ros::ok())
    {
        
        cout<<"time==="<<test.get_sys_time()<<endl;
    }

    return 0;
}