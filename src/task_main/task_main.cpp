/*
比赛任务主程序
*/
#include <ros/ros.h>
#include <iostream>
#include "fixed_wing_formation_control/FWstates.h"
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_main");

    cout << "HELLO,SKY!!!" << endl;

    return 0;
}