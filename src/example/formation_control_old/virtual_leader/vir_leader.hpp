/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */

// ros程序必备头文件
#ifndef _VIR_LEADER_HPP_
#define _VIR_LEADER_HPP_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

#include "../fixed_wing_lib/fixed_wing_sub_pub.hpp"
#include "../fixed_wing_lib/mathlib.h"

#define LEADER_HOME_LAT 47.3980795
#define LEADER_HOME_LONG 8.5441407
#define LEADER_HOME_ALT 580

using namespace std;

class VIR_LEADER
{
private:
    ros::NodeHandle nh;

    ros::Time begin_time;

    ros::Publisher vir_leader_pub;

    float current_time;

    float last_time;

    float distance_e;

public:
    _FIXED_WING_SUB_PUB fixed_wing_sub_pub;

    void run(int argc, char **argv);

    void ros_sub_pub();

    void show_vir_leader_status();
};

#endif