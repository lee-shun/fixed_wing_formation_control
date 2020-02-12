/*
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description: 
 * 本程序的作用是测试TECS控制器的输入输出
 * 输入值是设定好的飞机的状态以及领机的状态，单一值，
 * 只是看输出的计算结果如何 
 */

// ros程序必备头文件
#include <ros/ros.h>
#include "../fixed_wing_lib/tecs.hpp"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_TECS");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);

    TECS _tecs_controller;

    while (ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}