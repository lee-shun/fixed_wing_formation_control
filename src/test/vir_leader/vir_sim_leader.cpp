/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 *  本程序是虚拟领机，领机获取的是仿真环境的数据，速度以及航线不是固定不变的
 *  订阅打包程序的飞机状态，打包成虚拟领机的状态发布
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors  : lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime : 2020-02-13 10:25:36
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "vir_sim_leader.hpp"

void VIR_SIM_LEADER::fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    fwstates = *msg;
}

void VIR_SIM_LEADER::ros_sub_pub()
{
    vir_leader_pub = nh.advertise<fixed_wing_formation_control::Leaderstates>("fixed_wing_formation_control/leader_states", 10);
    fw_states_sub = nh.subscribe<fixed_wing_formation_control::FWstates>("fixed_wing_formation_control/fw_states", 10, &VIR_SIM_LEADER::fw_state_cb, this);
}

void VIR_SIM_LEADER::run(int argc, char **argv)
{
    ros::Rate rate(10.0);
    begin_time = get_sys_time(); // 记录启控时间
    ros_sub_pub();               //继承而来，只是发布消息

    while (ros::ok())
    {

        leaderstates.airspeed = fwstates.air_speed;
        leaderstates.altitude = fwstates.altitude;
        leaderstates.latitude = fwstates.latitude;
        leaderstates.longtitude = fwstates.longtitude;
        leaderstates.ned_vel_x = fwstates.ned_vel_x;
        leaderstates.ned_vel_y = fwstates.ned_vel_y;
        leaderstates.ned_vel_z = fwstates.ned_vel_z;

        vir_leader_pub.publish(leaderstates);

        ros::spinOnce(); //挂起一段时间，保证周期的速度
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vir_dir_leader");

    VIR_SIM_LEADER _vir_leader;

    if (true)
    {
        _vir_leader.run(argc, argv);
    }

    return 0;
}