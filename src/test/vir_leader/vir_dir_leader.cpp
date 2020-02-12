/*
 * @Author: lee-shun 
 * @Date: 2020-02-12 22:43:48 
 * @Last Modified by:   lee-shun 
 * @Last Modified time: 2020-02-12 22:43:48 
 */
/**
 * 本程序是虚拟领机，领机从西向东按照一定的速度飞行
*/
#include "vir_dir_leader.hpp"

void VIR_DIR_LEADER::ros_sub_pub()
{
    vir_leader_pub = nh.advertise<fixed_wing_formation_control::Leaderstates>("fixed_wing_formation_control/leader_states", 10);
}

void VIR_DIR_LEADER::show_vir_leader_status()
{
    cout << fixed << setprecision(10) << "leaderstates.latitude = " << leaderstates.latitude << endl
         << endl
         << endl;

    cout << fixed << setprecision(10) << "leaderstates.longtitude = " << leaderstates.longtitude << endl
         << endl
         << endl;

    cout << fixed << setprecision(10) << "leaderstates.altitiude = " << leaderstates.altitude << endl
         << endl
         << endl;

    cout << fixed << setprecision(10) << "leaderstates.ned_vel_y = " << leaderstates.ned_vel_y << endl
         << endl
         << endl;
}

void VIR_DIR_LEADER::run(int argc, char **argv)
{
    ros::Rate rate(20.0);
    begin_time = get_sys_time(); // 记录启控时间
    ros_sub_pub();

    leaderstates.latitude = LEADER_HOME_LAT;
    leaderstates.longtitude = LEADER_HOME_LONG;
    leaderstates.altitude = LEADER_HOME_ALT;

    leaderstates.global_vel_x = 0;
    leaderstates.global_vel_z = 0;
    distance_e = 0.75;

    double ref[3];
    double result[3];

    while (ros::ok())
    {

        current_time = get_time_from_begin(begin_time) * 1e-3;

        cout << "distance====" << distance_e << endl;

        //当前位置作为参考点
        ref[0] = leaderstates.latitude;
        ref[1] = leaderstates.longtitude;
        ref[2] = leaderstates.altitude;

        cov_m_2_lat_long_alt(ref, 0, distance_e, 0, result); //本函数计算的是将偏移量加入之后的最终的坐标值

        leaderstates.latitude = result[0];
        leaderstates.longtitude = result[1];
        leaderstates.altitude = result[2];

        double ref1[2], result1[2], m[2];

        ref1[0] = ref[0];
        ref1[1] = ref[1];
        result1[0] = result[0];
        result1[1] = result[1];
        //差分速度
        cov_lat_long_2_m(ref1, result1, m);

        float vel_n_cha = m[0] / (current_time - last_time);
        float vel_e_cha = m[1] / (current_time - last_time);

        cout << "差分速度 n，e==" << vel_n_cha << "m/s"
             << "    " << vel_e_cha << "m/s" << endl;

        leaderstates.global_vel_y = vel_e_cha;

        show_vir_leader_status();
        vir_leader_pub.publish(leaderstates);

        ros::spinOnce(); //挂起一段时间，保证周期的速度
        rate.sleep();
        last_time = current_time;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vir_dir_leader");

    VIR_DIR_LEADER _vir_leader;

    if (true)
    {
        _vir_leader.run(argc, argv);
    }

    return 0;
}