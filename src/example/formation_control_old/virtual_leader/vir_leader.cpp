#include "vir_leader.hpp"

void VIR_LEADER::ros_sub_pub()
{
    vir_leader_pub = nh.advertise<mavros_msgs::Formation_fixed_wing>("/mavros/fixed_wing_formation/status", 10);
}

void VIR_LEADER::show_vir_leader_status()
{
    cout << fixed << setprecision(10) << "fixed_wing_states_tran.latitude = " << fixed_wing_sub_pub.fixed_wing_states_tran.latitude << endl
         << endl
         << endl;

    cout << fixed << setprecision(10) << "fixed_wing_states_tran.longtitude = " << fixed_wing_sub_pub.fixed_wing_states_tran.longtitude << endl
         << endl
         << endl;

    cout << fixed << setprecision(10) << "fixed_wing_states_tran.altitiude = " << fixed_wing_sub_pub.fixed_wing_states_tran.altitude << endl
         << endl
         << endl;

    cout << fixed << setprecision(10) << "fixed_wing_states_tran.ned_vel_y = " << fixed_wing_sub_pub.fixed_wing_states_tran.ned_vel_y << endl
         << endl
         << endl;
}

void VIR_LEADER::run(int argc, char **argv)
{
    ros::Rate rate(2);
    begin_time = ros::Time::now(); // 记录启控时间
    ros_sub_pub();

    fixed_wing_sub_pub.fixed_wing_states_tran.air_speed = 16;

    fixed_wing_sub_pub.fixed_wing_states_tran.latitude = LEADER_HOME_LAT;
    fixed_wing_sub_pub.fixed_wing_states_tran.longtitude = LEADER_HOME_LONG;
    fixed_wing_sub_pub.fixed_wing_states_tran.altitude = LEADER_HOME_ALT;

    fixed_wing_sub_pub.fixed_wing_states_tran.ned_vel_x = 0;

    fixed_wing_sub_pub.fixed_wing_states_tran.ned_vel_z = 0;

    double ref[3];
    double result[3];

    while (ros::ok())
    {

        current_time = fixed_wing_sub_pub.get_ros_time(begin_time);

        distance_e = 5.5;
        cout << "distance====" << distance_e << endl;

        //当前位置作为参考点
        ref[0] = fixed_wing_sub_pub.fixed_wing_states_tran.latitude;
        ref[1] = fixed_wing_sub_pub.fixed_wing_states_tran.longtitude;
        ref[2] = fixed_wing_sub_pub.fixed_wing_states_tran.altitude;

        cov_m_2_lat_long_alt(ref, 0, distance_e, 0, result); //本函数计算的是将偏移量加入之后的最终的坐标值

        fixed_wing_sub_pub.fixed_wing_states_tran.latitude = result[0];
        fixed_wing_sub_pub.fixed_wing_states_tran.longtitude = result[1];
        fixed_wing_sub_pub.fixed_wing_states_tran.altitude = result[2];

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
        
        fixed_wing_sub_pub.fixed_wing_states_tran.ned_vel_y = vel_e_cha;
        show_vir_leader_status();

        vir_leader_pub.publish(fixed_wing_sub_pub.fixed_wing_states_tran);

        ros::spinOnce(); //挂起一段时间，保证周期的速度

        rate.sleep();

        last_time = current_time;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vir_leader");

    VIR_LEADER _vir_leader;

    if (true)
    {
        _vir_leader.run(argc, argv);
    }

    return 0;
}