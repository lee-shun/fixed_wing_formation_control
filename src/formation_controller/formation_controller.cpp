/*本程序的作用是提供几个类型下的编队控制器
例如只有GPS位置，有相对位置以及相对速度，有绝对位置以及绝对速度等
*/

#include "formation_controller.hpp"

void FORMATION_CONTROL::reset_formation_controller() //重置控制器中有“记忆”的量。
{
    rest_speed_pid = true;
    rest_tecs = true;
};

void FORMATION_CONTROL::set_formation_type(int formation_type)
{
    switch (formation_type)
    {
    case 1:
        formation_offset.xb = 0;
        formation_offset.yb = 0;
        formation_offset.zb = 0;

        break;

    case 2:
        formation_offset.xb = -10;
        formation_offset.yb = 0.1;
        formation_offset.zb = 0;

        break;
    }
}

void FORMATION_CONTROL::abs_pos_vel_controller(struct FORMATION_CONTROL::_s_leader_states leader_states,
                                               struct FORMATION_CONTROL::_s_fw_states fw_states)
{

    //测试数据通断
    print_data(&fw_states);

    //1. 根据队形要求，计算出从机期望的在领机机体坐标系下的位置-->GPS位置

    double cos_yaw = cos(leader_states.yaw_angle); //规定，运算全部用弧度，输出时考虑角度
    double sin_yaw = sin(leader_states.yaw_angle);

    formation_offset.ned_n = cos_yaw * formation_offset.xb + (-sin_yaw * formation_offset.yb);
    formation_offset.ned_e = sin_yaw * formation_offset.xb + cos_yaw * formation_offset.yb;
    formation_offset.ned_d = formation_offset.zb;

    double ref[3], result[3];
    ref[0] = leader_states.latitude;
    ref[1] = leader_states.longtitude;
    ref[2] = leader_states.altitude;
    cov_m_2_lat_long_alt(ref, formation_offset.ned_n, formation_offset.ned_e, formation_offset.ned_d, result);

    fw_sp.latitude = result[0];
    fw_sp.longtitude = result[1];
    fw_sp.altitude = result[2];

    //2. 计算领机的期望位置与当前位置的误差在从机坐标系下的投影

    Point pos_sp(fw_sp.latitude, fw_sp.longtitude),                   //期望位置
        current_pos(fw_states.latitude, fw_states.longtitude),        //当前位置
        fw_ground_speed_2d(fw_states.ned_vel_x, fw_states.ned_vel_y); //当前地速

    Point vector_plane_sp = get_plane_to_sp_vector(current_pos, pos_sp); //计算飞机到期望点向量

    Point fw_ground_speed_2d_unit = fw_ground_speed_2d.normalized();

    fw_error.PXb = vector_plane_sp * fw_ground_speed_2d_unit; //沿速度（机体x）方向距离误差（待检验）
    fw_error.PYb = vector_plane_sp ^ fw_ground_speed_2d_unit; //垂直于速度（机体x）方向距离误差
    fw_error.PZb = fw_sp.altitude - fw_states.altitude;       //高度方向误差

    //3. 计算领机速度与从机速度之差在从机坐标系下的投影

    Point led_ground_speed_2d(leader_states.ned_vel_x, leader_states.ned_vel_y);
    Point led_fol_vel_error = led_ground_speed_2d - fw_ground_speed_2d;

    fw_error.led_fol_vxb = led_fol_vel_error * fw_ground_speed_2d_unit; //沿速度（机体x）方向速度误差（待检验）
    fw_error.led_fol_vyb = led_fol_vel_error ^ fw_ground_speed_2d_unit; //垂直速度（机体x）方向速度误差（待检验）

    //4. 机体前向混合误差，进入PID，计算出期望速度。

    float mix_v_p_Xb = formation_params.kp_p * fw_error.PXb + formation_params.kv_p * fw_error.led_fol_vxb;

    gspeed_pid.init_pid(formation_params.mix_kp, formation_params.mix_ki, formation_params.mix_kd);

    if (rest_speed_pid) //若要重置编队控制器，这个pid得重置一下
    {
        gspeed_pid.reset_pid();
        rest_speed_pid = false;
    }

    bool use_integ = false;
    bool use_diff = false;
    if (abs_num(fw_error.PXb) < 50) //小于50m开始使用积分器，防止积分饱和。小于50m开始使用微分器，加快响应速度
    {
        use_integ = true;
        cout << "use_the_integ" << endl;
    }
    if (abs_num(fw_error.PXb) < 50) //小于50m开始使用积分器，防止积分饱和。小于50m开始使用微分器，加快响应速度
    {
        use_diff = true;
        cout << "use_the_diff" << endl;
    }

    fw_sp.ground_speed = gspeed_pid.pid_anti_saturated(mix_v_p_Xb, use_integ, use_diff);

    //5. 转换期望速度成为期望空速，与期望高度一起进入TECS，产生油门以及俯仰角。

    Point wind_vector(fw_states.wind_estimate_x, fw_states.wind_estimate_y);
    float wind_Xb = wind_vector * fw_ground_speed_2d_unit;
    fw_sp.air_speed = fw_sp.ground_speed - wind_Xb;

    if (rest_tecs)
    {
        _tecs.reset_state();
        rest_tecs = false;
    }
    //设置参数,真实的飞机还需要另外调参
    _tecs.set_speed_weight(tecs_params.speed_weight);
    _tecs.set_time_const_throt(tecs_params.time_const_throt); //这个值影响到总能量-->油门的（相当于Kp，他越大，kp越小）
    _tecs.set_time_const(tecs_params.time_const);             //这个值影响到能量分配-->俯仰角他越大，kp越小
    _tecs.enable_airspeed(true);

    if (fw_sp.altitude - fw_states.altitude >= 5) //判断一下是否要进入爬升

    {
        tecs_params.climboutdem = true;
    }
    else
    {
        tecs_params.climboutdem = false;
    }
    _tecs.update_state(fw_states.altitude, fw_states.air_speed, fw_states.rotmat,
                       fw_states.body_acc, fw_states.ned_acc, fw_states.altitude_lock, fw_states.in_air);

    _tecs.update_pitch_throttle(fw_states.rotmat, fw_states.pitch_angle,
                                fw_states.altitude, fw_sp.altitude, fw_sp.air_speed,
                                fw_states.air_speed, tecs_params.EAS2TAS, tecs_params.climboutdem,
                                tecs_params.climbout_pitch_min_rad, tecs_params.throttle_min,
                                tecs_params.throttle_max, tecs_params.throttle_cruise,
                                tecs_params.pitch_min_rad, tecs_params.pitch_max_rad);

    _tecs.get_tecs_state(tecs_outputs); //这个是tecs控制器状态，可以作为调试的窗口用

    _cmd.pitch = _tecs.get_pitch_demand();
    _cmd.thrust = _tecs.get_throttle_demand();

    //6. 机体侧向混合误差,或者只有位置误差（当前）,由L1控制器解算滚转以及偏航角。

    _lateral_controller.lateral_L1_modified(current_pos, pos_sp, fw_ground_speed_2d, fw_states.air_speed);

    _cmd.roll = constrain(_lateral_controller.nav_roll(),
                          -lateral_controller_params.roll_max, lateral_controller_params.roll_max);
}

struct FORMATION_CONTROL::_s_4cmd FORMATION_CONTROL::get_formation_4cmd()
{
    return _cmd;
}

Point FORMATION_CONTROL::get_plane_to_sp_vector(Point origin, Point target)
{
    /* this is an approximation for small angles, proposed by [2] */

    Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

    return out * double(CONSTANTS_RADIUS_OF_EARTH);
}

void FORMATION_CONTROL::print_data(FORMATION_CONTROL::_s_fw_states *p)
{
    cout << "***************以下是本飞机状态******************" << endl;
    cout << "***************以下是本飞机状态******************" << endl;

    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;
    cout << "飞机当前姿态欧美系【roll，pitch，yaw】" << rad_2_deg(p->roll_angle) << " [deg] "
         << rad_2_deg(p->pitch_angle) << " [deg] "
         << rad_2_deg(p->yaw_angle) << " [deg] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "飞机当前姿态的旋转矩阵【第2行】" << p->rotmat[2][0] << " [] "
         << p->rotmat[2][1] << " [] "
         << p->rotmat[2][2] << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "body下的加速度【XYZ】" << p->body_acc[0] << " [m/ss] " //待完成
         << p->body_acc[1] << " [m/ss] "
         << p->body_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的速度【XYZ】" << p->ned_vel_x << " [m/s] " //待完成
         << p->ned_vel_y << " [m/s] "
         << p->ned_vel_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的加速度【XYZ】(由旋转矩阵得来)" << p->ned_acc[0] << " [m/ss] " //待完成
         << p->ned_acc[1] << " [m/ss] "
         << p->ned_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "GPS位置【lat,long,alt,rel_alt】" << p->latitude << " [] " //待完成
         << p->longtitude << " [] "
         << p->altitude << " [] "
         << p->relative_alt << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "风估计【x,y,z】" << p->wind_estimate_x << " [m/s] " //待完成
         << p->wind_estimate_y << " [m/s] "
         << p->wind_estimate_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "***************以上是本飞机状态******************" << endl;
    cout << "***************以上是本飞机状态******************" << endl;
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
}