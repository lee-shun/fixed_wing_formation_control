/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 *  本程序的作用是提供几个类型下的编队控制器
 *  例如只有GPS位置，有相对位置以及相对速度，有绝对位置以及绝对速度等
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-24 18:32:49
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "formation_controller.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description: 更新飞机，领机飞行状态
 */
void FORMATION_CONTROLLER::update_led_fol_states(const struct _s_leader_states *leaderstates,
                                                 const struct _s_fw_states *thisfw_states)
{ /* 使用指针，避免内存浪费 */
    leader_states = *leaderstates;
    fw_states = *thisfw_states;
}

/**
 * @Input: void
 * @Output: void
 * @Description:以便重置控制器中有“记忆”的量
 */
void FORMATION_CONTROLLER::reset_formation_controller()
{
    rest_speed_pid = true;
    rest_tecs = true;
};

/**
 * @Input: void
 * @Output: void
 * @Description: 设定编队期望队形
 */
void FORMATION_CONTROLLER::set_formation_type(int formation_type)
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

/**
 * @Input: void
 * @Output: void
 * @Description: 设定编队控制器内部飞机模型函数，例如最大滚转角速度等
 */
void FORMATION_CONTROLLER::set_fw_model_params(struct _s_fw_model_params &input_params)
{
    fw_params = input_params;
}

void FORMATION_CONTROLLER::set_formation_params(struct _s_formation_params &input_params)
{
    formation_params = input_params;
}

void FORMATION_CONTROLLER::set_tecs_params(struct _s_tecs_params &input_params)
{
    tecs_params = input_params;
}

void FORMATION_CONTROLLER::set_lateral_ctrller_params(struct _s_lateral_controller_params &input_params)
{
    lateral_controller_params = input_params;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 是否使用计算获得期望地速，还是直接给满最大速度
 */
bool FORMATION_CONTROLLER::use_speed_sp_cal()
{
    if (abs_num(fw_error.PXb) <= 50.0) /* 当飞机超过领机50米以内，或者落后从机50m以内的时候，也得启用tecs速度高度控制 */
    {
        return true;
    }
    else
    { /* 从机的距离误差在50m以上，直接使用飞机的最大速度追踪！ */
        return false;
    }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 调用滤波器对输入的飞机原始状态进行滤波
 */
void FORMATION_CONTROLLER::filter_led_fol_states()
{
    fw_states_filtered = fw_states;
    leader_states_filtered = leader_states;

    if (use_the_filter)
    {
        leader_states_filtered.global_vel_x =
            led_gol_vel_x_filter.one_order_filter(leader_states.global_vel_x);

        leader_states_filtered.global_vel_y =
            led_gol_vel_y_filter.one_order_filter(leader_states.global_vel_y);
    }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 领机绝对位置以及绝对速度GPS控制器，以机体坐标系为准，误差直接投影到从机机体坐标系之中，并在机体系之中产生控制量
 */
void FORMATION_CONTROLLER::abs_pos_vel_controller()
{

    static long now = get_sys_time();
    _dt = constrain((now - abs_pos_vel_ctrl_timestamp) * 1.0e-3f, _dtMin, _dtMax);

    /**
    * 0. 原始数据滤波,从此之后，飞机的状态保存在了滤波后的状态值里面
    */

    filter_led_fol_states();

    if (!identify_led_fol_states())
    {
        cout << "警告：领机或从机未在飞行之中，无法执行编队控制器" << endl;
        return;
    }

    /**
    * 1. 根据队形要求，计算出从机期望的在领机机体坐标系下的位置，然后再到->GPS位置（期望经纬高）；
    * 在此之中注意领机机体方向的选择。
    *   a. 计算领机机头方向
    *   b. 计算从机期望位置的gps位置
    */

    static float led_airspd_x = leader_states_filtered.wind_estimate_x + leader_states_filtered.global_vel_x; /* 此处的空速算法有待验证 */
    static float led_airspd_y = leader_states_filtered.wind_estimate_y + leader_states_filtered.global_vel_y;

    led_arispd.set_vec_ele(led_airspd_x, led_airspd_y);                                                  /* 领机空速向量 */
    led_gspeed_2d.set_vec_ele(leader_states_filtered.global_vel_x, leader_states_filtered.global_vel_y); /* 领机地速向量 */

    cout << "领机状态" << endl;
    cout << "验证用，滤波后的地速x大小为：" << leader_states_filtered.global_vel_x << endl;
    cout << "验证用，滤波后的地速y大小为：" << leader_states_filtered.global_vel_y << endl;
    cout << "验证用，计算的空速大小为：" << led_arispd.len() << endl;
    cout << "验证用，实际获取空速为：" << leader_states_filtered.air_speed << endl;
    cout << "验证用，计算的空速大小以及实际获取空速之差为：" << (led_arispd.len() - leader_states_filtered.air_speed) << endl;

    if ((led_arispd.len() - leader_states_filtered.air_speed) >= 3.0) /* 计算获得的空速与读取的空速差距较大 */
    {
        led_airspd_states_valid = false; /* 此时选用地速方向 */
    }
    else
    {
        led_airspd_states_valid = true;
    }

    if ((leader_states_filtered.yaw_valid)) /* 将认为机头实际指向 */
    {
        led_cos_yaw = cosf(leader_states_filtered.yaw_angle);
        led_sin_yaw = sinf(leader_states_filtered.yaw_angle);
    }
    else if ((!leader_states_filtered.yaw_valid) && /* 将认为空速方向与机头方向一致 */
             (led_arispd.len2() >= 3.0) &&          /* 速度太小也不行，空速方向十分不稳定 */
             (led_airspd_states_valid))
    {

        led_cos_yaw = led_arispd.x / led_arispd.len();
        led_sin_yaw = led_arispd.y / led_arispd.len();
    }
    else if (((!leader_states_filtered.yaw_valid) && (led_gspeed_2d.len2() >= 3.0) && (!led_airspd_states_valid)) /* 将认为地速方向与机头方向一致 */
             || ((!leader_states_filtered.yaw_valid) && (led_gspeed_2d.len2() >= 3.0) && (led_arispd.len2() < 3.0)))
    {
        led_cos_yaw = led_gspeed_2d.x / led_gspeed_2d.len();
        led_sin_yaw = led_gspeed_2d.y / led_gspeed_2d.len();
    }
    else /* 三种可用情况之外 */
    {
        led_cos_yaw = 0;
        led_sin_yaw = 0;
        cout << "警告：无法计算领机机头朝向，请检查输入信息是否有误" << endl;
        return;
    }

    formation_offset.ned_n = led_cos_yaw * formation_offset.xb + (-led_sin_yaw * formation_offset.yb);
    formation_offset.ned_e = led_sin_yaw * formation_offset.xb + led_cos_yaw * formation_offset.yb;
    formation_offset.ned_d = formation_offset.zb;

    static double ref[3], result[3];
    ref[0] = leader_states_filtered.latitude;
    ref[1] = leader_states_filtered.longitude;
    ref[2] = leader_states_filtered.altitude;

    cov_m_2_lat_long_alt(ref, formation_offset.ned_n, formation_offset.ned_e, formation_offset.ned_d, result);

    fw_sp.latitude = result[0];
    fw_sp.longitude = result[1];
    fw_sp.altitude = result[2];

    /**
    * 2. 计算从机机体坐标系，优先根据yaw角度，其次根据“空速方向==机体方向”，即：
    * 飞机侧滑角为0，最后根据飞机的地速方向，认为空速方向与地速方向一致
    */

    static float fw_airspd_x = fw_states_filtered.wind_estimate_x + fw_states_filtered.global_vel_x;
    static float fw_airspd_y = fw_states_filtered.wind_estimate_y + fw_states_filtered.global_vel_y;

    fw_arispd.set_vec_ele(fw_airspd_x, fw_airspd_y);                                            /* 本机空速向量 */
    fw_gspeed_2d.set_vec_ele(fw_states_filtered.global_vel_x, fw_states_filtered.global_vel_y); /* 本机地速向量 */

    cout << "本机机状态" << endl;
    cout << "验证用，滤波后的地速x大小为：" << fw_states_filtered.global_vel_x << endl;
    cout << "验证用，滤波后的地速y大小为：" << fw_states_filtered.global_vel_y << endl;
    cout << "验证用，计算的空速大小为：" << fw_arispd.len() << endl;
    cout << "验证用，实际获取空速为：" << fw_states_filtered.air_speed << endl;
    cout << "验证用，计算的空速大小以及实际获取空速之差为：" << (fw_arispd.len() - fw_states_filtered.air_speed) << endl;

    if ((fw_arispd.len() - fw_states_filtered.air_speed) >= 3.0) /* 计算获得的空速与读取的空速差距较大 */
    {
        fw_airspd_states_valid = false; /* 此时选用地速方向 */
    }
    else
    {
        fw_airspd_states_valid = true;
    }

    if (fw_states_filtered.yaw_valid) /* 将认为机头实际指向 */
    {
        fw_cos_yaw = cosf(fw_states_filtered.yaw_angle);
        fw_sin_yaw = sinf(fw_states_filtered.yaw_angle);
    }
    else if ((!fw_states_filtered.yaw_valid) && /* 将认为空速方向与机头方向一致 */
             (fw_arispd.len2() >= 3.0) &&
             fw_airspd_states_valid)
    {

        fw_cos_yaw = fw_arispd.x / fw_arispd.len();
        fw_sin_yaw = fw_arispd.y / fw_arispd.len();
    }
    else if (((!fw_states_filtered.yaw_valid) && (fw_arispd.len2() < 3.0) && (fw_gspeed_2d.len2() >= 3.0)) || /* 将认为地速方向与机头方向一致 */
             ((!fw_states_filtered.yaw_valid) && (!fw_airspd_states_valid) && (fw_gspeed_2d.len2() >= 3.0)))
    {
        fw_cos_yaw = fw_gspeed_2d.x / fw_gspeed_2d.len();
        fw_sin_yaw = fw_gspeed_2d.y / fw_gspeed_2d.len();
    }
    else /* 三种可用情况之外 */
    {
        fw_cos_yaw = 0;
        fw_sin_yaw = 0;
        FORMATION_CONTROLLER_INFO("警告：无法计算从机机头朝向，请检查输入信息是否有误");
        return;
    }

    static Vec fw_body_unit(fw_cos_yaw, fw_sin_yaw);
    fw_body_unit = fw_body_unit.normalized(); /* 保证归一化的结果，此向量十分重要 */

    /**
    * 3. 计算从机的期望位置与当前位置的误差在从机坐标系下的投影
    */

    static Point pos_sp(fw_sp.latitude, fw_sp.longitude);                                /* 期望位置 */
    static Point current_pos(fw_states_filtered.latitude, fw_states_filtered.longitude); /* 当前位置 */
    static Vec vector_plane_sp = get_plane_to_sp_vector(current_pos, pos_sp);            /* 计算飞机到期望点向量(本质来说是误差向量) */

    fw_error.PXb = fw_body_unit * vector_plane_sp;               /* 沿速度（机体x）方向距离误差（待检验） */
    fw_error.PYb = fw_body_unit ^ vector_plane_sp;               /* 垂直于速度（机体x）方向距离误差 */
    fw_error.PZb = fw_sp.altitude - fw_states_filtered.altitude; /* 高度方向误差 */

    static double a_pos[2], b_pos[2], m[2]; /* 计算ned坐标系下的位置误差 */
    a_pos[0] = fw_states_filtered.latitude;
    a_pos[1] = fw_states_filtered.longitude;
    b_pos[0] = fw_sp.latitude;
    b_pos[1] = fw_sp.longitude;
    cov_lat_long_2_m(a_pos, b_pos, m);

    /* NED误差记录 */
    fw_error.P_N = m[0];
    fw_error.P_E = m[1];
    fw_error.P_D = fw_sp.altitude - fw_states_filtered.altitude;
    fw_error.P_NE = sqrt((m[0] * m[0] + m[1] * m[1]));

    /**
    * 4. 计算领机从机地速“差”在从机坐标系之中的投影，控制量是地速，所以是地速之差
    */

    static Vec led_fol_vel_error = led_gspeed_2d - fw_gspeed_2d;
    fw_error.led_fol_vxb = fw_body_unit * led_fol_vel_error; /* 沿速度（机体x）方向速度偏差（已检验） */
    fw_error.led_fol_vyb = fw_body_unit ^ led_fol_vel_error; /* 垂直速度（机体x）方向速度偏差（已检验） */

    /**
    * 5. 根据飞机机体前向混合误差产生原始空速期望值，并对此空速先后进行滤波、增量限幅、最终限幅约束
    * 得到最终期望空速。
    * 机体前向误差分类，超过一定误差，最大空速直接给满，迅速减小误差。小于一定误差，按照控制逻辑正常产生
    */

    static float mix_v_p_Xb = formation_params.kp_p * fw_error.PXb + formation_params.kv_p * fw_error.led_fol_vxb;

    gspeed_pid.init_pid(formation_params.mix_kp, formation_params.mix_ki, formation_params.mix_kd);

    if (rest_speed_pid) /* 若要重置编队控制器，这个pid得重置一下 */
    {
        gspeed_pid.reset_pid();
        rest_speed_pid = false;
    }

    static bool use_integ = false;
    static bool use_diff = false;
    if (abs_num(fw_error.PXb) <= 50) /* 小于50m开始使用积分器，防止积分饱和。小于50m开始使用微分器，加快响应速度 */
    {
        use_integ = true;
        cout << "use_the_integ" << endl;
    }
    if (abs_num(fw_error.PXb) <= 50) /* 小于50m开始使用积分器，防止积分饱和。小于50m开始使用微分器，加快响应速度 */
    {
        use_diff = true;
        cout << "use_the_diff" << endl;
    }

    del_fol_gspeed = gspeed_pid.pid_anti_saturated(mix_v_p_Xb, use_integ, use_diff); /* 飞机机头方向期望速度增量 */
    fw_sp.ground_speed = del_fol_gspeed + fw_body_unit * led_gspeed_2d;              /* 沿着从机机头方向（飞机空速方向）的期望值 */

    /* 期望地速按照风估计转化为期望空速 */
    fw_wind_vector.set_vec_ele(fw_states_filtered.wind_estimate_x, fw_states_filtered.wind_estimate_y);
    static float wind_Xb = fw_body_unit * fw_wind_vector; /* 沿着飞机机体前向的风估计 */

    airspd_sp = fw_sp.ground_speed + wind_Xb; /* 此处的空速应该是加法 */

    cout << "最原始的空速设定值为" << airspd_sp << endl;

    if (!use_speed_sp_cal()) /* 如果距离误差很大，那么就直接给满 */
    {
        airspd_sp = fw_params.max_arispd_sp;
    }

    /* 符合飞机本身的加速减速特性： */
    /* TODO:慎用，加上之后,需要加大飞机的前向后相加速度，由于延时作用太强，可能导致不稳定。 */
    if ((airspd_sp - airspd_sp_prev) > fw_params.maxinc_acc * _dt)
    {
        airspd_sp = airspd_sp_prev + fw_params.maxinc_acc * _dt;
    }
    else if ((airspd_sp - airspd_sp_prev) < -fw_params.maxdec_acc * _dt)
    {
        airspd_sp = airspd_sp_prev - fw_params.maxdec_acc * _dt;
    }

    airspd_sp = airspd_sp_filter.one_order_filter(airspd_sp); /* 进入一阶低通滤波器 */

    airspd_sp_prev = airspd_sp;

    fw_sp.air_speed = constrain(airspd_sp, fw_params.min_arispd_sp, fw_params.max_arispd_sp);

    /**
    * 6.期望GPS高度以及期望空速进入TECS得到油门以及俯仰角
    */

    if (rest_tecs)
    {
        _tecs.reset_state();
        rest_tecs = false;
    }
    /* 设置参数,真实的飞机还需要另外调参 */
    _tecs.set_speed_weight(tecs_params.speed_weight);
    _tecs.set_time_const_throt(tecs_params.time_const_throt); /* 这个值影响到总能量-->油门的（相当于Kp，他越大，kp越小） */
    _tecs.set_time_const(tecs_params.time_const);             /* 这个值影响到能量分配-->俯仰角他越大，kp越小 */
    _tecs.enable_airspeed(true);

    if (fw_sp.altitude - fw_states.altitude >= 10) /* 判断一下是否要进入爬升 */

    {
        tecs_params.climboutdem = true;
    }
    else
    {
        tecs_params.climboutdem = false;
    }

    _tecs.update_vehicle_state_estimates(fw_states_filtered.air_speed, fw_states_filtered.rotmat, fw_states_filtered.body_acc,
                                         fw_states_filtered.altitude_lock, fw_states_filtered.in_air, fw_states_filtered.altitude,
                                         vz_valid, fw_states_filtered.ned_vel_z, fw_states_filtered.body_acc[2]);

    _tecs.update_pitch_throttle(fw_states_filtered.rotmat, fw_states_filtered.pitch_angle,
                                fw_states_filtered.altitude, fw_sp.altitude, fw_sp.air_speed,
                                fw_states_filtered.air_speed, tecs_params.EAS2TAS, tecs_params.climboutdem,
                                tecs_params.climbout_pitch_min_rad, fw_params.throttle_min,
                                fw_params.throttle_max, fw_params.throttle_cruise,
                                fw_params.pitch_min_rad, fw_params.pitch_max_rad);

    _cmd.pitch = _tecs.get_pitch_setpoint();
    _cmd.thrust = _tecs.get_throttle_setpoint();

    /**
    * 7.根据飞机机体侧向混合误差，进入横侧向控制器，产生原始期望滚转角。
    */

    /* _lateral_controller.lateral_L1_modified(current_pos, pos_sp, fw_gspeed_2d, fw_states_filtered.air_speed); */

    _lateral_controller.mix_pos_vel_ctrl(fw_gspeed_2d, fw_body_unit, vector_plane_sp, led_fol_vel_error);

    roll_cmd = _lateral_controller.nav_roll(); /* 获取期望控制滚转 */

    /**
    * 8.原始期望滚转角平滑滤波，限幅，角速度限幅，
    */

    roll_cmd = roll_cmd_filter.one_order_filter(roll_cmd);

    if ((roll_cmd - roll_cmd_prev) > fw_params.roll_rate_max * _dt)
    {
        roll_cmd = roll_cmd_prev + fw_params.roll_rate_max * _dt;
    }
    else if ((roll_cmd - roll_cmd_prev) < -fw_params.roll_rate_max * _dt)
    {
        roll_cmd = roll_cmd_prev - fw_params.roll_rate_max * _dt;
    }

    roll_cmd = constrain(roll_cmd, -fw_params.roll_max, fw_params.roll_max);

    _cmd.roll = roll_cmd;

    roll_cmd_prev = roll_cmd;

    /**
    * 9. 下一次计算准备，以及其他工作
    */

    abs_pos_vel_ctrl_timestamp = now;
}

/**
 * @Input: void
 * @Output: bool
 * @Description: 判断飞机传入的状态值是否有问题，是否在飞行之中
 */
bool FORMATION_CONTROLLER::identify_led_fol_states()
{
    if ((leader_states_filtered.global_vel_x > 3.0) ||
        (leader_states_filtered.global_vel_y > 3.0) ||
        (leader_states_filtered.relative_alt > 3.0))
    {
        led_in_fly = true;
    }
    else
    {
        led_in_fly = false;

        FORMATION_CONTROLLER_INFO("警告：领机未在飞行之中");
    }

    if ((fw_states_filtered.global_vel_x > 3.0) ||
        (fw_states_filtered.global_vel_y > 3.0) ||
        (fw_states_filtered.relative_alt > 3.0))
    {
        fol_in_fly = true;
    }
    else
    {
        fol_in_fly = false;

        FORMATION_CONTROLLER_INFO("警告：本机未在飞行之中");
    }

    if (led_in_fly && fol_in_fly)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 计算从飞机当前位置到期望的位置的向量
 */
Point FORMATION_CONTROLLER::get_plane_to_sp_vector(Point origin, Point target)
{
    Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

    return out * double(CONSTANTS_RADIUS_OF_EARTH);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 获得飞机期望的四通道控制量
 */
void FORMATION_CONTROLLER::get_formation_4cmd(struct _s_4cmd &fw_cmd)
{
    fw_cmd = _cmd;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 得到编队中本机的运动学期望值
 */
void FORMATION_CONTROLLER::get_formation_sp(struct _s_fw_sp &formation_sp)
{
    formation_sp = fw_sp;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 得到编队控制误差
 */
void FORMATION_CONTROLLER::get_formation_error(struct _s_fw_error &formation_error)
{
    formation_error = fw_error;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 获得编队控制器参数
 */
void FORMATION_CONTROLLER::get_formation_params(struct _s_formation_params &format_params)
{
    format_params = formation_params;
}
