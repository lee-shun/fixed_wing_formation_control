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
 * @LastEditTime : 2020-02-15 21:20:49
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "formation_controller.hpp"

void FORMATION_CONTROL::update_led_fol_states(struct _s_leader_states &leaderstates,
                                              struct _s_fw_states &thisfw_states)
{
    leader_states = leaderstates;
    fw_states = thisfw_states;
}

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

void FORMATION_CONTROL::set_formation_params(struct _s_formation_params &input_params)
{
    formation_params = input_params;
}

void FORMATION_CONTROL::set_tecs_params(struct _s_tecs_params &input_params)
{
    tecs_params = input_params;
}

void FORMATION_CONTROL::set_lateral_ctrller_params(struct _s_lateral_controller_params &input_params)
{
    lateral_controller_params = input_params;
}

bool FORMATION_CONTROL::use_speed_sp_cal()
{
    if (abs_num(fw_error.PXb) <= 50.0) //当飞机超过领机50米以内，或者落后从机50m以内的时候，也得启用tecs速度高度控制
    {
        return true;
    }
    else
    { //从机的距离误差在50m以上，直接使用飞机的最大速度追踪！
        return false;
    }
}

void FORMATION_CONTROL::filter_led_fol_states()
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

void FORMATION_CONTROL::abs_pos_vel_controller()
{
    /*    
    *领机绝对位置以及绝对速度GPS控制器，以从机地速坐标系为准，但是注意最终的控制量一定要转换到机体系之中
    * */
    long now = get_sys_time();
    _dt = constrain((now - abs_pos_vel_ctrl_timestamp) * 1.0e-3f, _dtMin, _dtMax);

    cout << "formation_control_dt == " << _dt << endl;
    //测试数据通断
    print_data(&fw_states);

    //1. 根据队形要求，计算出从机期望的在领机机体坐标系下的位置-->GPS位置

    //没有直接得到领机的yaw的信息，需要计算一下
    if (leader_states_filtered.yaw_valid)
    {
        led_cos_yaw = cos(leader_states_filtered.yaw_angle); //规定，运算全部用弧度，输出时考虑角度
        led_sin_yaw = sin(leader_states_filtered.yaw_angle);
    }
    else
    {
        Vec led_gspeed_2d(leader_states_filtered.global_vel_x, leader_states_filtered.global_vel_y); //领机地速向量
        if (led_gspeed_2d.len2() == 0)                                                               //领机的地速为零，无方向可言
        {
            led_sin_yaw = 0;
            led_cos_yaw = 0;
        }
        else
        {
            led_cos_yaw = led_gspeed_2d.x / led_gspeed_2d.len2();
            led_sin_yaw = led_gspeed_2d.y / led_gspeed_2d.len2();
        }
    }

    formation_offset.ned_n = led_cos_yaw * formation_offset.xb + (-led_sin_yaw * formation_offset.yb);
    formation_offset.ned_e = led_sin_yaw * formation_offset.xb + led_cos_yaw * formation_offset.yb;
    formation_offset.ned_d = formation_offset.zb;

    double ref[3], result[3];
    ref[0] = leader_states_filtered.latitude;
    ref[1] = leader_states_filtered.longitude;
    ref[2] = leader_states_filtered.altitude;

    cov_m_2_lat_long_alt(ref, formation_offset.ned_n, formation_offset.ned_e, formation_offset.ned_d, result);

    fw_sp.latitude = result[0];
    fw_sp.longitude = result[1];
    fw_sp.altitude = result[2];

    //2. 计算从机的期望位置与当前位置的误差在从机坐标系下的投影
    //TODO:当从机的速度很小的时候,空速的方向实际上很是不稳定，
    //此时要完成坐标变换的话，需要使用到本机而机头朝向,需要将这个地方更改一下

    Point pos_sp(fw_sp.latitude, fw_sp.longitude),                                            //期望位置
        current_pos(fw_states_filtered.latitude, fw_states_filtered.longitude),               //当前位置
        fw_ground_speed_2d(fw_states_filtered.global_vel_x, fw_states_filtered.global_vel_y); //当前地速

    Point vector_plane_sp = get_plane_to_sp_vector(current_pos, pos_sp); //计算飞机到期望点向量

    Point fw_ground_speed_2d_unit = fw_ground_speed_2d.normalized();

    fw_error.PXb = fw_ground_speed_2d_unit * vector_plane_sp;    //沿速度（机体x）方向距离误差（待检验）
    fw_error.PYb = fw_ground_speed_2d_unit ^ vector_plane_sp;    //垂直于速度（机体x）方向距离误差
    fw_error.PZb = fw_sp.altitude - fw_states_filtered.altitude; //高度方向误差

    double a_pos[2], b_pos[2], m[2]; //计算ned坐标系下的位置误差
    a_pos[0] = fw_states_filtered.latitude;
    a_pos[1] = fw_states_filtered.longitude;
    b_pos[0] = fw_sp.latitude;
    b_pos[1] = fw_sp.longitude;
    cov_lat_long_2_m(a_pos, b_pos, m);

    fw_error.P_N = m[0];
    fw_error.P_E = m[1];
    fw_error.P_D = fw_sp.altitude - fw_states_filtered.altitude;
    fw_error.P_NE = sqrt((m[0] * m[0] + m[1] * m[1]));

    //3. 计算领机速度与从机速度之差在从机坐标系下的投影

    Point led_ground_speed_2d(leader_states_filtered.global_vel_x, leader_states_filtered.global_vel_y);
    Point led_fol_vel_error = led_ground_speed_2d - fw_ground_speed_2d;

    fw_error.led_fol_vxb = fw_ground_speed_2d_unit * led_fol_vel_error; //沿速度（机体x）方向速度偏差（已检验）
    fw_error.led_fol_vyb = fw_ground_speed_2d_unit ^ led_fol_vel_error; //垂直速度（机体x）方向速度偏差（已检验）

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

    del_fol_gspeed = gspeed_pid.pid_anti_saturated(mix_v_p_Xb, use_integ, use_diff); //飞机机头方向期望速度增量
    fw_sp.ground_speed = del_fol_gspeed + fw_ground_speed_2d.len();                  //沿着从机机头方向（飞机地速方向）的期望值

    //5. 转换期望速度成为期望空速，与期望高度一起进入TECS，产生油门以及俯仰角。

    Point wind_vector(fw_states_filtered.wind_estimate_x, fw_states_filtered.wind_estimate_y);
    float wind_Xb = wind_vector * fw_ground_speed_2d_unit;

    airspd_sp = fw_sp.ground_speed + wind_Xb; //此处的空速应该是加法

    //符合飞机本身的加速减速特性
    if ((airspd_sp - airspd_sp_prev) > formation_params.maxinc_acc * _dt)
    {
        airspd_sp = airspd_sp_prev + formation_params.maxinc_acc * _dt;
    }
    else if ((airspd_sp - airspd_sp_prev) < -formation_params.maxdec_acc * _dt)
    {
        airspd_sp = airspd_sp_prev - formation_params.maxdec_acc * _dt;
    }

    airspd_sp_prev = airspd_sp;

    fw_sp.air_speed = constrain(airspd_sp, formation_params.min_arispd_sp, formation_params.max_arispd_sp);
    //fw_sp.air_speed = 20.0;
    /**
    test,此处显示了TECS直接追动态的速度过于不好，期望速度的噪声过大
    （或者变化范围过大）直接导致总能量的变化过大，飞机的油门就会抽搐
    */
    /**
    * TODO:此处应该完成的任务是：
    * 1.将期望空速记录一下，看一下和时间的关系
    * 2.实现空速期望值的阶跃式增加或者减少，增加的量不超过飞机的最大加速度以及最小减速加速度×dt
    * 3.实现空速期望值的给定值平稳，给的频率要降下来，因为TECS内环的带宽有限
   */

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

    if (fw_sp.altitude - fw_states.altitude >= 10) //判断一下是否要进入爬升

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
                                tecs_params.climbout_pitch_min_rad, tecs_params.throttle_min,
                                tecs_params.throttle_max, tecs_params.throttle_cruise,
                                tecs_params.pitch_min_rad, tecs_params.pitch_max_rad);

    _cmd.pitch = _tecs.get_pitch_setpoint();
    _cmd.thrust = _tecs.get_throttle_setpoint();

    //6. 机体侧向混合误差,或者只有位置误差（当前）,由L1控制器解算滚转以及偏航角。

    _lateral_controller.lateral_L1_modified(current_pos, pos_sp, fw_ground_speed_2d, fw_states_filtered.air_speed);

    _cmd.roll = constrain(_lateral_controller.nav_roll(),
                          -lateral_controller_params.roll_max, lateral_controller_params.roll_max);

    abs_pos_vel_ctrl_timestamp = now;

    //7. 数据记录
    /*     float data[5];
    data[0] = fw_sp.air_speed;
    data[1] = _cmd.thrust;
    data[2] = _cmd.pitch;
    data[3] = _cmd.roll;
    write_to_files("/home/lee/fw_sp.air_speed", fw_states.flight_mode, data); */
}

void FORMATION_CONTROL::abs_pos_vel_controller1()
{
    /*    
    *领机绝对位置以及绝对速度GPS控制器，以机体坐标系为准，误差直接投影到从机机体坐标系之中
    * */
    long now = get_sys_time();
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
    //TODO:此处的空速像这样计算才算正确，使用加法
    float led_airspd_x = leader_states_filtered.wind_estimate_x + leader_states_filtered.global_vel_x; //此处的空速算法有待验证
    float led_airspd_y = leader_states_filtered.wind_estimate_y + leader_states_filtered.global_vel_y;

    led_arispd.set_vec_ele(led_airspd_x, led_airspd_y);                                                  //领机空速向量
    led_gspeed_2d.set_vec_ele(leader_states_filtered.global_vel_x, leader_states_filtered.global_vel_y); //领机地速向量

    cout << "领机状态" << endl;
    cout << "验证用，滤波后的地速x大小为：" << leader_states_filtered.global_vel_x << endl;
    cout << "验证用，滤波后的地速y大小为：" << leader_states_filtered.global_vel_y << endl;
    cout << "验证用，计算的空速大小为：" << led_arispd.len() << endl;
    cout << "验证用，实际获取空速为：" << leader_states_filtered.air_speed << endl;
    cout << "验证用，计算的空速大小以及实际获取空速之差为：" << (led_arispd.len() - leader_states_filtered.air_speed) << endl;

    if (leader_states_filtered.yaw_valid) //将认为机头实际指向
    {
        led_cos_yaw = cosf(leader_states_filtered.yaw_angle);
        led_sin_yaw = sinf(leader_states_filtered.yaw_angle);
    }
    else if ((!leader_states_filtered.yaw_valid) && //将认为空速方向与机头方向一致
             (led_arispd.len2() != 0.0))
    {

        led_cos_yaw = led_arispd.x / led_arispd.len();
        led_sin_yaw = led_arispd.y / led_arispd.len();
    }
    else if ((!leader_states_filtered.yaw_valid) && //将认为地速方向与机头方向一致
             (led_arispd.len2() == 0.0) &&
             (led_gspeed_2d.len2() != 0.0))
    {
        led_cos_yaw = led_gspeed_2d.x / led_gspeed_2d.len();
        led_sin_yaw = led_gspeed_2d.y / led_gspeed_2d.len();
    }
    else //三种可用情况之外
    {
        led_cos_yaw = 0;
        led_sin_yaw = 0;
        cout << "警告：无法计算领机机头朝向，请检查输入信息是否有误" << endl;
        return;
    }

    formation_offset.ned_n = led_cos_yaw * formation_offset.xb + (-led_sin_yaw * formation_offset.yb);
    formation_offset.ned_e = led_sin_yaw * formation_offset.xb + led_cos_yaw * formation_offset.yb;
    formation_offset.ned_d = formation_offset.zb;

    double ref[3], result[3];
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

    float fw_airspd_x = fw_states_filtered.wind_estimate_x + fw_states_filtered.global_vel_x; //此处的空速算法有待验证
    float fw_airspd_y = fw_states_filtered.wind_estimate_y + fw_states_filtered.global_vel_y;

    fw_arispd.set_vec_ele(fw_airspd_x, fw_airspd_y);                                            //本机空速向量
    fw_gspeed_2d.set_vec_ele(fw_states_filtered.global_vel_x, fw_states_filtered.global_vel_y); //本机地速向量

    cout << "本机机状态" << endl;
    cout << "验证用，滤波后的地速x大小为：" << fw_states_filtered.global_vel_x << endl;
    cout << "验证用，滤波后的地速y大小为：" << fw_states_filtered.global_vel_y << endl;
    cout << "验证用，计算的空速大小为：" << fw_arispd.len() << endl;
    cout << "验证用，实际获取空速为：" << fw_states_filtered.air_speed << endl;
    cout << "验证用，计算的空速大小以及实际获取空速之差为：" << (fw_arispd.len() - fw_states_filtered.air_speed) << endl;

    if (fw_states_filtered.yaw_valid) //将认为机头实际指向
    {
        fw_cos_yaw = cosf(fw_states_filtered.yaw_angle);
        fw_sin_yaw = sinf(fw_states_filtered.yaw_angle);
    }
    else if ((!fw_states_filtered.yaw_valid) && //将认为空速方向与机头方向一致
             (fw_arispd.len2() != 0.0))
    {

        fw_cos_yaw = fw_arispd.x / fw_arispd.len();
        fw_sin_yaw = fw_arispd.y / fw_arispd.len();
    }
    else if ((!fw_states_filtered.yaw_valid) && //将认为地速方向与机头方向一致
             (fw_arispd.len2() == 0.0) &&
             (fw_gspeed_2d.len2() != 0.0))
    {
        fw_cos_yaw = fw_gspeed_2d.x / fw_gspeed_2d.len();
        fw_sin_yaw = fw_gspeed_2d.y / fw_gspeed_2d.len();
    }
    else //三种可用情况之外
    {
        fw_cos_yaw = 0;
        fw_sin_yaw = 0;
        cout << "警告：无法计算从机机头朝向，请检查输入信息是否有误" << endl;
        return;
    }

    Vec fw_body_unit(fw_cos_yaw, fw_sin_yaw);
    fw_body_unit = fw_body_unit.normalized(); //保证归一化的结果，此向量十分重要

    /**
    * 3. 计算从机的期望位置与当前位置的误差在从机坐标系下的投影
    */
    Point pos_sp(fw_sp.latitude, fw_sp.longitude);                                //期望位置
    Point current_pos(fw_states_filtered.latitude, fw_states_filtered.longitude); //当前位置
    Vec vector_plane_sp = get_plane_to_sp_vector(current_pos, pos_sp);            //计算飞机到期望点向量(本质来说是误差向量)

    fw_error.PXb = fw_body_unit * vector_plane_sp;               //沿速度（机体x）方向距离误差（待检验）
    fw_error.PYb = fw_body_unit ^ vector_plane_sp;               //垂直于速度（机体x）方向距离误差
    fw_error.PZb = fw_sp.altitude - fw_states_filtered.altitude; //高度方向误差

    double a_pos[2], b_pos[2], m[2]; //计算ned坐标系下的位置误差
    a_pos[0] = fw_states_filtered.latitude;
    a_pos[1] = fw_states_filtered.longitude;
    b_pos[0] = fw_sp.latitude;
    b_pos[1] = fw_sp.longitude;
    cov_lat_long_2_m(a_pos, b_pos, m);

    //NED误差记录
    fw_error.P_N = m[0];
    fw_error.P_E = m[1];
    fw_error.P_D = fw_sp.altitude - fw_states_filtered.altitude;
    fw_error.P_NE = sqrt((m[0] * m[0] + m[1] * m[1]));

    /**
    * 4. 计算领机从机地速“差”在从机坐标系之中的投影，控制量是地速，所以是地速之差
    */
    Vec led_fol_vel_error = led_gspeed_2d - fw_gspeed_2d;
    fw_error.led_fol_vxb = fw_body_unit * led_fol_vel_error; //沿速度（机体x）方向速度偏差（已检验）
    fw_error.led_fol_vyb = fw_body_unit ^ led_fol_vel_error; //垂直速度（机体x）方向速度偏差（已检验）
    /**
    * 5. 根据飞机机体前向混合误差产生原始空速期望值，并对此空速先后进行滤波、增量限幅、最终限幅约束
    * 得到最终期望空速。
    * 机体前向误差分类，超过一定误差，最大空速直接给满，迅速减小误差。小于一定误差，按照控制逻辑正常产生
    */
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

    del_fol_gspeed = gspeed_pid.pid_anti_saturated(mix_v_p_Xb, use_integ, use_diff); //飞机机头方向期望速度增量
    fw_sp.ground_speed = del_fol_gspeed + fw_body_unit * led_gspeed_2d;              //沿着从机机头方向（飞机空速方向）的期望值

    //期望地速按照风估计转化为期望空速
    fw_wind_vector.set_vec_ele(fw_states_filtered.wind_estimate_x, fw_states_filtered.wind_estimate_y);
    float wind_Xb = fw_body_unit * fw_wind_vector; //沿着飞机机体前向的风估计

    airspd_sp = fw_sp.ground_speed + wind_Xb; //此处的空速应该是加法

    if (fw_error.PXb > 50.0) //如果距离误差很大，那么就直接给满
    {
        airspd_sp = formation_params.max_arispd_sp;
    }

    //符合飞机本身的加速减速特性
    if ((airspd_sp - airspd_sp_prev) > formation_params.maxinc_acc * _dt)
    {
        airspd_sp = airspd_sp_prev + formation_params.maxinc_acc * _dt;
    }
    else if ((airspd_sp - airspd_sp_prev) < -formation_params.maxdec_acc * _dt)
    {
        airspd_sp = airspd_sp_prev - formation_params.maxdec_acc * _dt;
    }

    airspd_sp_prev = airspd_sp;

    fw_sp.air_speed = constrain(airspd_sp, formation_params.min_arispd_sp, formation_params.max_arispd_sp);
    /**
    * 6.期望GPS高度以及期望空速进入TECS得到油门以及俯仰角
    */
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

    if (fw_sp.altitude - fw_states.altitude >= 10) //判断一下是否要进入爬升

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
                                tecs_params.climbout_pitch_min_rad, tecs_params.throttle_min,
                                tecs_params.throttle_max, tecs_params.throttle_cruise,
                                tecs_params.pitch_min_rad, tecs_params.pitch_max_rad);

    _cmd.pitch = _tecs.get_pitch_setpoint();
    _cmd.thrust = _tecs.get_throttle_setpoint();
    /**
    * 7.根据飞机机体侧向混合误差，进入横侧向控制器，产生原始期望滚转角。
    */
    _lateral_controller.lateral_L1_modified(current_pos, pos_sp, fw_gspeed_2d, fw_states_filtered.air_speed);

    _cmd.roll = constrain(_lateral_controller.nav_roll(),
                          -lateral_controller_params.roll_max, lateral_controller_params.roll_max);

    /**
    * 8.TODO:原始期望滚转角平滑滤波，限幅，角速度限幅，
    */

    abs_pos_vel_ctrl_timestamp = now;
}

bool FORMATION_CONTROL::identify_led_fol_states()
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

Point FORMATION_CONTROL::get_plane_to_sp_vector(Point origin, Point target)
{
    Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

    return out * double(CONSTANTS_RADIUS_OF_EARTH);
}

void FORMATION_CONTROL::print_data(const struct _s_fw_states *p)
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

    cout << "body下的加速度【XYZ】" << p->body_acc[0] << " [m/ss] "
         << p->body_acc[1] << " [m/ss] "
         << p->body_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的速度【XYZ】" << p->ned_vel_x << " [m/s] "
         << p->ned_vel_y << " [m/s] "
         << p->ned_vel_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的加速度【XYZ】(由旋转矩阵得来)" << p->ned_acc[0] << " [m/ss] "
         << p->ned_acc[1] << " [m/ss] "
         << p->ned_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "GPS位置【lat,long,alt,rel_alt】" << p->latitude << " [] "
         << p->longitude << " [] "
         << p->altitude << " [] "
         << p->relative_alt << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "风估计【x,y,z】" << p->wind_estimate_x << " [m/s] "
         << p->wind_estimate_y << " [m/s] "
         << p->wind_estimate_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "***************以上是本飞机状态******************" << endl;
    cout << "***************以上是本飞机状态******************" << endl;
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
}

void FORMATION_CONTROL::get_formation_4cmd(struct _s_4cmd &fw_cmd)
{
    fw_cmd = _cmd;
}

void FORMATION_CONTROL::get_formation_sp(struct _s_fw_sp &formation_sp) //得到编队中本机的运动学期望值
{
    formation_sp = fw_sp;
}

void FORMATION_CONTROL::get_formation_error(struct _s_fw_error &formation_error) //得到编队控制误差
{
    formation_error = fw_error;
}

void FORMATION_CONTROL::get_formation_params(struct _s_formation_params &format_params)
{
    format_params = formation_params;
}
