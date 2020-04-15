/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-08 10:56:50
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 00:00:06
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "abs_formation_controller.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description: 以便重置控制器中有“记忆”的量
 */
void ABS_FORMATION_CONTROLLER::reset_formation_controller()
{
  rest_speed_pid = true;
  rest_tecs = true;
  reset_lateral_controller = true;
}
/**
 * @Input: void
 * @Output: void
 * @Description: 设定abs编队混合误差产生参数
 */
void ABS_FORMATION_CONTROLLER::set_mix_Xerr_params(struct _s_mix_Xerr_params &input_params)
{
  mix_Xerr_params = input_params;
}
/**
 * @Input: void
 * @Output: void
 * @Description: 设定tecs控制器参数
 */
void ABS_FORMATION_CONTROLLER::set_tecs_params(struct _s_tecs_params &input_params)
{
  tecs_params = input_params;
}
/**
 * @Input: void
 * @Output: void
 * @Description: 设定衡侧向控制器参数
 */
void ABS_FORMATION_CONTROLLER::set_lateral_ctrller_params(struct _s_lateral_controller_params &input_params)
{
  lateral_controller_params = input_params;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 调用滤波器对输入的飞机原始状态进行滤波
 */
void ABS_FORMATION_CONTROLLER::filter_led_fol_states()
{
  fw_states_f = fw_states;
  leader_states_f = leader_states;

  if (use_the_filter)
  {
    leader_states_f.global_vel_x =
        led_gol_vel_x_filter.one_order_filter(leader_states.global_vel_x);

    leader_states_f.global_vel_y =
        led_gol_vel_y_filter.one_order_filter(leader_states.global_vel_y);
  }
}
/**
 * @Input: void
 * @Output: void
 * @Description: 计算从飞机当前位置到期望的位置的向量
 */
Point ABS_FORMATION_CONTROLLER::get_plane_to_sp_vector(Point origin, Point target)
{
  Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

  return out * double(CONSTANTS_RADIUS_OF_EARTH);
}
/**
 * @Input: void
 * @Output: void
 * @Description: 获得编队控制器参数
 */
void ABS_FORMATION_CONTROLLER::get_mix_Xerr_params(struct _s_mix_Xerr_params &mix_error_para)
{
  mix_error_para = mix_Xerr_params;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 控制器主函数
 */
void ABS_FORMATION_CONTROLLER::control_formation()
{
  long now = get_sys_time();
  _dt = constrain((now - abs_pos_vel_ctrl_timestamp) * 1.0e-3f, _dtMin, _dtMax);

  /**
    * 0. 原始数据滤波,从此之后，飞机的状态保存在了滤波后的状态值里面
    */

  filter_led_fol_states();

  if (!identify_led_fol_states())
  {
    ABS_FORMATION_CONTROLLER_INFO("警告：领机或从机未在飞行之中，无法执行编队控制器");
    return;
  }

  /**
    * 1. 根据队形要求，计算出从机期望位置的在领机航迹坐标系下的位置，然后再到->GPS位置（期望经纬高）；
    * 在此之中注意领机航迹方向的选择。
    *   a. 计算领机机头方向
    *   b. 计算从机期望位置的gps位置
    */

  float led_airspd_x = leader_states_f.wind_estimate_x + leader_states_f.global_vel_x;
  float led_airspd_y = leader_states_f.wind_estimate_y + leader_states_f.global_vel_y;

  led_arispd.set_vec_ele(led_airspd_x, led_airspd_y);                                    /* 领机空速向量 */
  led_gspeed_2d.set_vec_ele(leader_states_f.global_vel_x, leader_states_f.global_vel_y); /* 领机地速向量 */

  cout << "领机状态" << endl;
  cout << "验证用，滤波后的地速x大小为：" << leader_states_f.global_vel_x << endl;
  cout << "验证用，滤波后的地速y大小为：" << leader_states_f.global_vel_y << endl;
  cout << "验证用，计算的空速大小为：" << led_arispd.len() << endl;
  cout << "验证用，实际获取空速为：" << leader_states_f.air_speed << endl;
  cout << "验证用，计算的空速大小以及实际获取空速之差为：" << (led_arispd.len() - leader_states_f.air_speed) << endl;

  /* 计算获得的空速与读取的空速差距较大 */
  if ((led_arispd.len() - leader_states_f.air_speed) >= 3.0)
  {
    led_airspd_states_valid = false;
  }
  else
  {
    led_airspd_states_valid = true;
  }

  /*地速太小的情况*/
  if (led_gspeed_2d.len() <= 3.0)
  {
    if (leader_states_f.yaw_valid)
    {
      led_cos_dir = cosf(leader_states_f.yaw_angle);
      led_sin_dir = sinf(leader_states_f.yaw_angle);
      ABS_FORMATION_CONTROLLER_INFO("领机地速太小，选用领机航向角");
    }
    else
    {
      led_cos_dir = 0;
      led_sin_dir = 0;
      ABS_FORMATION_CONTROLLER_INFO("领机地速太小，且领机航向角未知");
    }
  }
  else
  {
    led_cos_dir = led_gspeed_2d.x / led_gspeed_2d.len();
    led_sin_dir = led_gspeed_2d.y / led_gspeed_2d.len();
    ABS_FORMATION_CONTROLLER_INFO("领机地速正常，选用领机地速方向");
  }

  formation_offset.ned_n = led_cos_dir * formation_offset.xb + (-led_sin_dir * formation_offset.yb);
  formation_offset.ned_e = led_sin_dir * formation_offset.xb + led_cos_dir * formation_offset.yb;
  formation_offset.ned_d = formation_offset.zb;

  double ref[3], result[3];
  ref[0] = leader_states_f.latitude;
  ref[1] = leader_states_f.longitude;
  ref[2] = leader_states_f.altitude;

  cov_m_2_lat_long_alt(ref, formation_offset.ned_n, formation_offset.ned_e, formation_offset.ned_d, result);

  fw_sp.latitude = result[0];
  fw_sp.longitude = result[1];
  fw_sp.altitude = result[2];

  /* 保证归一化的结果，此向量代表了领机的速度方向*/
  Vec led_dir_unit(led_cos_dir, fw_sin_dir);
  led_dir_unit = led_dir_unit.normalized();

  /**
     * 2. 计算本机航迹系，默认顺序：
     *      1. 本机地速方向
     *      2. 本机航迹角
     *      3. 有误
    */

  float fw_airspd_x = fw_states_f.wind_estimate_x + fw_states_f.global_vel_x;
  float fw_airspd_y = fw_states_f.wind_estimate_y + fw_states_f.global_vel_y;

  fw_arispd.set_vec_ele(fw_airspd_x, fw_airspd_y);                              /* 本机空速向量 */
  fw_gspeed_2d.set_vec_ele(fw_states_f.global_vel_x, fw_states_f.global_vel_y); /* 本机地速向量 */

  cout << "本机机状态" << endl;
  cout << "验证用，滤波后的地速x大小为：" << fw_states_f.global_vel_x << endl;
  cout << "验证用，滤波后的地速y大小为：" << fw_states_f.global_vel_y << endl;
  cout << "验证用，计算的空速大小为：" << fw_arispd.len() << endl;
  cout << "验证用，实际获取空速为：" << fw_states_f.air_speed << endl;
  cout << "验证用，计算的空速大小以及实际获取空速之差为：" << (fw_arispd.len() - fw_states_f.air_speed) << endl;

  if ((fw_arispd.len() - fw_states_f.air_speed) >= 3.0) /* 计算获得的空速与读取的空速差距较大 */
  {
    fw_airspd_states_valid = false;
  }
  else
  {
    fw_airspd_states_valid = true;
  }

  if (fw_gspeed_2d.len() <= 3.0) /*本机地速太小的情况*/
  {
    if (fw_states_f.yaw_valid)
    {
      fw_cos_dir = cosf(fw_states_f.yaw_angle);
      fw_sin_dir = sinf(fw_states_f.yaw_angle);
      ABS_FORMATION_CONTROLLER_INFO("本机地速太小，选用从机航向角");
    }
    else
    {
      fw_cos_dir = 0;
      fw_sin_dir = 0;
      ABS_FORMATION_CONTROLLER_INFO("本机地速太小，且从机航向角未知");
    }
  }
  else
  {
    fw_cos_dir = fw_gspeed_2d.x / fw_gspeed_2d.len();
    fw_sin_dir = fw_gspeed_2d.y / fw_gspeed_2d.len();
    ABS_FORMATION_CONTROLLER_INFO("本机地速正常，选用从机地速方向");
  }

  /* 保证归一化的结果，此向量十分重要，代表了从机速度方向*/
  Vec fw_dir_unit(fw_cos_dir, fw_sin_dir);
  fw_dir_unit = fw_dir_unit.normalized();

  /**
    * 3. 计算从机的期望位置与当前位置的误差在从机航迹坐标系下的投影
    */

  Point pos_sp(fw_sp.latitude, fw_sp.longitude);                     /* 期望位置 */
  Point current_pos(fw_states_f.latitude, fw_states_f.longitude);    /* 当前位置 */
  Vec vector_plane_sp = get_plane_to_sp_vector(current_pos, pos_sp); /* 计算飞机到期望点向量(本质来说是误差向量) */

  fw_error.PXk = fw_dir_unit * vector_plane_sp;         /* 沿地速度方向距离误差（待检验） */
  fw_error.PYk = fw_dir_unit ^ vector_plane_sp;         /* 垂直于地速度方向距离误差 */
  fw_error.PZk = fw_sp.altitude - fw_states_f.altitude; /* 高度方向误差 */

  double a_pos[2], b_pos[2], m[2]; /* 计算ned坐标系下的位置误差 */
  a_pos[0] = fw_states_f.latitude;
  a_pos[1] = fw_states_f.longitude;
  b_pos[0] = fw_sp.latitude;
  b_pos[1] = fw_sp.longitude;
  cov_lat_long_2_m(a_pos, b_pos, m);

  /* NED误差记录 */
  fw_error.P_N = m[0];
  fw_error.P_E = m[1];
  fw_error.P_D = fw_sp.altitude - fw_states_f.altitude;
  fw_error.P_NE = sqrt((m[0] * m[0] + m[1] * m[1]));

  /**
    * 4. 计算领机从机地速“差”在从机航迹坐标系之中的投影，控制量是地速，所以是地速之差
    */

  Vec led_fol_vel_error = led_gspeed_2d - fw_gspeed_2d;
  fw_error.led_fol_vxk = fw_dir_unit * led_fol_vel_error;                         /* 沿地速方向 */
  fw_error.led_fol_vyk = fw_dir_unit ^ led_fol_vel_error;                         /* 垂直地直速方向*/
  fw_error.led_fol_vzk = leader_states_f.global_vel_z - fw_states_f.global_vel_z; /*竖直速度之差*/

  fw_error.led_fol_vk = fw_error.led_fol_vxk * fw_error.led_fol_vxk + /*地速大小之差*/
                        fw_error.led_fol_vyk * fw_error.led_fol_vyk +
                        fw_error.led_fol_vzk * fw_error.led_fol_vzk;

  /**
     *5. 计算领机从机速度角度，机体前向右偏为正,范围（0～pi/2),左偏为负，范围（-pi/2～0）。
     */

  float led_gsp_Xk = fw_dir_unit * led_gspeed_2d; /*领机地速在本机航迹坐标系中的投影*/
  float led_gsp_Yk = fw_dir_unit ^ led_gspeed_2d;

  if (led_gsp_Xk > 0 && led_gsp_Yk > 0) /*右前方*/
  {
    fw_error.led_fol_eta = atanf(led_gsp_Yk / led_gsp_Xk);
  }
  else if (led_gsp_Xk > 0 && led_gsp_Yk < 0) /*右后方*/
  {
    fw_error.led_fol_eta = PI + atanf(led_gsp_Yk / led_gsp_Xk);
  }
  else if (led_gsp_Xk < 0 && led_gsp_Yk > 0) /*左前方*/
  {
    fw_error.led_fol_eta = atanf(led_gsp_Yk / led_gsp_Xk);
  }
  else if (led_gsp_Xk < 0 && led_gsp_Yk < 0) /*左后方*/
  {
    fw_error.led_fol_eta = -PI + atanf(led_gsp_Yk / led_gsp_Xk);
  }
  else if (led_gsp_Xk == 0 && led_gsp_Yk < 0) /*左方*/
  {
    fw_error.led_fol_eta = -PI / 2;
  }
  else if (led_gsp_Xk == 0 && led_gsp_Yk > 0) /*右方*/
  {
    fw_error.led_fol_eta = PI / 2;
  }
  else if (led_gsp_Xk < 0 && led_gsp_Yk == 0) /*后方*/
  {
    fw_error.led_fol_eta = -PI;
  }
  else if (led_gsp_Xk > 0 && led_gsp_Yk == 0) /*前方*/
  {
    fw_error.led_fol_eta = 0;
  }
  else /*有误*/
  {
    fw_error.led_fol_eta = 0;
    ABS_FORMATION_CONTROLLER_INFO("领机从机角度误差有误");
  }

  /**
     * 6.判断控制分段
     */

  if (vector_plane_sp.len() >= 30.0)
  {
    format_method = _e_format_method::LONG_DIS;
  }
  else
  {
    format_method = _e_format_method::CLOSE_DIS;
  }

  /**
     * 7. 利用前向位置、速度误差产生期望速度大小
     */

  if (format_method == _e_format_method::LONG_DIS)
  {
    airspd_sp = fw_params.max_arispd_sp;
  }
  else if (format_method == _e_format_method::CLOSE_DIS)
  {
    /* 1.产生前向混合误差 */

    float mix_err_Xk = mix_Xerr_params.kp_p * fw_error.PXk +
                       mix_Xerr_params.kv_p * fw_error.led_fol_vk;

    /* 2.混合误差产生期望地速 */
    if (rest_speed_pid) {
      rest_speed_pid = false;
      gspeed_sp_pid.reset_incre_pid();
    }

    gspeed_sp_pid.increment_pid(mix_err_Xk, mix_Xerr_params.mix_kp,
                                mix_Xerr_params.mix_ki, mix_Xerr_params.mix_kd);

    fw_sp.ground_speed = gspeed_sp_pid.get_full_output();

    /* 3.期望地速产生期望空速 */

    fw_wind_vector.set_vec_ele(fw_states_f.wind_estimate_x,
                               fw_states_f.wind_estimate_y);
    float wind_Xk = fw_dir_unit * fw_wind_vector; /* 沿着飞机机体前向的风估计 */

    airspd_sp = fw_sp.ground_speed + wind_Xk; /* 此处的空速应该是加法 */

    cout << "最原始的空速设定值为" << airspd_sp << endl;

    /* 符合飞机本身的加速减速特性： */
    /* TODO:慎用，加上之后,需要加大飞机的前向后相加速度，由于延时作用太强，可能导致不稳定。
       */
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

    fw_sp.air_speed = constrain(airspd_sp, fw_params.min_arispd_sp,
                                fw_params.max_arispd_sp);
  }
  else
  {
    ABS_FORMATION_CONTROLLER_INFO("分段有误，无法计算期望速度");
  }

  /**
     * 8.调用TECS获得期望俯仰角以及期望油门
     */
  if (rest_tecs)
  {
    rest_tecs = false;
    _tecs.reset_state();
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

  _tecs.update_vehicle_state_estimates(fw_states_f.air_speed, fw_states_f.rotmat, fw_states_f.body_acc,
                                       fw_states_f.altitude_lock, fw_states_f.in_air, fw_states_f.altitude,
                                       vz_valid, fw_states_f.ned_vel_z, fw_states_f.body_acc[2]);

  _tecs.update_pitch_throttle(fw_states_f.rotmat, fw_states_f.pitch_angle,
                              fw_states_f.altitude, fw_sp.altitude, fw_sp.air_speed,
                              fw_states_f.air_speed, tecs_params.EAS2TAS, tecs_params.climboutdem,
                              tecs_params.climbout_pitch_min_rad, fw_params.throttle_min,
                              fw_params.throttle_max, fw_params.throttle_cruise,
                              fw_params.pitch_min_rad, fw_params.pitch_max_rad);

  _cmd.pitch = _tecs.get_pitch_setpoint();
  _cmd.thrust = _tecs.get_throttle_setpoint();

  /**
     * 9. 利用横侧向位置、角度误差产生期望滚转角
     */

  if (format_method == _e_format_method::LONG_DIS)
  {

    /* L1控制方法 */

    l1_controller.l1_controller(current_pos, pos_sp, fw_gspeed_2d, fw_states_f.air_speed);

    roll_cmd = l1_controller.nav_roll(); /* 获取期望控制滚转 */
  }
  else if (format_method == _e_format_method::CLOSE_DIS)
  {

    /* 位置与角度误差控制方法 */

    float mix_err_Yk = mix_Yerr_params.keta_p * fw_error.led_fol_eta + mix_Yerr_params.kp_p * fw_error.PYk;

    if (reset_lateral_controller)
    {
      reset_lateral_controller = false;
      roll_sp_pid.reset_incre_pid();
    }

    roll_sp_pid.increment_pid(mix_err_Yk, mix_Yerr_params.mix_kp, mix_Yerr_params.mix_ki, mix_Yerr_params.mix_kd);

    roll_cmd = roll_sp_pid.get_full_output();
  }
  else
  {
    ABS_FORMATION_CONTROLLER_INFO("分段有误，无法计算期望滚转");
  }

  roll_cmd = constrain(roll_cmd, -fw_params.roll_max, fw_params.roll_max);

  _cmd.roll = roll_cmd;
}
