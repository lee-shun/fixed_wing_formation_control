/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-08 10:36:38
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-10 01:04:08
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _ABS_FORMATION_CONTROLLER_HPP_
#define _ABS_FORMATION_CONTROLLER_HPP_
#include "formation_controller.hpp"

#define ABS_FORMATION_CONTROLLER_INFO(a) cout << "[ABS_FORMATION_CONTROLLER_INFO]:" << a << endl

class ABS_FORMATION_CONTROLLER : protected FORMATION_CONTROLLER
{
public:
    /*本控制器主函数*/
    void control_formation() override;

    /*重置控制器*/
    void reset_formation_controller();

    /* 设定编队控制器参数（主管产生期望空速） */
    void set_formation_params(struct _s_mix_error_params &input_params);

    /* 设定TECS控制器参数 */
    void set_tecs_params(struct _s_tecs_params &input_params);

    /* 设定横侧向控制器参数 */
    void set_lateral_ctrller_params(struct _s_lateral_controller_params &mix_error_para);

    /* 得到编队控制器参数 */
    void get_formation_params(struct _s_mix_error_params &format_params);

private:
    /**
    *编队控制器外函数，变量（组）
    */

    long abs_pos_vel_ctrl_timestamp{0}; /* 绝对速度位置控制器时间戳 */

    _s_leader_states leader_states_f; /* 滤波后的领机信息 */
    _s_fw_states fw_states_f;         /* 滤波后的从机信息 */

    void filter_led_fol_states();       /* 完成对于领机从机的滤波函数 */
    bool use_the_filter{true};          /* 是否使用滤波器对原始数据滤波 */
    FILTER led_gol_vel_x_filter;        /* 领机gol速度x滤波器 */
    FILTER led_gol_vel_y_filter;        /* 领机gol速度y滤波器 */
    bool fw_airspd_states_valid{true};  /* 检验计算本机的空速（状态）以及实际读取的空速的合法性 */
    bool led_airspd_states_valid{true}; /* 检验计算领机的空速（状态）以及实际读取的空速的合法性 */

    Vec led_arispd;     /* 领机空速向量 */
    Vec led_gspeed_2d;  /* 领机地速向量 */
    Vec fw_arispd;      /* 本机空速向量 */
    Vec fw_gspeed_2d;   /* 本机地速向量 */
    Vec fw_wind_vector; /* 本机风估计向量 */

    double led_cos_dir{0.0}; /* 领机dir_cos，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double led_sin_dir{0.0}; /* 领机dir_sin，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double fw_cos_dir{0.0};  /* 本机dir_cos，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double fw_sin_dir{0.0};  /* 本机dir_sin，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */

    _s_mix_error_params mix_error_params; /* 编队控制器混合误差产生参数,编队控制器参数 */

    INCREMENT_PID_CONTROLLER gspeed_sp_pid; /*产生期望地速的pid*/
    bool rest_speed_pid{false};            /* 重置内部控器标志量 */
    float del_fol_gspeed{0.0};             /* 从机期望地速增量，最终实现的是领机与从机地速一致 */
    float airspd_sp_prev{0.0};             /* 飞机期望空速（前一时刻） */
    float airspd_sp{0.0};                  /* 飞机期望空速 */
    FILTER airspd_sp_filter;               /* 本机空速期望值滤波器 */

    /**
    * TECS函数，变量（组）
    */

    TECS _tecs;                 /* TECS控制器 */
    bool rest_tecs{false};      /* 重置TECS */
    bool vz_valid{false};       /* 纵向速度有效标志位 */
    _s_tecs_params tecs_params; /* TECS参数 */

    /**
    * 横侧性控制器函数，变量（组）
    */

    LATERAL_CONTROLLER _lateral_controller;                 /* 横侧向控制器 */
    _s_lateral_controller_params lateral_controller_params; /* 横侧向控制器参数 */
    float roll_cmd{0.0};                                    /* 最终roll通道控制量 */
    float roll_cmd_prev{0.0};                               /* 最终roll通道控制量 */
    FILTER roll_cmd_filter;                                 /* 期望滚转角滤波器 */

    /**
    * 其他计算函数，变量（组）
    */
    Point get_plane_to_sp_vector(Point origin, Point target); /* 原始信息预处理 */
};

#endif
