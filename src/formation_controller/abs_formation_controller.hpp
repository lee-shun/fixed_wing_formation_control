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
 * @LastEditTime: 2020-04-13 00:04:52
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _ABS_FORMATION_CONTROLLER_HPP_
#define _ABS_FORMATION_CONTROLLER_HPP_

#include "formation_controller.hpp"
#include "../fixed_wing_lib/vector.hpp"
#include "../fixed_wing_lib/filter.hpp"
#include "../fixed_wing_lib/increment_pid_controller.hpp"
#include "../fixed_wing_lib/lateral_controller/L1_vel_controller.hpp"
#include "../fixed_wing_lib/lateral_controller/mix_angle_pos_controller.hpp"
#include "../fixed_wing_lib/vertical_controller/tecs.hpp"

#define ABS_FORMATION_CONTROLLER_INFO(a) \
    cout << "[ABS_FORMATION_CONTROLLER_INFO]:" << a << endl

class ABS_FORMATION_CONTROLLER : public FORMATION_CONTROLLER
{
public:
    /* 编队控制器X方向混合误差产生参数,编队控制器参数 */
    struct _s_mix_Xerr_params
    {
        /* 主从机速度差比例项 */
        float kv_p{0.2};

        /* 从机期望与实际位置误差比例 */
        float kp_p{0.5};

        /* 总混合产生期望空速pid参数 */
        float mix_kp{0.4};

        /* 总混合产生期望空速pid参数 */
        float mix_kd{0.0};

        /* 总混合产生期望空速pid参数 */
        float mix_ki{0.0};
    };

    /* 编队控制器Y方向混合误差产生参数,编队控制器参数 */
    struct _s_mix_Yerr_params
    {
        /* 主从机位置差比例项 */
        float kp_p{0.2};

        /* 领机从机速度方向偏差比例 */
        float keta_p{0.5};

        /* 总混合产生期望roll pid参数 */
        float mix_kp{0.4};

        /* 总混合产生期望roll pid参数 */
        float mix_kd{0.0};

        /* 总混合产生期望roll pid参数 */
        float mix_ki{0.0};
    };

    /* TECS控制器参数 */
    struct _s_tecs_params
    {
        int EAS2TAS{1};

        bool climboutdem{false};

        float climbout_pitch_min_rad{0.2};

        float speed_weight{1};

        float time_const_throt{8.0};

        float time_const{5.0};
    };

    /*  横侧向控制器参数 */
    struct _s_lateral_controller_params
    {
    };

    /*编队控制器分段方法*/
    enum _e_format_method
    {
        LONG_DIS = 0,
        CLOSE_DIS
    };

    /*本控制器主函数*/
    void control_formation() override;

    /*重置控制器*/
    void reset_formation_controller();

    /* 设定编队控制器参数（主管产生期望空速） */
    void set_mix_Xerr_params(struct _s_mix_Xerr_params &input_params);

    /* 设定编队控制器参数（主管产生期望滚转角） */
    void set_mix_Yerr_params(struct _s_mix_Yerr_params &input_params);

    /* 设定TECS控制器参数 */
    void set_tecs_params(struct _s_tecs_params &input_params);

    /* 得到前向编队控制器参数 */
    void get_mix_Xerr_params(struct _s_mix_Xerr_params &mix_Xerr_para);

    /* 得到侧向编队控制器参数 */
    void get_mix_Yerr_params(struct _s_mix_Yerr_params &mix_Yerr_para);

private:
    /**
   *编队控制器外函数，变量（组）
   */

    /* 绝对速度位置控制器时间戳 */
    long abs_pos_vel_ctrl_timestamp{0};

    /* 滤波后的领机信息 */
    _s_leader_states leader_states_f;

    /* 滤波后的从机信息 */
    _s_fw_states fw_states_f;

    /* 完成对于领机从机的滤波函数 */
    void filter_led_fol_states();

    /* 是否使用滤波器对原始数据滤波 */
    bool use_the_filter{true};
    
    /* 领机gol速度x滤波器 */
    FILTER led_gol_vel_x_filter;

    /* 领机gol速度y滤波器 */
    FILTER led_gol_vel_y_filter;

    /* 检验计算本机的空速（状态）以及实际读取的空速的合法性 */
    bool fw_airspd_states_valid{true};

    /* 检验计算领机的空速（状态）以及实际读取的空速的合法性 */
    bool led_airspd_states_valid{true};

    /* 领机空速向量 */
    Vec led_arispd;

    /* 领机地速向量 */
    Vec led_gspeed_2d;

    /* 本机空速向量 */
    Vec fw_arispd;

    /* 本机地速向量 */
    Vec fw_gspeed_2d;

    /* 本机风估计向量 */
    Vec fw_wind_vector;

    /* 领机dir_cos，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double led_cos_dir{0.0};

    /* 领机dir_sin，这其中的dir这个角度，可能是yaw，也可能是速度偏角*/
    double led_sin_dir{0.0};

    /* 本机dir_cos，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double fw_cos_dir{0.0};

    /* 本机dir_sin，这其中的dir这个角度，可能是yaw，也可能是速度偏角*/
    double fw_sin_dir{0.0};

    /* 编队控制器分段 */
    _e_format_method format_method;

    /* 编队控制器混合误差产生参数,编队控制器参数 */
    _s_mix_Xerr_params mix_Xerr_params;

    /*编队控制器Y方向混合误差产生参数*/
    _s_mix_Yerr_params mix_Yerr_params;

    /*产生期望地速的pid*/
    INCREMENT_PID_CONTROLLER gspeed_sp_pid;

    /* 重置内部控器标志量 */
    bool rest_speed_pid{false};

    /* 从机期望地速增量，最终实现的是领机与从机地速一致 */
    float del_fol_gspeed{0.0};

    /* 飞机期望空速（前一时刻） */
    float airspd_sp_prev{0.0};

    /* 飞机期望空速 */
    float airspd_sp{0.0};

    /* 本机空速期望值滤波器 */
    FILTER airspd_sp_filter;

    /**
   * TECS函数，变量（组）
   */

    /* TECS控制器 */
    TECS _tecs;

    /* 重置TECS */
    bool rest_tecs{false};

    /* 纵向速度有效标志位 */
    bool vz_valid{false};

    /* TECS参数 */
    _s_tecs_params tecs_params;

    /**
   * 横侧向控制器函数，变量（组）
   */

    /* 横侧向控制器 */
    L1_CONTROLLER l1_controller;

    /* 横侧向控制器参数 */
    _s_lateral_controller_params lateral_controller_params;

    /*产生期望滚转角的pid*/
    INCREMENT_PID_CONTROLLER roll_sp_pid;

    /* 重置横侧向控制器 */
    bool reset_lateral_controller{false};

    /* 最终roll通道控制量 */
    float roll_cmd{0.0};

    /* 最终roll通道控制量 */
    float roll_cmd_prev{0.0};
    
    /* 期望滚转角滤波器 */
    FILTER roll_cmd_filter;

    /**
   * 其他计算函数，变量（组）
   */
    /* 原始信息预处理 */
    Point get_plane_to_sp_vector(Point origin, Point target);
};

#endif
