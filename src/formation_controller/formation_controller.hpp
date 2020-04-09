/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 *  本程序的作用是作为不同类型编队控制器的父类
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-10 01:04:44
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _FORMATION_CONTROLLER_HPP_
#define _FORMATION_CONTROLLER_HPP_

#include <iostream>
#include "../fixed_wing_lib/lateral_controller.hpp"
#include "../fixed_wing_lib/tecs.hpp"
#include "../fixed_wing_lib/mathlib.hpp"
#include "../fixed_wing_lib/pid_controller.hpp"
#include "../fixed_wing_lib/vector.hpp"
#include "../fixed_wing_lib/filter.hpp"
#include "../fixed_wing_lib/increment_pid_controller.hpp"

using namespace std;

#ifndef the_space_between_lines
#define the_space_between_lines 1  /* 为了打印中间空格 */
#define the_space_between_blocks 3 /* 为了打印中间空格 */
#define FORMATION_CONTROLLER_INFO(a) cout << "[FORMATION_CONTROLLER_INFO]:" << a << endl
#endif

class FORMATION_CONTROLLER
{
public:
    /**
    * 控制器重要的结构体，承担着数据载体与容器的作用、
    * 将控制器内部的数据规整，方便传递与维护
    * 十分重要的数据桥梁，写成public为了外部访问结构体的声明
    */

    struct _s_fw_model_params /* 一系列的飞机动力学模型参数 */
    {
        /*for tecs use*/

        float throttle_min{0.1}; /* 最小油门 */

        float throttle_max{1}; /* 最大油门 */

        float throttle_cruise{0.3}; /* 巡航油门 */

        float pitch_min_rad{-PI / 4}; /* 最小俯仰角rad */

        float pitch_max_rad{PI / 4}; /* 最大俯仰角rad */

        float pitch_rate_max{PI / 3}; /* 最大俯仰角速度 */

        /* for later_controller use*/

        float roll_max{PI / 2}; /* 最大滚转角rad */

        float roll_rate_max{PI / 3}; /* 最大滚转角速度rad/s */

        /*for generate the airspeed setpoint use*/

        float maxinc_acc{10.0}; /* 飞机前向最大瞬时加速度 */

        float maxdec_acc{10.0}; /* 飞机减速最大瞬时加速度 */

        float max_arispd_sp{25.0}; /* 飞机空速最大设定值,此处的最大速度，一定要和飞机的最快速度贴合，否则容易造成油门抖动 */

        float min_arispd_sp{8.0}; /* 飞机空速最小设定值 */
    };

    struct _s_mix_error_params /* 编队控制器混合误差产生参数,编队控制器参数 */
    {
        float kv_p{0.2}; /* 主从机速度差比例项 */

        float kp_p{0.5}; /* 从机期望与实际位置误差比例 */

        float mix_kp{0.4}; /* 总混合产生期望空速pid参数 */

        float mix_kd{0.0}; /* 总混合产生期望空速pid参数 */

        float mix_ki{0.0}; /* 总混合产生期望空速pid参数 */
    };

    struct _s_rel_states /* 领机从机相对状态 */
    {
        /* data */
    };

    struct _s_leader_states /* 领机状态信息 */
    {
        float pitch_angle{0}; /* 姿态只有姿态那个控制器才可能用到 */

        float yaw_angle{0};

        float roll_angle{0};

        bool yaw_valid{false}; /* 有时候不能直接得到领机的航向信息 */

        float ned_vel_x{0}; /* NED速度与GPS速度是不一致的 */

        float ned_vel_y{0};

        float ned_vel_z{0};

        float global_vel_x{0}; /* GPS速度 */

        float global_vel_y{0};

        float global_vel_z{0};

        double latitude{0};

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float wind_estimate_x{0};

        float wind_estimate_y{0};

        float wind_estimate_z{0};
    };

    struct _s_fw_states /* 本机状态信息 */
    {
        string flight_mode;

        float pitch_angle{0};

        float yaw_angle{0};

        float roll_angle{0};

        float att_quat[4];

        float ned_vel_x{0}; /* NED速度 */

        float ned_vel_y{0};

        float ned_vel_z{0};

        float global_vel_x{0}; /* GPS速度 */

        float global_vel_y{0};

        float global_vel_z{0};

        float body_acc[3];

        float ned_acc[3]; /* 内部计算后填充，或者外部填充均可 */

        float rotmat[3][3];

        double latitude{0};

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float wind_estimate_x{0};

        float wind_estimate_y{0};

        float wind_estimate_z{0};

        bool in_air{true};

        bool altitude_lock{false};

        bool yaw_valid{true}; /* TODO:添加yaw_valid的判断，因为此是一个十分重要的计算量，将来的控制量基本与之有关 */
    };

    struct _s_formation_offset /* 编队队形几何偏移 */
    {
        /* 机体系 */
        float xb{0};
        float yb{0};
        float zb{0};

        /* 航迹系 */
        float xk{0};
        float yk{0};
        float zk{0};

        /* NED系 */
        float ned_n{0};
        float ned_e{0};
        float ned_d{0};
    };

    struct _s_fw_sp /* 这个结构体为了区分，角度以及油门的期望值就是单独要发布的，是由运动学位置以及速度的期望值以及当前飞机的状态，是计算出来的。 */
    {
        float ned_vel_x{0};

        float ned_vel_y{0};

        float ned_vel_z{0};

        double latitude{0};

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float ground_speed{0};
    };

    struct _s_fw_error /* 本机误差，包括与领机的偏差 */
    {
        /* ned坐标系之下的位置误差 */
        float P_N{0};
        float P_E{0};
        float P_D{0};
        float P_NE{0};

        /* 体轴系位置误差<与自己期望> */
        float PXb{0};
        float PYb{0};
        float PZb{0};

        /* 航迹轴系位置误差<与自己期望> */
        float PXk{0};
        float PYk{0};
        float PZk{0};

        /* 体轴系速度误差<与自己期望> */
        float VXb{0};
        float VYb{0};
        float VZb{0};
        float Vb{0};

        /* 航迹轴速度误差<与自己期望> */
        float VXk{0};
        float VYk{0};
        float VZk{0};
        float Vk{0};

        /* 体轴系速度误差<与领机> */
        float led_fol_vxb{0};
        float led_fol_vyb{0};
        float led_fol_vzb{0};
        float led_fol_vb{0};

        /* 航迹轴系速度误差<与领机> */
        float led_fol_vxk{0};
        float led_fol_vyk{0};
        float led_fol_vzk{0};
        float led_fol_vk{0};

        /* 航迹轴系下的速度角度（方向）误差 */
        float led_fol_eta{0};
    };

    struct _s_tecs_params /* TECS控制器参数 */
    {
        int EAS2TAS{1};

        bool climboutdem{false};

        float climbout_pitch_min_rad{0.2};

        float speed_weight{1};

        float time_const_throt{8.0};

        float time_const{5.0};
    };

    struct _s_lateral_controller_params /*  横侧向控制器参数 */
    {
    };

    struct _s_4cmd /* 四通道控制量 */
    {
        float roll{0};
        float pitch{0};
        float yaw{0};
        float thrust{0};
    };

    /**
    * 控制器初始化、设置函数（组）
    */

    /* 更新领从机状态*/
    void update_led_fol_states(const struct _s_leader_states *leaderstates,
                               const struct _s_fw_states *thisfw_states);

    /* 设定编队形状 */
    void set_formation_type(int formation_type);

    /* 设定飞机模型参数 */
    void set_fw_model_params(struct _s_fw_model_params &input_params);

    /* 领机从机起飞识别函数 */
    bool identify_led_fol_states();

    /**
    * 各个编队控制器主函数
    */

    virtual void control_formation();

    /**
    * 控制输出获取函数（组）
    */
    void get_formation_4cmd(struct _s_4cmd &fw_cmd);               /* 得到编队控制后的四通道控制量 */
    void get_formation_sp(struct _s_fw_sp &formation_sp);          /* 得到编队中本机的运动学期望值 */
    void get_formation_error(struct _s_fw_error &formation_error); /* 得到编队控制误差 */

protected:
    float _dt{0.02};    /* 控制时间间隔 */
    float _dtMax{0.1};  /* 控制时间间隔max */
    float _dtMin{0.01}; /* 控制时间间隔min */

    _s_formation_offset formation_offset; /* 编队队形偏移量 */
    _s_fw_model_params fw_params;         /* 飞机模型参数 */

    _s_leader_states leader_states; /* 领机状态 */
    _s_fw_states fw_states;         /* 从机状态 */

    bool led_in_fly{false}; /* 领机正在飞行标志位 */
    bool fol_in_fly{false}; /* 从机正在飞行标志位 */

    _s_4cmd _cmd;         /* 最后的控制量 */
    _s_fw_sp fw_sp;       /* 本机的期望 */
    _s_fw_error fw_error; /* 本机误差，包括与期望的差和领机的偏差 */

private:
};

#endif
