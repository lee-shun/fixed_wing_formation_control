#ifndef _FORMATION_CONTROL_HPP_
#define _FORMATION_CONTROL_HPP_
/*本程序的作用是提供几个类型下的编队控制器
例如只有GPS位置，有相对位置以及相对速度，有绝对位置以及绝对速度等
*/

#include <iostream>
#include "../fixed_wing_lib/lateral_controller.hpp"
#include "../fixed_wing_lib/tecs.hpp"
#include "../fixed_wing_lib/mathlib.hpp"
#include "../fixed_wing_lib/pid_controller.hpp"
#include "../fixed_wing_lib/vector.hpp"

using namespace std;

#ifndef the_space_between_lines
#define the_space_between_lines 1  //为了打印中间空格
#define the_space_between_blocks 3 //为了打印中间空格
#endif

class FORMATION_CONTROL
{
public:
    struct _s_formation_controller_states //编队控制器内部的情况
    {
    };

    struct rel_states //领机从机相对状态
    {
        /* data */
    };

    struct _s_leader_states //领机状态信息
    {
        float pitch_angle{0}; //姿态只有姿态那个控制器才可能用到

        float yaw_angle{0};

        float roll_angle{0};

        float ned_vel_x{0}; //由于NED以及GPS坐标系系均为惯性系，两个速度是一致的

        float ned_vel_y{0};

        float ned_vel_z{0};

        double latitude{0};

        double longtitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};
    };

    struct _s_fw_states //本机状态信息
    {
        float pitch_angle{0};

        float yaw_angle{0};

        float roll_angle{0};

        float att_quat[4];

        float ned_vel_x{0}; //由于NED以及GPS坐标系系均为惯性系，两个速度是一致的

        float ned_vel_y{0};

        float ned_vel_z{0};

        float body_acc[3];

        float ned_acc[3]; //内部计算后填充，或者外部填充均可

        float rotmat[3][3];

        double latitude{0};

        double longtitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float wind_estimate_x{0};

        float wind_estimate_y{0};

        float wind_estimate_z{0};

        bool in_air{true};

        bool altitude_lock{false};
    };

    struct _s_fw_error //本机误差
    {
        //体轴系位置误差<与自己期望>
        float PXb{0};
        float PYb{0};
        float PZb{0};

        //体轴系速度误差<与自己期望>
        float VXb{0};
        float VYb{0};
        float VZb{0};

        //体轴系速度误差<与领机>
        float led_fol_vxb{0};
        float led_fol_vyb{0};
        float led_fol_vzb{0};
    };

    struct _s_4cmd
    {
        float roll{0};
        float pitch{0};
        float yaw{0};
        float thrust{0};
    };

    void set_formation_type(int formation_type); //设定编队形状

    void reset_formation_controller(); //重置控制器，防止不同阶段控制器的状态混乱。

    struct FORMATION_CONTROL::_s_4cmd get_formation_4cmd(); //得到编队控制后的四通道控制量

    //几个编队控制器类型,根据能得到的领机信息分类

    //得到领机的位置，速度，姿态
    void att_vel_pos_controller();
    //得到领机的绝对位置，绝对速度
    void abs_pos_vel_controller(struct FORMATION_CONTROL::_s_leader_states leader_states,
                                struct FORMATION_CONTROL::_s_fw_states fw_states);
    //得到领机的仅仅有绝对位置
    void abs_pos_controller();
    //得到领机的相对位置与相对速度
    void rel_pos_vel_controller();
    //得到领机的相对位置
    void rel_pos_controller();

private:
    bool rest_speed_pid{false}; //重置内部控器标志量
    bool rest_tecs{false};

    Point get_plane_to_sp_vector(Point origin, Point target); //原始信息预处理
    _s_fw_error fw_error;

    TECS _tecs; //TECS
    struct TECS::tecs_state tecs_outputs;

    PID_CONTROLLER gspeed_pid; //横侧向控制器
    LATERAL_CONTROLLER _lateral_controller;

    _s_4cmd _cmd; //最后的控制量

    void print_data(FORMATION_CONTROL::_s_fw_states *p); //测试数据通断
    struct _s_formation_offset                           //编队队形集合位移
    {
        //机体系
        float xb{0};
        float yb{0};
        float zb{0};

        //NED系
        float ned_n{0};
        float ned_e{0};
        float ned_d{0};
    } formation_offset;

    //这个结构体为了区分，将角度以及油门就是单独的，由运动学位置以及速度的期望值，是计算出来的
    struct _s_fw_sp
    {
        float ned_vel_x{0};

        float ned_vel_y{0};

        float ned_vel_z{0};

        double latitude{0};

        double longtitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float ground_speed{0};
    } fw_sp;

    struct _s_formation_params //编队控制器混合误差产生参数
    {
        float kv_p{0.5}; //主从机速度差比例项

        float kp_p{0.8}; //从机期望与实际位置误差比例

        float mix_kp{0.6};

        float mix_kd{0.0};

        float mix_ki{0.01};
    } formation_params;

    struct _s_tecs_params //TECS控制器参数
    {
        int EAS2TAS{1};

        bool climboutdem{false};

        float climbout_pitch_min_rad{0.2};

        float throttle_min{0.1};

        float throttle_max{1};

        float throttle_cruise{0.1};

        float pitch_min_rad{-0.5};

        float pitch_max_rad{0.5};

        float speed_weight{1};

        float time_const_throt{1.0};

        float time_const{5.0};

    } tecs_params;

    struct _s_lateral_controller_params //横侧向控制器参数
    {
        float roll_max{PI / 2};
    } lateral_controller_params;
};

#endif