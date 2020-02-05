#ifndef _FORMATION_CONTROL_HPP_
#define _FORMATION_CONTROL_HPP_
/**
 * 本程序的作用是提供几个类型下的编队控制器
 * 例如只有GPS位置，有相对位置以及相对速度，有绝对位置以及绝对速度等
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
    struct _s_formation_params //编队控制器混合误差产生参数,编队控制器参数
    {
        float kv_p{0.5}; //主从机速度差比例项

        float kp_p{0.8}; //从机期望与实际位置误差比例

        float mix_kp{0.6};

        float mix_kd{0.0};

        float mix_ki{0.01};
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

        bool yaw_valid{false}; //有时候不能直接得到领机的航向信息

        float ned_vel_x{0}; //NED速度与GPS速度是不一致的

        float ned_vel_y{0};

        float ned_vel_z{0};

        float global_vel_x{0}; //GPS速度

        float global_vel_y{0};

        float global_vel_z{0};

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

        float ned_vel_x{0}; //NED速度

        float ned_vel_y{0};

        float ned_vel_z{0};

        float global_vel_x{0}; //GPS速度

        float global_vel_y{0};

        float global_vel_z{0};

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

    struct _s_formation_offset //编队队形几何偏移
    {
        //机体系
        float xb{0};
        float yb{0};
        float zb{0};

        //NED系
        float ned_n{0};
        float ned_e{0};
        float ned_d{0};
    };

    struct _s_fw_sp //这个结构体为了区分，角度以及油门的期望值就是单独要发布的，//是由运动学位置以及速度的期望值以及当前飞机的状态，是计算出来的。
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
    };

    struct _s_fw_error //本机误差，包括与领机的偏差
    {
        //ned坐标系之下的位置误差
        float P_N{0};
        float P_E{0};
        float P_D{0};
        float P_NE{0};

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
    };

    struct _s_lateral_controller_params //横侧向控制器参数
    {
        float roll_max{PI / 2};
    };

    struct _s_4cmd //四通道控制量
    {
        float roll{0};
        float pitch{0};
        float yaw{0};
        float thrust{0};
    };

    void set_formation_type(int formation_type); //设定编队形状
    void reset_formation_controller();           //重置控制器，防止不同阶段控制器的状态混乱。

    //几个编队控制器类型,根据能得到的领机信息分类
    void att_vel_pos_controller();                                     //得到领机的位置，速度，姿态
    void abs_pos_vel_controller(struct _s_leader_states leader_states, //
                                struct _s_fw_states fw_states);        //得到领机的绝对位置，绝对速度
    void abs_pos_controller();                                         //得到领机的仅仅有绝对位置
    void rel_pos_vel_controller();                                     //得到领机的相对位置与相对速度
    void rel_pos_controller();                                         //得到领机的相对位置

    struct FORMATION_CONTROL::_s_4cmd get_formation_4cmd();               //得到编队控制后的四通道控制量
    struct FORMATION_CONTROL::_s_fw_sp get_formation_sp();                //得到编队中本机的运动学期望值
    struct FORMATION_CONTROL::_s_fw_error get_formation_error();          //得到编队控制误差
    struct FORMATION_CONTROL::_s_formation_params get_formation_params(); //得到编队控制器参数

private:
    _s_formation_offset formation_offset; //编队偏移量
    _s_formation_params formation_params; //编队控制器混合误差产生参数,编队控制器参数
    _s_fw_sp fw_sp;                       //本机的期望
    _s_fw_error fw_error;                 //本机误差，包括与期望的差和领机的偏差
    double led_cos_yaw;                   //领机yaw_cos
    double led_sin_yaw;                   //领机yaw_sin

    TECS _tecs;                 //TECS
    bool rest_tecs{false};      //重置TECS
    bool vz_valid{false};       //纵向速度有效标志位
    _s_tecs_params tecs_params; //TECS参数

    LATERAL_CONTROLLER _lateral_controller;                 //横侧向控制器
    PID_CONTROLLER gspeed_pid;                              //产生期望地速的pid
    bool rest_speed_pid{false};                             //重置内部控器标志量
    _s_lateral_controller_params lateral_controller_params; //横侧向控制器参数

    _s_4cmd _cmd; //最后的控制量

    Point get_plane_to_sp_vector(Point origin, Point target); //原始信息预处理
    void print_data(struct _s_fw_states *p);                  //测试数据通断
};

#endif