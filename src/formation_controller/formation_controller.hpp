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
 * @LastEditTime : 2020-02-15 20:51:59
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _FORMATION_CONTROL_HPP_
#define _FORMATION_CONTROL_HPP_

#include <iostream>
#include "../fixed_wing_lib/lateral_controller.hpp"
#include "../fixed_wing_lib/tecs.hpp"
#include "../fixed_wing_lib/mathlib.hpp"
#include "../fixed_wing_lib/pid_controller.hpp"
#include "../fixed_wing_lib/vector.hpp"
#include "../fixed_wing_lib/filter.hpp"

using namespace std;

#ifndef the_space_between_lines
#define the_space_between_lines 1  //为了打印中间空格
#define the_space_between_blocks 3 //为了打印中间空格
#endif

class FORMATION_CONTROL
{
public:
    /**
    * 控制器重要的结构体，承担着数据载体与容器的作用、
    * 将控制器内部的数据规整，方便传递与维护
    * 十分重要的数据桥梁，写成public为了外部访问结构体的声明
    */
    struct _s_formation_params //编队控制器混合误差产生参数,编队控制器参数
    {
        float kv_p{0.5}; //主从机速度差比例项

        float kp_p{0.8}; //从机期望与实际位置误差比例

        float mix_kp{0.6}; //总混合产生期望空速pid参数

        float mix_kd{0.0}; //总混合产生期望空速pid参数

        float mix_ki{0.01}; //总混合产生期望空速pid参数

        float maxinc_acc{5.0}; //飞机前向最大瞬时加速度

        float maxdec_acc{3.0}; //飞机减速最大瞬时加速度

        float max_arispd_sp{25.0}; //飞机空速最大设定值,此处的最大速度，一定要和飞机的最快速度贴合，否则容易造成油门抖动

        float min_arispd_sp{8.0}; //飞机空速最小设定值
    };

    struct _s_rel_states //领机从机相对状态
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

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float wind_estimate_x{0};

        float wind_estimate_y{0};

        float wind_estimate_z{0};
    };

    struct _s_fw_states //本机状态信息
    {
        string flight_mode;

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

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float wind_estimate_x{0};

        float wind_estimate_y{0};

        float wind_estimate_z{0};

        bool in_air{true};

        bool altitude_lock{false};

        bool yaw_valid{true}; //TODO:添加yaw_valid的判断，因为此是一个十分重要的计算量，将来的控制量基本与之有关
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

        double longitude{0};

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

        float time_const_throt{8.0};

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

    /**
    * 控制器初始化、设置函数（组）
    */

    //更新领从机状态
    void update_led_fol_states(struct _s_leader_states &leaderstates, struct _s_fw_states &thisfw_states);

    //设定是否使用滤波器
    void set_if_use_filter(bool use);

    //设定编队形状
    void set_formation_type(int formation_type);

    //重置控制器，防止不同阶段控制器的状态混乱
    void reset_formation_controller();

    //设定编队控制器参数（主管产生期望空速）
    void set_formation_params(struct _s_formation_params &input_params);

    //设定TECS控制器参数
    void set_tecs_params(struct _s_tecs_params &input_params);

    //设定横侧向控制器参数
    void set_lateral_ctrller_params(struct _s_lateral_controller_params &input_params);

    /**
     * TODO:5种编队控制器类型
     * 几个编队控制器类型,根据能得到的领机信息分类;，
     * 此处传入的领机状态以及本机的状态写成静态变量的形式
     * 保证控制器内不会更改飞机状态信息。
    */

    //1. 输入领机的位置，速度，姿态
    void att_vel_pos_controller();

    //2. 输入领机的绝对位置，绝对速度
    void abs_pos_vel_controller();
    void abs_pos_vel_controller1(); //临时

    //3. 输入领机的仅仅有绝对位置
    void abs_pos_controller();

    //4. 输入领机的相对位置与相对速度
    void rel_pos_vel_controller();

    //5. 输入领机的相对位置
    void rel_pos_controller();

    /**
    * 控制输出获取函数（组）
    */
    void get_formation_4cmd(struct _s_4cmd &fw_cmd);                      //得到编队控制后的四通道控制量
    void get_formation_sp(struct _s_fw_sp &formation_sp);                 //得到编队中本机的运动学期望值
    void get_formation_error(struct _s_fw_error &formation_error);        //得到编队控制误差
    void get_formation_params(struct _s_formation_params &format_params); //得到编队控制器参数

private:
    /**
    * 编队控制器外函数，变量（组）
    */

    long abs_pos_vel_ctrl_timestamp{0};             //绝对速度位置控制器时间戳
    float _dt{0.02};                                //控制时间间隔
    float _dtMax{0.1};                              //控制时间间隔max
    float _dtMin{0.01};                             //控制时间间隔min
    _s_formation_offset formation_offset;           //编队偏移量
    _s_formation_params formation_params;           //编队控制器混合误差产生参数,编队控制器参数
    _s_fw_sp fw_sp;                                 //本机的期望
    _s_fw_error fw_error;                           //本机误差，包括与期望的差和领机的偏差
    double led_cos_yaw{0};                          //领机yaw_cos
    double led_sin_yaw{0};                          //领机yaw_sin
    double fw_cos_yaw{0};                           //本机yaw_cos
    double fw_sin_yaw{0};                           //本机yaw_sin
    float del_fol_gspeed{0};                        //从机期望地速增量，最终实现的是领机与从机地速一致
    float airspd_sp_prev{0};                        //飞机期望空速（前一时刻）
    float airspd_sp{0};                             //飞机期望空速
    FILTER arispd_sp_filter;                        //期望空速滤波器
    bool use_speed_sp_cal();                        //按照距离误差分配，是否启用空速产生的模块
    struct _s_leader_states leader_states;          //领机状态
    struct _s_leader_states leader_states_filtered; //滤波后的领机信息
    struct _s_fw_states fw_states;                  //从机状态
    struct _s_fw_states fw_states_filtered;         //滤波后的从机信息
    bool use_the_filter{true};                      //是否使用滤波器对原始数据滤波
    void filter_led_fol_states();                   //完成对于领机从机的滤波函数
    FILTER led_gol_vel_x_filter;                    //领机gol速度x滤波器
    FILTER led_gol_vel_y_filter;                    //领机gol速度y滤波器
    bool identify_led_fol_states();                 //领机从机起飞识别函数
    bool led_in_fly{false};                         //领机正在飞行标志位
    bool fol_in_fly{false};                         //从机正在飞行标志位

    Vec led_arispd;    //领机空速向量
    Vec led_gspeed_2d; //领机地速向量
    Vec fw_arispd;     //本机空速向量
    Vec fw_gspeed_2d;  //本机地速向量
    Vec fw_wind_vector;   //风估计向量

    /**
    * TECS函数，变量（组）
    */

    TECS _tecs;                 //TECS控制器
    bool rest_tecs{false};      //重置TECS
    bool vz_valid{false};       //纵向速度有效标志位
    _s_tecs_params tecs_params; //TECS参数

    /**
    * 横侧性控制器函数，变量（组）
    */

    LATERAL_CONTROLLER _lateral_controller;                 //横侧向控制器
    PID_CONTROLLER gspeed_pid;                              //产生期望地速的pid
    bool rest_speed_pid{false};                             //重置内部控器标志量
    _s_lateral_controller_params lateral_controller_params; //横侧向控制器参数

    _s_4cmd _cmd; //最后的控制量

    /**
    * 其他计算函数，变量（组）
    */
    Point get_plane_to_sp_vector(Point origin, Point target); //原始信息预处理

    void print_data(const struct _s_fw_states *p); //测试数据通断
};

#endif