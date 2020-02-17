/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 *  本程序是基于L1控制器的改动版本，输入是飞机的当前的位置，当前的地速
 *  (向量），空速，目标位置点，输出的是期望的偏航角以及期望的滚转角
 *  估计应该会写几个种类的横向控制器，看最后用那个吧。。。。。。。。
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-17 22:10:59
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description: TODO:更改此部分的函数设置，当飞机的距离远以及距离近的时候分别考虑 
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _LATERAL_CONTROLLER_HPP_
#define _LATERAL_CONTROLLER_HPP_

#include <iostream>
#include <math.h>
#include "mathlib.hpp"
#include "vector.hpp"

using namespace std;

class LATERAL_CONTROLLER
{
private:
    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/
    float acc_lateral{0};

    float _nav_bearing;

    float _lateral_accel{0};

    float _K_L1{2};

    float _roll_lim_rad{deg_2_rad(60)};

    float roll_sp{0};

    float _L1_period{25};

    float _L1_ratio{5.0};

    float _L1_damping{0.75};

    float _L1_distance{20.0};

    struct _s_control_lateral_params
    {
        float kp{0.1};

        float kd{0.1};
    } control_lateral_params;

    float track_vel_k{1};
    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/

    ///////////速度追踪法/////////////

    ///////////速度追踪法/////////////

public:
    ///////////视线与速度夹角控制/////////////
    void lateral_yaw();

    ///////////视线与速度夹角速率控制/////////////
    void lateral_yaw_rate();

    ///////////滑膜控制/////////////
    void lateral_sliding_mode();

    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/
    void lateral_L1_modified(Point curr_pos, Point sp_pos, Point ground_speed_2d, float airspeed);

    Point get_local_planar_vector(Point origin, Point target);

    float get_lateral_roll_sp()
    {
        return roll_sp;
    }

    float get_lateral_acc_lateral()
    {
        return acc_lateral;
    }

    void set_l1_period(float period)
    {
        _L1_period = period;
        /* calculate the ratio introduced in [2] */
        _L1_ratio = 1.0f / PI * _L1_damping * _L1_period;
    }

    void set_l1_damping(float damping)
    {
        _L1_damping = damping;
        /* calculate the ratio introduced in [2] */
        _L1_ratio = 1.0f / PI * _L1_damping * _L1_period;
        /* calculate the L1 gain (following [2]) */
        _K_L1 = 4.0f * _L1_damping * _L1_damping;
    }

    void set_l1_roll_limit(float roll_lim_rad)
    {
        _roll_lim_rad = roll_lim_rad;
    }

    float nav_roll()
    {
        float ret = atanf(_lateral_accel * 1.0f / CONSTANTS_ONE_G);
        ret = constrain(ret, -_roll_lim_rad, _roll_lim_rad);
        return ret;
    }

    float nav_lateral_acceleration_demand()
    {
        return _lateral_accel;
    }

    float nav_bearing()
    {
        return _nav_bearing;
    }
    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/

    ///////////速度追踪法/////////////
    void track_velocity(Point curr_pos, Point sp_pos, Point ground_speed_2d, Point sp_speed_2d);
    ///////////速度追踪法/////////////

    float mix_pos_vel_ctrl(float pos_err_yb, float vel_err_yb, Point ground_speed_2d);
};

#endif