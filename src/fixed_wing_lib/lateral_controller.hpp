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
 * @LastEditTime: 2020-02-18 15:45:12
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
    /**
    * L1以及改进的L1算法参数定义
    */

    //原始的L1算法

    float _nav_bearing;
    float _lateral_accel{0};
    float _K_L1{2};
    float _L1_period{25};
    float _L1_ratio{5.0};
    float _L1_damping{0.75};
    float _L1_distance{20.0};
    float _roll_lim_rad{PI / 3};

    //改进的L1算法

    float _K_L1_pos{2.0};
    float _K_L1_vel{2.0};
    float lateral_acc_vel{0.0};
    float lateral_acc_pos{0.0};

    /**
    * 速度追踪法算法参数定义
    */

    float track_vel_k{1};

public:
    /**
    * L1以及改进的L1算法参数定义函数区
    */

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

    Point get_local_planar_vector(Point origin, Point target);

    //原始L1控制器
    void lateral_L1_modified(Point curr_pos, Point sp_pos, Point ground_speed_2d, float airspeed);

    //改进的L1控制器
    float mix_pos_vel_ctrl(Vec &ground_speed_2d, Vec &fw_unit,
                           Vec &pos_err_vector, Vec &vel_err_vector);

    /**
    *速度追踪法
    */

    void track_velocity(Point curr_pos, Point sp_pos, Point ground_speed_2d, Point sp_speed_2d);
};

#endif