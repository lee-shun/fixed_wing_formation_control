/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  普通位置式pid
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-10 01:02:04
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H

#include <iostream>
#include "mathlib.hpp"
#include "syslib.hpp"

using namespace std;

class PID_CONTROLLER
{
private:
    //时间参数
    float current_time{0};

    float last_time{0};

    float _dt{0};

    float _dt_default{0.2};

    float _dt_max{0.1};

    float _dt_min{0.01};

    //积分微分参数

    float input{0};

    float last_input{0};

    float output{0};

    float integ{0};

    float diffe{0};

    float integ_max{500};

    float integ_min{-500};

    //基本参数

    bool direction{true}; //方向

    float kp{0};

    float ki{0};

    float kd{0};

    //由输入计算的比例，积分，微分的的成分
    float ele_p{0};

    float ele_i{0};

    float ele_d{0};

    float ele_d_max{500};

    float ele_d_min{-500};

public:
    //功能函数

    //计时函数，计算时间间隔
    void cal_time_interval();

    //经典pid
    float pid_classic();

    //抗饱和积分 pid
    float pid_anti_saturated(float input_val, bool use_integ, bool use_diff);

    //增量式 pid
    float pid_incremental();

    //配置pid
    void init_pid(float in_kp, float in_ki, float in_kd)
    {
        kp = in_kp;

        ki = in_ki;

        kd = in_kd;
    }

    //重置pid控制器，尤其是在进行新的控制的时候
    void reset_pid()
    {
        current_time = 0;

        last_time = 0;

        _dt = 0;

        integ = 0;
    }

    //获取积分值
    float get_integ()
    {
        return integ;
    }
};

void PID_CONTROLLER::cal_time_interval()
{

    _dt = current_time - last_time;

    _dt = constrain(_dt, _dt_min, _dt_max);
}

float PID_CONTROLLER::pid_anti_saturated(float input_val, bool use_integ, bool use_diff)
{
    current_time = get_sys_time() / 1000; //获取系统时间，单位为毫秒，需要转化成秒
    input = input_val;

    cal_time_interval();

    ele_p = input * kp;

    //if (direction)
    ele_d = kd * (input - last_input) / _dt;

    //下面抗积分饱和环节计算混合误差的积分值
    if (integ > integ_max) //抗积分饱和
    {
        if (input < 0)
        {
            integ = integ + input;
        }
        else
        {
            integ = integ;
        }
    }
    else if (integ < integ_min)
    {
        if (input > 0)
        {
            integ = integ + input;
        }
        else
        {
            integ = integ;
        }
    }
    else //落在合理区间之内
    {
        integ = integ + input;
    }

    ele_i = ki * integ;

    if (!use_integ) //使用积分开关，距离比较小的时候使用
    {
        ele_i = 0;
    }

    if (!use_diff) //使用微分开关，距离比较小的时候使用
    {
        ele_d = 0;
    }
    output = ele_p + ele_i + ele_d;

    //为下一次做好准备
    last_time = current_time;

    last_input = input;

    return output;
}

#endif // PID_CONTROLLER_H
