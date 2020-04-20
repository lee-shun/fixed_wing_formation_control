/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-10 00:50:00
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  增量式pid控制器
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-11 23:00:55
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _INCREMENT_PID_CONTROLLER_HPP_
#define _INCREMENT_PID_CONTROLLER_HPP_

#include "mathlib.hpp"
#include "syslib.hpp"
#include <iostream>

using namespace std;

class INCREMENT_PID_CONTROLLER
{
public:
  void increment_pid(float in, float Kp, float Ki, float Kd);
  float get_full_output();
  float get_inc_output();
  void reset_incre_pid();
  void set_prev_output(float prev);/* 设定上一次的输出值，保证从别的控制逻辑切入时控制连续 */

private:
  float input{0.0};
  float prev_input{0.0};
  float prev2_input{0.0};

  float increment{0.0};
  float output{0.0};
  float prev_output{0.0};
  void update_input();
};

float INCREMENT_PID_CONTROLLER::get_inc_output()
{
    return increment;
}

/**
 * @Input: 
 * @Output: 
 * @Description: 用于第一次进入时与其他控制方式的衔接
 */

void INCREMENT_PID_CONTROLLER::set_prev_output(float prev) {

  prev_output = prev;
}

float INCREMENT_PID_CONTROLLER::get_full_output()
{

  output = prev_output + increment;

  prev_output = output;

  return output;
}

void INCREMENT_PID_CONTROLLER::reset_incre_pid()
{

  prev_input = 0.0;
  prev2_input = 0.0;
  output = 0.0;
}

void INCREMENT_PID_CONTROLLER::update_input()
{

  prev2_input = prev_input;

  prev_input = input;
}

void INCREMENT_PID_CONTROLLER::increment_pid(float in, float Kp, float Ki,
                                              float Kd)
{
  input = in;

  float param_p = Kp * (input - prev_input);
  float param_i = Ki * input;
  float param_d = Kd * (input - 2 * prev_input + prev2_input);

  increment = param_p + param_i + param_d;

  update_input();
}
#endif
