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
 * @LastEditTime: 2020-04-10 01:03:26
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

class INCREMENT_PID_CONTROLLER {
public:
  float increment_pid(float in, float Kp, float Ki, float Kd);
  float get_full_output();
  void reset_incre_pid();

private:
  float _dt{0.0};
  int stamp{0};

  float input{0.0};
  float prev_input{0.0};
  float prev2_input{0.0};

  float increment{0.0};
  float output{0.0};
  void update_input();
};

float INCREMENT_PID_CONTROLLER::get_full_output() {

  output = output + increment;

  return output;
}

void INCREMENT_PID_CONTROLLER::reset_incre_pid() {

  stamp = 0;
  prev_input = 0.0;
  prev2_input = 0.0;
  output = 0.0;
}

void INCREMENT_PID_CONTROLLER::update_input() {

    prev2_input = prev_input;

    prev_input = input;
}

float INCREMENT_PID_CONTROLLER::increment_pid(float in, float Kp, float Ki,
                                              float Kd) {
  input = in;

  static float param_p = Kp * (input - prev_input);
  static float param_i = Ki * input;
  static float param_d = Kd * (input - 2 * prev_input + prev2_input);

  increment = param_p + param_i + param_d;

  return increment;

  update_input();

}
#endif
