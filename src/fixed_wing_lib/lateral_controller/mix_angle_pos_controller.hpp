/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 23:53:38
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:57:06
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _MIX_ANGLE_POS_CONTROLLER_HPP_
#define _MIX_ANGLE_POS_CONTROLLER_HPP_

#include "../increment_pid_controller.hpp"
#include "../mathlib.hpp"
#include "iostream"

using namespace std;

class MIX_ANGLE_POS_CONTROLLER {
public:
  /*更新位置，角度误差*/
  void update_angle_pos_err(float angle_err, float pos_err);
  /*设定误差混合的比例系数*/
  void set_mix_params(float kp_p, float kangle_p);
  /*设定内部的pid的参数*/
  void set_pid_params(float mix_kp, float mix_ki, float mix_kd);

  /*主控*/
  float mix_angle_pos_controller();

private:
  struct _s_mix_error_params {
    float kp_p{0.0};
    float kangle_p{0.0};

    float mix_kp{0.0};
    float mix_ki{0.0};
    float mix_kd{0.0};
  };
  struct _s_mix_error_params mix_params;

  float angle_error;
  float pos_error;

  INCREMENT_PID_CONTROLLER inc_pid;
};
#endif
