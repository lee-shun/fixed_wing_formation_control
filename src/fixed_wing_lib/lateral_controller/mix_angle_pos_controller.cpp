/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 23:52:52
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:57:02
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "mix_angle_pos_controller.hpp"

void MIX_ANGLE_POS_CONTROLLER::update_angle_pos_err(float angle_err,
                                                    float pos_err) {
  angle_error = angle_err;

  pos_error = pos_err;
}
void MIX_ANGLE_POS_CONTROLLER::set_mix_params(float kp_p, float kangle_p) {
  mix_params.kp_p = kp_p;
  mix_params.kangle_p = kangle_p;
}
void MIX_ANGLE_POS_CONTROLLER::set_pid_params(float mix_kp, float mix_ki,
                                             float mix_kd) {
  mix_params.mix_kp = mix_kp;
  mix_params.mix_ki = mix_ki;
  mix_params.mix_kd = mix_kd;
}
float MIX_ANGLE_POS_CONTROLLER::mix_angle_pos_controller() {
  cout << "in the mix_angle_pos_controller!!!!" << endl;
  return 0.0;
}
