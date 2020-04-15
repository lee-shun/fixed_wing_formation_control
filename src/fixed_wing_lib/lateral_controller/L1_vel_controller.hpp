/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 17:28:07
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:56:26
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _L1_VEL_CONTROLLER_HPP_
#define _L1_VEL_CONTROLLER_HPP_

#include "L1_controller.hpp"

class L1_VEL_CONTROLLERS : public L1_CONTROLLER
{
public:
  /*改进的L1控制器:按照L1的思路，添加了速度的信息*/
  void mix_pos_vel_ctrl(Vec &ground_speed_2d, Vec &fw_unit, Vec &pos_err_vector,
                        Vec &vel_err_vector);

private:
  /*改进的L1算法*/

  float _K_L1_pos{2.0};

  float _K_L1_vel{2.0};

  float lateral_acc_vel{0.0};

  float lateral_acc_pos{0.0};
};

#endif
