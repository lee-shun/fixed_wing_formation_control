/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 17:26:49
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:56:02
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _L1_CONTROLLER_HPP
#define _L1_CONTROLLER_HPP

#include "../mathlib.hpp"
#include "../vector.hpp"
#include <iostream>
#include <math.h>

using namespace std;

#define L1_CONTROLLER_INFO(a) cout << "[L1_CONTROLLER_INFO]:" << a << endl;

class L1_CONTROLLER
{
public:
  /**
   * L1算法参数定义函数区
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

  void set_l1_roll_limit(float roll_lim_rad) { _roll_lim_rad = roll_lim_rad; }

  float nav_roll()
  {
    float ret = atanf(_lateral_accel * 1.0f / CONSTANTS_ONE_G);
    ret = constrain(ret, -_roll_lim_rad, _roll_lim_rad);
    return ret;
  }

  float nav_lateral_acceleration_demand() { return _lateral_accel; }

  float nav_bearing() { return _nav_bearing; }

  Point get_local_planar_vector(Point origin, Point target);

  /*原始L1控制器*/
  void l1_controller(Point curr_pos, Point sp_pos, Point ground_speed_2d,
                     float airspeed);

protected:
  /**
   * L1算法参数定义
   */

  /* 偏航角 */
  float _nav_bearing;

  float _lateral_accel{0};

  float _K_L1{2};

  float _L1_period{25};

  float _L1_ratio{5.0};

  float _L1_damping{0.75};

  float _L1_distance{20.0};

  float _roll_lim_rad{PI / 3};

private:
};

#endif
