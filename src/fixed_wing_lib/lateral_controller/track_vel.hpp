/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 23:34:16
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:56:51
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _TRACK_VEL_HPP_
#define _TRACK_VEL_HPP_

#include "L1_controller.hpp"

class TRACK_VEL : protected L1_CONTROLLER
{
public:
    void track_velocity(Point curr_pos, Point sp_pos, Point ground_speed_2d, Point sp_speed_2d);

private:
    /**
    * 速度追踪法算法参数定义
    */

    float track_vel_k{1};
};

#endif
