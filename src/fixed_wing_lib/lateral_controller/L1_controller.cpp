/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 23:01:29
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:55:42
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "L1_controller.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description: 
 */
Point L1_CONTROLLER::get_local_planar_vector(Point origin, Point target)
{
    /* this is an approximation for small angles, proposed by [2] */

    Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

    return out * double(CONSTANTS_RADIUS_OF_EARTH);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 使用的是魔改的l1控制器，此中的l1_distance就是飞机与期望点的连线
    创建三个变量，分别代表当前速度与L1方向的夹角；与期望航航点连线方向的位置差；在期望航向上的位置差 
 *  纯L1控制器，飞往航点的算法，收敛效果不错
 */
void L1_CONTROLLER::l1_controller(Point curr_pos, Point sp_pos,
                                  Point ground_speed_2d, float airspeed)
{

    float eta;
    float xtrack_vel;
    float ltrack_vel;

    Point vector_curr_position = curr_pos;
    Point vector_A = sp_pos;

    /* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
    float ground_speed = max(ground_speed_2d.len(), 0.1f);

    _L1_distance = _L1_ratio * ground_speed;

    /* 计算目标点到飞机的向量，ned */
    Point vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position);

    /* calculate eta to fly to waypoint A */

    /* unit vector from waypoint A to current position */
    Point vector_A_to_airplane_unit = vector_A_to_airplane.normalized();

    /* velocity across / orthogonal to line */
    xtrack_vel = ground_speed_2d ^ (-vector_A_to_airplane_unit);
    /* velocity along line */
    ltrack_vel = ground_speed_2d * (-vector_A_to_airplane_unit);

    eta = atan2f(xtrack_vel, ltrack_vel);
    /* bearing from current position to L1 point */
    _nav_bearing = atan2f(-vector_A_to_airplane_unit.y, -vector_A_to_airplane_unit.x);

    /* limit angle to +-90 degrees */
    eta = constrain(eta, (-PI) / 2.0f, +PI / 2.0f);
    /**
     * 计算切向加速度,此处的L1_distance是否替换成vector_A_to_airplane.len()有待之后检验
    */
    _lateral_accel = _K_L1 * ground_speed * ground_speed / vector_A_to_airplane.len() * sinf(eta);
}
