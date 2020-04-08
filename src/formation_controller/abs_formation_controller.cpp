/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-08 10:56:50
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-08 22:23:51
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "abs_formation_controller.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description: 以便重置控制器中有“记忆”的量
 */
void ABS_FORMATION_CONTROLLER::reset_formation_controller()
{
    rest_speed_pid = true;
    rest_tecs = true;
}
/**
 * @Input: void
 * @Output: void
 * @Description: 设定abs编队混合误差产生参数
 */
void ABS_FORMATION_CONTROLLER::set_formation_params(struct _s_mix_error_params &input_params)
{
    mix_error_params = input_params;
}
/**
 * @Input: void
 * @Output: void
 * @Description: 设定tecs控制器参数
 */
void ABS_FORMATION_CONTROLLER::set_tecs_params(struct _s_tecs_params &input_params)
{
    tecs_params = input_params;
}
/**
 * @Input: void
 * @Output: void
 * @Description: 设定衡侧向控制器参数
 */
void ABS_FORMATION_CONTROLLER::set_lateral_ctrller_params(struct _s_lateral_controller_params &input_params)
{
    lateral_controller_params = input_params;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 调用滤波器对输入的飞机原始状态进行滤波
 */
void ABS_FORMATION_CONTROLLER::filter_led_fol_states()
{
    fw_states_filtered = fw_states;
    leader_states_filtered = leader_states;

    if (use_the_filter)
    {
        leader_states_filtered.global_vel_x =
            led_gol_vel_x_filter.one_order_filter(leader_states.global_vel_x);

        leader_states_filtered.global_vel_y =
            led_gol_vel_y_filter.one_order_filter(leader_states.global_vel_y);
    }
}
/**
 * @Input: void
 * @Output: void
 * @Description: 计算从飞机当前位置到期望的位置的向量
 */
Point ABS_FORMATION_CONTROLLER::get_plane_to_sp_vector(Point origin, Point target)
{
    Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

    return out * double(CONSTANTS_RADIUS_OF_EARTH);
}
/**
 * @Input: void
 * @Output: void
 * @Description: 获得编队控制器参数
 */
void ABS_FORMATION_CONTROLLER::get_formation_params(struct _s_mix_error_params &mix_error_para)
{
    mix_error_para = mix_error_params;
}