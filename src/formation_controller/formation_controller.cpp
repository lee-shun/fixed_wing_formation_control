/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-08 23:16:38
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "formation_controller.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description: 更新飞机，领机飞行状态
 */
void FORMATION_CONTROLLER::update_led_fol_states(const struct _s_leader_states *leaderstates,
                                                 const struct _s_fw_states *thisfw_states)
{ /* 使用指针，避免内存浪费 */
    leader_states = *leaderstates;
    fw_states = *thisfw_states;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 设定编队期望队形
 */
void FORMATION_CONTROLLER::set_formation_type(int formation_type)
{
    switch (formation_type)
    {
    case 1:
        formation_offset.xb = 0;
        formation_offset.yb = 0;
        formation_offset.zb = 0;

        break;

    case 2:
        formation_offset.xb = -10;
        formation_offset.yb = 0.1;
        formation_offset.zb = 0;

        break;
    }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 主函数，虚函数
 */
void FORMATION_CONTROLLER::control_formation()
{
}

/**
 * @Input: void
 * @Output: void
 * @Description: 设定编队控制器内部飞机模型函数，例如最大滚转角速度等
 */
void FORMATION_CONTROLLER::set_fw_model_params(struct _s_fw_model_params &input_params)
{
    fw_params = input_params;
}

/**
 * @Input: void
 * @Output: bool
 * @Description: 判断飞机传入的状态值是否有问题，是否在飞行之中
 */
bool FORMATION_CONTROLLER::identify_led_fol_states()
{
    if ((leader_states.global_vel_x > 3.0) ||
        (leader_states.global_vel_y > 3.0) ||
        (leader_states.relative_alt > 3.0))
    {
        led_in_fly = true;
    }
    else
    {
        led_in_fly = false;

        FORMATION_CONTROLLER_INFO("警告：领机未在飞行之中");
    }

    if ((fw_states.global_vel_x > 3.0) ||
        (fw_states.global_vel_y > 3.0) ||
        (fw_states.relative_alt > 3.0))
    {
        fol_in_fly = true;
    }
    else
    {
        fol_in_fly = false;

        FORMATION_CONTROLLER_INFO("警告：本机未在飞行之中");
    }

    if (led_in_fly && fol_in_fly)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 获得飞机期望的四通道控制量
 */
void FORMATION_CONTROLLER::get_formation_4cmd(struct _s_4cmd &fw_cmd)
{
    fw_cmd = _cmd;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 得到编队中本机的运动学期望值
 */
void FORMATION_CONTROLLER::get_formation_sp(struct _s_fw_sp &formation_sp)
{
    formation_sp = fw_sp;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 得到编队控制误差
 */
void FORMATION_CONTROLLER::get_formation_error(struct _s_fw_error &formation_error)
{
    formation_error = fw_error;
}
