/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 23:30:23
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:56:17
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "L1_vel_controller.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description:
 * 本函数完成的是L1控制器未包含速度信息的缺点，并且将L1默认的地速与机体重合改进了一下
 */
void L1_VEL_CONTROLLERS::mix_pos_vel_ctrl(Vec &ground_speed_2d, Vec &fw_unit,
                                          Vec &pos_err_vector,
                                          Vec &vel_err_vector)
{

    float ground_speed = max(ground_speed_2d.len(), 0.1f);

    _L1_distance = _L1_ratio * ground_speed; //确定L1distance

    float gspd_Xb = fw_unit * ground_speed_2d; //计算沿着飞机机体方向的地速分量

    float sin_eta =
        fw_unit ^ pos_err_vector.normalized(); //计算飞机机头与期望航点的夹角sin

    sin_eta = constrain(sin_eta, -1.0, 1.0);

    float lateral_acc_pos =
        _K_L1_pos * gspd_Xb * gspd_Xb / _L1_distance * sin_eta;

    float vel_err_Yb = fw_unit ^ vel_err_vector;

    //所谓“测速反馈增大阻尼”
    float lateral_acc_vel =
        _K_L1_vel * gspd_Xb * vel_err_Yb / _L1_distance * sin_eta;

    lateral_acc_vel = 0.0; //调试作用，如果是0，那么效果就会和l1无异

    _lateral_accel = lateral_acc_pos + lateral_acc_vel;
}
