#ifndef _FORMATION_CONTROL_HPP_
#define _FORMATION_CONTROL_HPP_
/*本程序的作用是提供几个类型下的编队控制器
例如只有GPS位置，有相对位置以及相对速度，有绝对位置以及绝对速度等
*/

#include <iostream>
#include <sys/time.h>
#include "../fixed_wing_lib/mathlib.hpp"
#include "../fixed_wing_lib/pid_controller.hpp"
#include "../fixed_wing_lib/vector.hpp"

using namespace std;

class FORMATION_CONTROL
{
public:
    struct _s_formation_controller_states //编队控制器内部的情况
    {
    };
    struct _s_leader_states //领机状态信息
    {
        /* data */
    };

    struct _s_fw_states //本机状态信息
    {
        /* data */
    };
    struct _s_4cmd
    {
        /* data */
    };

    void set_formation_type(); //设定编队形状

    //几个编队控制器类型，abs_pos_vel，pos_only，rel_pos_vel

    void update_controller(_s_leader_states leader_states, _s_fw_states fw_states);

    _s_4cmd get_formation_4cmd(); //得到编队控制后的四通道控制量

    float get_sys_time(); //获取系统时间，单位为秒
private:
    //float get_sys_time(); //获取系统时间，单位为秒
};

#endif