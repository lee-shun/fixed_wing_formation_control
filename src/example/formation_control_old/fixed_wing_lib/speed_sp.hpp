#ifndef _SPEED_SP_H_
#define _SPEED_SP_H_
/*本程序实现的是 根据编队的需要的，
根据领机的速度（基准量），
位置误差等信息计算出从机的期望速度*/

#include <iostream>
#include "mathlib.h"
#include "vector.hpp"
#include "pid_controller.hpp"

using namespace std;

class SPEED_SP
{
public:
    struct _s_error
    {
        float ned_vel_x; //ned速度，北向

        float ned_vel_y;

        float global_pos_lat;

        float global_pos_long;

        float global_pos_alt;

        float distance_level;

        float n_distance;

        float e_distance;

        float distance_3d;
    };

    struct _s_status
    {
        float ned_vel_x; //ned速度，北向

        float ned_vel_y;

        float global_pos_lat;

        float global_pos_long;

        float global_pos_alt;

        float wind_x;

        float wind_y;
    };

    struct _s_SPEED_SP_status
    {

        float ned_vel_x;

        float ned_vel_y;

        float vel_led_fol_x{0}; //ned下，领机和从机速度之差，由speed——sp类产生

        float vel_led_fol_y{0};

        float vel_led_fol_z{0};
    };

    float get_airspeed_sp()
    {
        return airspeed_sp;
    }

    _s_SPEED_SP_status get_sp_status()
    {
        return SPEED_SP_status;
    }

    void calculated_vel_led_fol(SPEED_SP::_s_status follower_status, SPEED_SP::_s_status leader_status);

    void cov_gdsp_2_airsp(SPEED_SP::_s_status follower_status);

    //控制方案：相对位置误差的p控制产生期望速度增量，加到领机的速度之上作为从机的期望速度
    void update_airspeed_pos_p(SPEED_SP::_s_error error, SPEED_SP::_s_status follower_status, SPEED_SP::_s_status leader_status);

    //控制方案：相对位置误差,相对速度误差作为总控制误差，将这个error pid之后产生期望速度
    void update_airspeed_mix_vp(float time, SPEED_SP::_s_error error, SPEED_SP::_s_status follower_status, SPEED_SP::_s_status leader_status);

    void re_cal_speed() { reset_cal_speed = true; }

private:
    PID_CONTROLLER n_pid, e_pid;

    bool first_use{true};

    bool reset_cal_speed{false};

    float current_time{0.0};

    float last_time{0.0};

    float _dt{0.0};

    float integ_e_pv_n{0}; //混合误差积分项

    float integ_e_pv_e{0};

    struct _s_formation_params
    {
        float v_kp1{0.1}; //近距离

        float v_kp2{0.5}; //远距离

        float kv_p{0.5}; //主从机速度差比例项

        float kp_p{0.8}; //从机期望与实际位置误差比例

        float mix_kp{0.3};

        float mix_kd{0.0};

        float mix_ki{0.01};

        double altitude_offset{0};

        double longtitude_offset{0};

        double latitude_offset{0};

    } formation_params;

    float airspeed_sp{0};

    _s_SPEED_SP_status SPEED_SP_status;
};

void SPEED_SP::calculated_vel_led_fol(SPEED_SP::_s_status follower_status, SPEED_SP::_s_status leader_status)
{
    SPEED_SP_status.vel_led_fol_x = leader_status.ned_vel_x - follower_status.ned_vel_x;
    SPEED_SP_status.vel_led_fol_y = leader_status.ned_vel_y - follower_status.ned_vel_y;
}
void SPEED_SP::update_airspeed_pos_p(SPEED_SP::_s_error error, SPEED_SP::_s_status follower_status, SPEED_SP::_s_status leader_status)
{

    calculated_vel_led_fol(follower_status, leader_status);

    if (-3 < error.distance_level && error.distance_level < 3)
    //近距离
    {
        cout << "in the 0.01lei" << endl;
        SPEED_SP_status.ned_vel_x = leader_status.ned_vel_x + 0.01 * error.n_distance;

        SPEED_SP_status.ned_vel_y = leader_status.ned_vel_y + 0.01 * error.e_distance;
    }
    else if (-5 < error.distance_level && error.distance_level < 5)
    //近距离
    {
        cout << "in the 0.03" << endl;
        SPEED_SP_status.ned_vel_x = leader_status.ned_vel_x + 0.03 * error.n_distance;

        SPEED_SP_status.ned_vel_y = leader_status.ned_vel_y + 0.03 * error.e_distance;
    }
    else if (-10 < error.distance_level && error.distance_level < 10)
    //近距离
    {
        cout << "in the 0.05" << endl;
        SPEED_SP_status.ned_vel_x = leader_status.ned_vel_x + 0.05 * error.n_distance;

        SPEED_SP_status.ned_vel_y = leader_status.ned_vel_y + 0.05 * error.e_distance;
    }
    else if (-15 < error.distance_level && error.distance_level < 15)
    //近距离
    {
        cout << "in the 0.08" << endl;
        SPEED_SP_status.ned_vel_x = leader_status.ned_vel_x + 0.08 * error.n_distance;

        SPEED_SP_status.ned_vel_y = leader_status.ned_vel_y + 0.08 * error.e_distance;
    }

    else if (-20 < error.distance_level && error.distance_level < 20)
    //近距离
    {
        cout << "in the 0.1" << endl;
        SPEED_SP_status.ned_vel_x = leader_status.ned_vel_x + formation_params.v_kp1 * error.n_distance;

        SPEED_SP_status.ned_vel_y = leader_status.ned_vel_y + formation_params.v_kp1 * error.e_distance;
    }

    else if (-25 < error.distance_level && error.distance_level < 25)
    //近距离
    {
        cout << "in the 0.2" << endl;
        SPEED_SP_status.ned_vel_x = leader_status.ned_vel_x + 0.2 * error.n_distance;

        SPEED_SP_status.ned_vel_y = leader_status.ned_vel_y + 0.2 * error.e_distance;
    }

    else
    {
        cout << "in the long_distance" << endl;
        SPEED_SP_status.ned_vel_x = leader_status.ned_vel_x + formation_params.v_kp2 * error.distance_level;

        SPEED_SP_status.ned_vel_y = leader_status.ned_vel_y + formation_params.v_kp2 * error.distance_level;
    }
    cov_gdsp_2_airsp(follower_status);
}

void SPEED_SP::cov_gdsp_2_airsp(SPEED_SP::_s_status follower_status)
{
    airspeed_sp = sqrt((SPEED_SP_status.ned_vel_x - follower_status.wind_x) * (SPEED_SP_status.ned_vel_x - follower_status.wind_x) //
                       + (SPEED_SP_status.ned_vel_y - follower_status.wind_y) * (SPEED_SP_status.ned_vel_y - follower_status.wind_y));
}

void SPEED_SP::update_airspeed_mix_vp(float time, SPEED_SP::_s_error error, SPEED_SP::_s_status follower_status, SPEED_SP::_s_status leader_status)
{

    //计算与从机的速度误差；
    current_time = time;
    _dt = current_time - last_time;

    if (first_use)
    {
        _dt = 0.02;
        first_use = false;
    }

    calculated_vel_led_fol(follower_status, leader_status);

    float e_pv_n = formation_params.kp_p * error.n_distance + formation_params.kv_p * SPEED_SP_status.vel_led_fol_x; 
    float e_pv_e = formation_params.kp_p * error.e_distance + formation_params.kv_p * SPEED_SP_status.vel_led_fol_y;
    
    cout<<"error.n_distance==="<<error.n_distance<<endl;
    cout<<"error.e_distance==="<<error.e_distance<<endl;
    //pid控制器：抗击分包和pid，增量式pid等等
    n_pid.init_pid(formation_params.mix_kp, formation_params.mix_ki, formation_params.mix_kd);
    e_pid.init_pid(formation_params.mix_kp, formation_params.mix_ki, formation_params.mix_kd);

    if (reset_cal_speed)
    {
        n_pid.reset_pid();
        e_pid.reset_pid();

        reset_cal_speed = false;
    }

    SPEED_SP_status.ned_vel_x = n_pid.pid_anti_saturated(current_time, e_pv_n);

    SPEED_SP_status.ned_vel_y = e_pid.pid_anti_saturated(current_time, e_pv_e);

    
    cov_gdsp_2_airsp(follower_status);

    last_time = current_time;
}
#endif