/*
 * @Author: lee-shun 
 * @Date: 2020-02-12 22:39:22 
 * @Last Modified by:   lee-shun 
 * @Last Modified time: 2020-02-12 22:39:22 
 */
#ifndef _LATERAL_CONTROLLER_HPP_
#define _LATERAL_CONTROLLER_HPP_
/*
*作者：lee-shun
*本程序是基于L1控制器的改动版本，输入是飞机的当前的位置，当前的地速
*(向量），空速，目标位置点，输出的是期望的偏航角以及期望的滚转角
*估计应该会写几个种类的横向控制器，看最后用那个吧。。。。。。。。
*
*
*/
#include <iostream>
#include <math.h>
#include "mathlib.hpp"
#include "vector.hpp"

using namespace std;

class LATERAL_CONTROLLER
{
private:
    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/
    float acc_lateral{0};

    float _nav_bearing;

    float _lateral_accel{0};

    float _K_L1{2};

    float _roll_lim_rad{deg_2_rad(60)};

    float roll_sp{0};

    float _L1_period{25};

    float _L1_ratio{5.0};

    float _L1_damping{0.75};

    float _L1_distance{20.0};

    struct _s_control_lateral_params
    {
        float kp{0.1};

        float kd{0.1};
    } control_lateral_params;

    float track_vel_k{1};
    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/

    ///////////速度追踪法/////////////

    ///////////速度追踪法/////////////

public:
    ///////////视线与速度夹角控制/////////////
    void lateral_yaw();

    ///////////视线与速度夹角速率控制/////////////
    void lateral_yaw_rate();

    ///////////滑膜控制/////////////
    void lateral_sliding_mode();

    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/
    void lateral_L1_modified(Point curr_pos, Point sp_pos, Point ground_speed_2d, float airspeed);

    Point get_local_planar_vector(Point origin, Point target);

    float get_lateral_roll_sp()
    {
        return roll_sp;
    }

    float get_lateral_acc_lateral()
    {
        return acc_lateral;
    }

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

    void set_l1_roll_limit(float roll_lim_rad)
    {
        _roll_lim_rad = roll_lim_rad;
    }

    float nav_roll()
    {
        float ret = atanf(_lateral_accel * 1.0f / CONSTANTS_ONE_G);
        ret = constrain(ret, -_roll_lim_rad, _roll_lim_rad);
        return ret;
    }

    float nav_lateral_acceleration_demand()
    {
        return _lateral_accel;
    }

    float nav_bearing()
    {
        return _nav_bearing;
    }
    /***********************************魔改L1控制器函数***************************************/
    /***********************************魔改L1控制器函数***************************************/

    ///////////速度追踪法/////////////
    void track_velocity(Point curr_pos, Point sp_pos, Point ground_speed_2d, Point sp_speed_2d);
    ///////////速度追踪法/////////////
};

void LATERAL_CONTROLLER::lateral_yaw()
{
    // float delat_a_n = control_lateral_params.kp * error_follwer1.n_diatance + control_lateral_params.kd * error_follwer1.ned_vel_x;
    // float delat_a_e = control_lateral_params.kp * error_follwer1.e_distance + control_lateral_params.kd * error_follwer1.ned_vel_y;

    // follower_setpoint.ned_acc_x = delat_a_n + leader_status.ned_acc_x; //测试时领机的加速度为0
    // follower_setpoint.ned_acc_y = delat_a_e + leader_status.ned_acc_y;

    // //将ned下的加速度期望值转换到体轴系下
    // follower_setpoint.body_acc_x = cos(follower_status.yaw_angle) * follower_setpoint.ned_acc_x +
    //                                sin(follower_status.yaw_angle) * follower_setpoint.ned_acc_y;

    // follower_setpoint.body_acc_y = -sin(follower_status.yaw_angle) * follower_setpoint.ned_acc_x +
    //                                cos(follower_status.yaw_angle) * follower_setpoint.ned_acc_y;

    // //去掉x方向的加速度，利用协调转弯，计算滚转角期望值

    // follower_setpoint.roll_angle = rain(atan(follower_setpoint.body_acc_y / ANTS_ONE_G), -1, 1); //atan返回的是弧度制下的-90到90
}

void LATERAL_CONTROLLER::lateral_yaw_rate()
{
    // float last_angle_error = 0.0;

    // float DT = current_time - last_time_lateral;

    // int angle_zone_flag = 0;

    // float angle_error;

    // //将ned下的位置误差转换到体轴系下,得到的是机体系下的目标点的位置坐标
    // error_follwer1.Xb_distance = cos(follower_status.yaw_angle) * error_follwer1.n_diatance +
    //                              sin(follower_status.yaw_angle) * error_follwer1.e_distance;

    // error_follwer1.Yb_distance = -sin(follower_status.yaw_angle) * error_follwer1.n_diatance +
    //                              cos(follower_status.yaw_angle) * error_follwer1.e_distance;

    // cout << "error_follwer1.Xb_distance" << error_follwer1.Xb_distance << endl;

    // cout << "error_follwer1.Yb_distance" << error_follwer1.Yb_distance << endl;

    // float angle_error_raw = atan(error_follwer1.Yb_distance / (error_follwer1.Xb_distance + 0.01)); //注意机体系的坐标

    // if (error_follwer1.Xb_distance >= 0 && error_follwer1.Yb_distance > 0)
    // {
    //     angle_zone_flag = 1;
    //     angle_error = angle_error_raw;
    // }
    // else if (error_follwer1.Xb_distance >= 0 && error_follwer1.Yb_distance < 0)
    // {
    //     angle_zone_flag = 2;
    //     angle_error = deg_2_rad(360) + angle_error_raw; //本身是个负的
    // }
    // else if (error_follwer1.Xb_distance < 0 && error_follwer1.Yb_distance <= 0)
    // {
    //     angle_zone_flag = 3;
    //     angle_error = angle_error_raw + deg_2_rad(180); //////
    // }
    // else
    // {
    //     angle_zone_flag = 4;
    //     angle_error = angle_error_raw + deg_2_rad(180);
    // }
    // cout << "angle_zone_flag====" << angle_zone_flag << endl;
    // cout << "angle_error====" << angle_error << endl;

    // //1和4象限为正，2和3象限为负，和期望产生的加速度的符号相同
    // float angle_error_dot = (angle_error - last_angle_error) / DT;
    // float gama_dot = control_lateral_params.kp * angle_error + control_lateral_params.kd * angle_error_dot;

    // follower_setpoint.roll_angle = rain(atan(gama_dot * ANTS_ONE_G / follower_status.air_speed), -1, 1); //atan返回的是弧度制下的-90到90

    // last_angle_error = angle_error;

    // last_time_lateral = current_time;
}

//速度追踪法（看导弹飞行力学）
void LATERAL_CONTROLLER::track_velocity(Point curr_pos, Point sp_pos, Point ground_speed_2d, Point target_speed_2d)
{
    Point vector_curr_position = curr_pos;
    Point vector_A = sp_pos;

    /* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
    float ground_speed = max(ground_speed_2d.len(), 0.1f);

    /* 计算目标点到飞机的向量，ned */
    Point vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position); //A-->P
    cout << "vector_A_to_airplane.x == " << vector_A_to_airplane.x << endl;
    cout << "vector_A_to_airplane.y == " << vector_A_to_airplane.y << endl;

    float r = vector_A_to_airplane.len(); //A-->P的长度，r
    float v = ground_speed_2d.len();      //自己的速度（标量）
    float vt = target_speed_2d.len();     //目标的速度（标量）（领机的，也是目标位置的速度，因为要编队）

    cout << "r == " << r << endl;
    cout << "v == " << v << endl;
    cout << "vt == " << vt << endl;
    //cout << "vector_A_to_airplane.y == " << vector_A_to_airplane.y << endl;

    Point grou_speed_unit = ground_speed_2d.normalized();
    Point sp_speed_unit = target_speed_2d.normalized();

    cout << "grou_speed_unit.x " << grou_speed_unit.x << endl;
    cout << "grou_speed_unit.y " << grou_speed_unit.y << endl;
    cout << "sp_speed_unit.x " << sp_speed_unit.x << endl;
    cout << "sp_speed_unit.y " << sp_speed_unit.y << endl;

    float sin_q = grou_speed_unit ^ sp_speed_unit; //计算出两个速度的夹角的sin_q
    cout << "sin_q" << sin_q << endl;

    _lateral_accel = track_vel_k * (v * vt * sin_q) / r;
    cout << "_lateral_accel" << _lateral_accel << endl;
    //
}

////////魔改l1控制器
void LATERAL_CONTROLLER::lateral_L1_modified(Point curr_pos, Point sp_pos,
                                             Point ground_speed_2d, float airspeed)
{
    //使用的是魔改的l1控制器，此中的l1_distance就是飞机与期望点的连线
    /* 创建三个变量，分别代表当前速度与L1方向的夹角；与期望航线垂起方向的位置差；在期望航向上的位置差 */

    float eta;
    float xtrack_vel;
    float ltrack_vel;

    Point vector_curr_position = curr_pos;
    Point vector_A = sp_pos;

    /* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
    float ground_speed = max(ground_speed_2d.len(), 0.1f);

    //cout << "ground_speed == " << ground_speed << endl;

    _L1_distance = _L1_ratio * ground_speed;

    //cout << "_L1_distance == " << _L1_distance << endl;

    /* 计算目标点到飞机的向量，ned */
    Point vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position);

    /* calculate eta to fly to waypoint A */
    //cout << "vector_A_to_airplane.x == " << vector_A_to_airplane.x << endl;
    //cout << "vector_A_to_airplane.y == " << vector_A_to_airplane.y << endl;

    /* unit vector from waypoint A to current position */
    Point vector_A_to_airplane_unit = vector_A_to_airplane.normalized();

    //cout << "vector_A_to_airplane_unit.x == " << vector_A_to_airplane_unit.x << endl;
    //cout << "vector_A_to_airplane_unit.y == " << vector_A_to_airplane_unit.y << endl;

    /* velocity across / orthogonal to line */
    xtrack_vel = ground_speed_2d ^ (-vector_A_to_airplane_unit);
    /* velocity along line */
    ltrack_vel = ground_speed_2d * (-vector_A_to_airplane_unit);

    //cout << "xtrack_vel == " << xtrack_vel << endl;
    //cout << "ltrack_vel == " << ltrack_vel << endl;

    eta = atan2f(xtrack_vel, ltrack_vel);
    //cout << "eta == " << rad_2_deg(eta) << endl;
    /* bearing from current position to L1 point */
    _nav_bearing = atan2f(-vector_A_to_airplane_unit.y, -vector_A_to_airplane_unit.x);

    /* limit angle to +-90 degrees */
    eta = constrain(eta, (-PI) / 2.0f, +PI / 2.0f);
    /* 计算切向加速度 */
    _lateral_accel = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);

    //cout << "_lateral_accel == " << _lateral_accel << endl;
}

Point LATERAL_CONTROLLER::get_local_planar_vector(Point origin, Point target)
{
    /* this is an approximation for small angles, proposed by [2] */

    Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

    return out * double(CONSTANTS_RADIUS_OF_EARTH);
}
////////魔改l1控制器

void LATERAL_CONTROLLER::lateral_sliding_mode()
{
}
#endif