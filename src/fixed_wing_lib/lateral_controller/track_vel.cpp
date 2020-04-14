/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-13 23:34:26
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-13 23:56:43
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "track_vel.hpp"
/**
 * @Input: void
 * @Output: void
 * @Description: 速度追踪法（看导弹飞行力学）
 */
void TRACK_VEL::track_velocity(Point curr_pos, Point sp_pos, Point ground_speed_2d, Point target_speed_2d)
{
    Point vector_curr_position = curr_pos;
    Point vector_A = sp_pos;

    /* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
    float ground_speed = max(ground_speed_2d.len(), 0.1f);

    /* 计算目标点到飞机的向量，ned */
    Point vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position); /*A-->P*/
    cout << "vector_A_to_airplane.x == " << vector_A_to_airplane.x << endl;
    cout << "vector_A_to_airplane.y == " << vector_A_to_airplane.y << endl;

    float r = vector_A_to_airplane.len(); /*A-->P的长度，r*/
    float v = ground_speed_2d.len();      /*自己的速度（标量）*/
    float vt = target_speed_2d.len();     /*目标的速度（标量）（领机的，也是目标位置的速度，因为要编队）*/

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

    float sin_q = grou_speed_unit ^ sp_speed_unit; /*计算出两个速度的夹角的sin_q*/
    cout << "sin_q" << sin_q << endl;

    _lateral_accel = track_vel_k * (v * vt * sin_q) / r;
    cout << "_lateral_accel" << _lateral_accel << endl;
    //
}
