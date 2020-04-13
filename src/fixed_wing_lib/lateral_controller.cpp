/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-17 22:09:24
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * 横侧向控制器
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-11 23:03:48
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description: 
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "lateral_controller.hpp"

/**
 * @Input: void
 * @Output: void
 * @Description: 速度追踪法（看导弹飞行力学）
 */
void LATERAL_CONTROLLER::track_velocity(Point curr_pos, Point sp_pos, Point ground_speed_2d, Point target_speed_2d)
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

Point LATERAL_CONTROLLER::get_local_planar_vector(Point origin, Point target)
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
void LATERAL_CONTROLLER::lateral_L1_modified(Point curr_pos, Point sp_pos,
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

/**
 * @Input: void
 * @Output: void
 * @Description: 本函数完成的是L1控制器未包含速度信息的缺点，并且将L1默认的地速与机体重合改进了一下
 */
void LATERAL_CONTROLLER::mix_pos_vel_ctrl(Vec &ground_speed_2d, Vec &fw_unit,
                                          Vec &pos_err_vector, Vec &vel_err_vector)
{

    float ground_speed = max(ground_speed_2d.len(), 0.1f);

    _L1_distance = _L1_ratio * ground_speed; //确定L1distance

    float gspd_Xb = fw_unit * ground_speed_2d; //计算沿着飞机机体方向的地速分量

    float sin_eta = fw_unit ^ pos_err_vector.normalized(); //计算飞机机头与期望航点的夹角sin

    sin_eta = constrain(sin_eta, -1.0, 1.0);

    float lateral_acc_pos = _K_L1_pos * gspd_Xb * gspd_Xb / _L1_distance * sin_eta;

    float vel_err_Yb = fw_unit ^ vel_err_vector;

    //所谓“测速反馈增大阻尼”
    float lateral_acc_vel = _K_L1_vel * gspd_Xb * vel_err_Yb / _L1_distance * sin_eta;

    lateral_acc_vel = 0.0; //调试作用，如果是0，那么效果就会和l1无异

    _lateral_accel = lateral_acc_pos + lateral_acc_vel;
}

