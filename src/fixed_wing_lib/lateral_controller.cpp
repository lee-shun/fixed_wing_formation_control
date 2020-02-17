/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-17 22:09:24
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  横侧向控制器
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-17 22:11:07
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "lateral_controller.hpp"

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
//混合速度误差，
float LATERAL_CONTROLLER::mix_pos_vel_ctrl(float pos_err_yb, float vel_err_yb, Point ground_speed_2d)
{
    /**TODO:
     * 这里的控制所需的误差量是在机体系之中定义的，但是实际上此处的导航所需的期望的加速度应该与地速方向一致（需要的是位置上的变化）
     * V^2/r此式中的V也应当是地速的完整大小，因为是在惯性系之中产生的加速度，所需的加速度方向应该是垂直于地速方向，但是此处产生的
     * 方向是沿着飞机机体侧向，按照此算法算得的加速度应用时与实际设计的方向有至少一个侧滑角的偏差，但是因为飞机的内环应用了所谓
     * ”无侧滑条件“，保证飞机的侧滑角比较小，使得飞机的机体系Xb与空速方向一致，但是此时的地速方向与空速方向的夹角，其实就是体轴系，
     * 或者风轴系与航迹坐标系之间的关系在哥哥参考书之中未被明确定义。
    */
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