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
 * @LastEditTime: 2020-03-17 19:54:40
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description: 
 *  TODO:
 *  这里的控制所需的误差量是在机体系之中定义的，但是实际上此处的导航所需的期望的加速度应该与地速方向一致（需要的是位置上的变化）
 *  V^2/r此式中的V也应当是地速的完整大小，因为是在惯性系之中产生的加速度，所需的加速度方向应该是垂直于地速方向，但是此处产生的
 *  方向是沿着飞机机体侧向，按照此算法算得的加速度应用时与实际设计的方向有至少一个侧滑角的偏差，但是因为飞机的内环应用了所谓
 *  ”无侧滑条件“，保证飞机的侧滑角比较小，使得飞机的机体系Xb与空速方向一致，但是此时的地速方向与空速方向的夹角，其实就是体轴系，
 *  或者风轴系与航迹坐标系之间的关系在各参考书之中未被明确定义。
 *  
 *  ->可以确定的是：
 *  1. 首先明确：攻角以及侧滑角是所谓的气动角，是机体与气流之间的关系
 *  2. 其次:所谓的航机系，那一定是飞机的地速运动矢量相对于惯性系（地面系）的关系，有航迹倾角以及航迹偏角的关系。
 *  3. 因而无风的时候，才会有飞机的空速矢量与地速矢量重合！！！！
 *  
 *  ->推理可以得到：
 *  1. 如果认为空速与地速方向不一致（实际上也不一致），此时，由于“无侧滑条件”的存在，且只考虑水平平面内的运动关系，则会有空
 *  速方向与机体前向一致。即飞机的地速方向将不会沿着飞机机体轴，因而由协调转弯在气流轴系产生的侧向的加速度将不会垂直于飞机的地
 *  速方向，会垂直于飞机的空速方向。此时，混合误差在机体前向，以及高度方向的分量将由空速以及高度控制正确解决。但是在横侧向通道，
 *  此时控制器控制飞机，按照本来应该作用于地速方向的期望加速度an的大小，在飞机的机体系，也就是空速的法向上作用了一个等大小的
 *  加速度。这个实际作用的加速度在地速方向上的分量，将影响到地速的大小，在垂直于地速的方向上，将使飞机的地速的方向发生改变，达到
 *  了一定的制导的目的，但是由于此分量小于原始的期望分量，飞机的最终追踪误差将根据横侧向控制器的鲁棒性产生不同程度的位置误差。以
 *  上为控制量在机体系产生的分析
 *  
 *  
 *  2. 如果认为空速方向与地速方向一致（飞机传感器测得的空速方向有误，或者无风条件，风速太小等等），这时在体轴系中产生的控制量与
 *  在地速中产生的控制量并无区别。
 *  
 *  3. 如果在地速中产生控制量，但是实际的空速与地速不一致的话，地速方向的产生期望速度以及垂直地速方向产生的侧向加速度的期望值将
 *  不能直接作用于飞机的内环，因为机体前向通道控制的是速度，机体侧向通道控制的是加速度，二者在沿空速方向以及空速的法相均有分量，
 *  最终导致每个机体通道拿到的是加速度与速度的混合期望值！！！将不符合控制方法，但是如果直接将空速的方向的控制量作为机体双通道的
 *  控制量的话，将导致控制的不精确。
 *  
 *  结论：在机体系之中产生的控制量，无论是机体前行通道还是侧向通道，如果控制是正确的话，只要在相应的通道产生设定的加速度，就可以
 *  达到相应的控制效果。只是在飞机的机体侧向通道，计算使用的V应该是沿着机体方向的分量，而不是全速度。
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

/**
 * @Input: vel_dir_sp就是期望的速度方向，是否为单位向量均可
 * @Output: 期望法相加速度
 * @Description: 本函数控制的是速度的方向，只控制速度方向
 */
void LATERAL_CONTROLLER::ctrl_vel_direction(Vec &ground_speed_2d, Vec &fw_unit,
                                            Vec &vel_dir_sp)
{
    /**
     * 1.计算领机的地速向量在本机的机体坐标系之中的投影
    */

    static Vec gdSpd2d_unit = ground_speed_2d.normalized();

    static float vel_dir_Xb = gdSpd2d_unit * vel_dir_sp; /*期望速度方向沿着本机速度方向的分量*/
    static float vel_dir_Yb = gdSpd2d_unit ^ vel_dir_sp; /*期望速度方向垂直于本机速度方向的分量*/

    /**
    * 2. 判断期望速度是在哪一个方位（象限）
    */
    figure_out_vel_dir(vel_dir_Xb, vel_dir_Yb);

    /**
     * 3. 计算航迹偏角与期望速度方向角误差
    */

    static float eta_err = 0;

    switch (vel_direction)
    {

    case Whereis_vel_dir_sp::FRONT:
        eta_err = deg_2_rad(0.0);
        break;
    case Whereis_vel_dir_sp::BACK:
        eta_err = deg_2_rad(180.0);
        break;
    case Whereis_vel_dir_sp::LEFT:
        eta_err = deg_2_rad(-90.0);
        break;
    case Whereis_vel_dir_sp::RIGHT:
        eta_err = deg_2_rad(90.0);
        break;
    case Whereis_vel_dir_sp::FRONT_RIGHT:
        
        break;
    case Whereis_vel_dir_sp::BACK_RIGHT:

        break;
    case Whereis_vel_dir_sp::BACK_LEFT:

        break;
    case Whereis_vel_dir_sp::FRONT_LEFT:

        break;
    default:
        LATERAL_CONTROLLER_INFO("期望速度判断方向有误");
        break;
    }
}

void LATERAL_CONTROLLER::figure_out_vel_dir(float X, float Y)
{

    if (X == 0 && Y == 0)
    {
        LATERAL_CONTROLLER_INFO("期望速度有误，均为0");
    }
    else if (X > 0) /*期望速度方向在飞机之前*/
    {
        if (Y > 0) /*右前方*/
        {
            vel_direction = Whereis_vel_dir_sp::FRONT_RIGHT;
        }
        else if (Y < 0) /*左前方*/
        {
            vel_direction = Whereis_vel_dir_sp::FRONT_LEFT;
        }
    }
    else if (X < 0) /*期望速度方向在飞机之后*/
    {
        if (Y > 0) /*右后方*/
        {
            vel_direction = Whereis_vel_dir_sp::BACK_RIGHT;
        }
        else if (Y < 0) /*左后方*/
        {
            vel_direction = Whereis_vel_dir_sp::BACK_LEFT;
        }
    }
    else if (X == 0 && Y != 0) /*在Y轴*/
    {
        if (Y > 0) /*正右方*/
        {
            vel_direction = Whereis_vel_dir_sp::RIGHT;
        }
        else if (Y < 0) /*正左方*/
        {
            vel_direction = Whereis_vel_dir_sp::LEFT;
        }
    }
    else if (X != 0 && Y == 0) /*在X轴*/
    {
        if (X > 0) /*正前方*/
        {
            vel_direction = Whereis_vel_dir_sp::FRONT;
        }
        else if (X < 0) /*正后方*/
        {
            vel_direction = Whereis_vel_dir_sp::BACK;
        }
    }
}
