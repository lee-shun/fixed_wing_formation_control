#include "task_main.hpp"
float TASK_MAIN::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void TASK_MAIN::fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    fwstates = *msg;
}
void TASK_MAIN::leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg)
{
    leaderstates = *msg;
}

void TASK_MAIN::ros_sub_pub()
{
    //1.【订阅】固定翼全部状态量
    fw_states_sub = nh.subscribe<fixed_wing_formation_control::FWstates>("fixed_wing_formation_control/fw_states", 10, &TASK_MAIN::fw_state_cb, this);

    //2. 【订阅】领机信息
    leader_states_sub = nh.subscribe<fixed_wing_formation_control::Leaderstates>("fixed_wing_formation_control/leader_states", 10, &TASK_MAIN::leader_states_cb, this);

    //3.【订阅】监控节点飞机以及任务状态

    //4.【发布】固定翼四通道控制量
    fw_cmd_pub = nh.advertise<fixed_wing_formation_control::FWcmd>("/fixed_wing_formation_control/fw_cmd", 10);

    //5.【发布】编队控制器状态
    formation_control_states_pub = nh.advertise<fixed_wing_formation_control::Formation_control_states>("/fixed_wing_formation_control/formation_control_states", 10);
}

void TASK_MAIN::control_formation()
{
    fw_col_mode_current = fwstates.control_mode;
    if (fw_col_mode_current != fw_col_mode_last) //模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！
    {
        formation_controller.reset_formation_controller();
    }
    //领机状态赋值
    leader_states.air_speed = leaderstates.airspeed;

    leader_states.altitude = leaderstates.altitude;
    leader_states.latitude = leaderstates.latitude;
    leader_states.longtitude = leaderstates.longtitude;
    leader_states.relative_alt = leaderstates.relative_alt;

    leader_states.ned_vel_x = leaderstates.ned_vel_x;
    leader_states.ned_vel_y = leaderstates.ned_vel_y;
    leader_states.ned_vel_z = leaderstates.ned_vel_z;

    leader_states.pitch_angle = leaderstates.pitch_angle;
    leader_states.roll_angle = leaderstates.roll_angle;
    leader_states.yaw_angle = leaderstates.yaw_angle;

    //从机状态赋值
    thisfw_states.air_speed = fwstates.air_speed;
    thisfw_states.in_air = fwstates.in_air;

    thisfw_states.altitude = fwstates.altitude;
    thisfw_states.altitude_lock = fwstates.altitude_lock;
    thisfw_states.latitude = fwstates.latitude;
    thisfw_states.longtitude = fwstates.longtitude;
    thisfw_states.relative_alt = fwstates.relative_alt;

    thisfw_states.ned_vel_x = fwstates.ned_vel_x;
    thisfw_states.ned_vel_y = fwstates.ned_vel_y;
    thisfw_states.ned_vel_z = fwstates.ned_vel_z;

    thisfw_states.pitch_angle = fwstates.pitch_angle;
    thisfw_states.roll_angle = fwstates.roll_angle;
    thisfw_states.yaw_angle = fwstates.yaw_angle;
    thisfw_states.att_quat[0] = fwstates.att_quater.w;
    thisfw_states.att_quat[1] = fwstates.att_quater.x;
    thisfw_states.att_quat[2] = fwstates.att_quater.y;
    thisfw_states.att_quat[3] = fwstates.att_quater.z;
    quat_2_rotmax(thisfw_states.att_quat, thisfw_states.rotmat);

    thisfw_states.body_acc[0] = fwstates.body_acc_x;
    thisfw_states.body_acc[1] = fwstates.body_acc_y;
    thisfw_states.body_acc[2] = fwstates.body_acc_z;
    matrix_plus_vector_3(thisfw_states.ned_acc, thisfw_states.rotmat, thisfw_states.body_acc);

    thisfw_states.wind_estimate_x = fwstates.wind_estimate_x;
    thisfw_states.wind_estimate_y = fwstates.wind_estimate_y;
    thisfw_states.wind_estimate_z = fwstates.wind_estimate_z;

    //设定编队形状
    formation_controller.set_formation_type(2);
    //选定控制器类型，并进行控制
    formation_controller.abs_pos_vel_controller(leader_states, thisfw_states);
    //获得最终控制量
    formation_cmd = formation_controller.get_formation_4cmd();
    //控制量赋值
    fw_4cmd.throttle_sp = formation_cmd.thrust;
    fw_4cmd.roll_angle_sp = formation_cmd.roll;
    fw_4cmd.pitch_angle_sp = formation_cmd.pitch;
    fw_4cmd.yaw_angle_sp = formation_cmd.yaw;

    fw_cmd_pub.publish(fw_4cmd); //发布四通道控制量

    fw_col_mode_last = fw_col_mode_current; //上一次模式的纪录
}

void TASK_MAIN::run()
{
    ros::Rate rate(10.0);
    begin_time = ros::Time::now(); // 记录启控时间
    ros_sub_pub();

    while (ros::ok() && need_task)
    {
        /*执行比赛大逻辑，并非执行一次，飞机如果进入了失控保护又进入正常，可以再次进行编队控制*/

        while (ros::ok() && need_take_off && fw_is_ok)
        {
            /*起飞代码*/
        }

        while (ros::ok() && need_control_formation && fw_is_ok) //控制编队控制跟随
        {
            /*编队控制代码*/
            current_time = get_ros_time(begin_time); //此时的时间，只作为纪录，不用于控制

            if (true) //监控节点的并没有发现飞机完成时间，距离任务
            {
                cout << "编队启控时间：[" << current_time << "]秒" << endl;
                control_formation();
            }

            ros::spinOnce();
            rate.sleep();
        }

        while (ros::ok() && need_landing && fw_is_ok)
        {
            /*降落代码*/
        }

        while (ros::ok() && (!fw_is_ok))
        {
            /*失控保护代码*/
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "task_main");
    TASK_MAIN _task_main;
    if (true)
    {
        _task_main.run();
    }
    return 0;
}