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

void TASK_MAIN::ros_sub_pub()
{
    //1.【订阅】固定翼全部状态量
    fw_states_sub = nh.subscribe<fixed_wing_formation_control::FWstates>("fixed_wing_formation_control/fw_states", 10, &TASK_MAIN::fw_state_cb, this);
    //2. 【订阅】领机信息
    //3.【发布】固定翼四通道控制量
    fw_cmd_pub = nh.advertise<fixed_wing_formation_control::FWcmd>("/fixed_wing_formation_control/fw_cmd", 10);
}

void TASK_MAIN::control_formation()
{
    fw_col_mode_current = fwstates.control_mode;
    if (fw_col_mode_current != fw_col_mode_last) //模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！
    {
        formation_controller.reset_formation_controller();
    }

    leader_states.air_speed = 0;
    leader_states.altitude = 0;
    leader_states.latitude = 0;
    leader_states.longtitude = 0;
    leader_states.ned_vel_x = 0;
    leader_states.ned_vel_y = 0;
    leader_states.ned_vel_z = 0;
    leader_states.pitch_angle = 0;
    leader_states.relative_alt = 0;
    leader_states.roll_angle = 0;
    leader_states.yaw_angle = 0;

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

    formation_controller.set_formation_type(2);

    formation_controller.abs_pos_vel_controller(leader_states, thisfw_states);

    formation_cmd = formation_controller.get_formation_4cmd();

    fw_4cmd.throttle_sp = formation_cmd.thrust;
    fw_4cmd.roll_angle_sp = formation_cmd.roll;
    fw_4cmd.pitch_angle_sp = formation_cmd.pitch;
    fw_4cmd.yaw_angle_sp = formation_cmd.yaw;

    fw_cmd_pub.publish(fw_4cmd);//发布四通道控制量

    fw_col_mode_last = fw_col_mode_current; //上一次模式的纪录
}

void TASK_MAIN::run()
{
    ros::Rate rate(50.0);
    begin_time = ros::Time::now(); // 记录启控时间
    ros_sub_pub();

    while (ros::ok())
    {
        current_time = get_ros_time(begin_time); //此时的时间，只作为纪录，不用于控制

        if (need_control_formation)
        {
            control_formation();
        }

        ros::spinOnce();
        rate.sleep();
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