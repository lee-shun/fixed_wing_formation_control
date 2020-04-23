/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  比赛任务主程序
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-21 22:27:52
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "task_main.hpp"

/**
 * @Input: int
 * @Output: 
 * @Description: 设定当前飞机的ID
 */
void TASK_MAIN::set_planeID(int id) {
  planeID = id;
  switch (planeID) {
  case 0:
    uavID = "uav0/";
    break;
  case 1:
    uavID = "uav1/";
    break;
  case 2:
    uavID = "uav2/";
    break;
  case 3:
    uavID = "uav3/";
    break;
  }
}
/**
 * @Input: ros::Time begin
 * @Output: float time_now
 * @Description: 获取当前时间
 */
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
void TASK_MAIN::fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg)
{
    fwmonitor_flag = *msg;
}

void TASK_MAIN::fw_cmd_mode_cb(const fixed_wing_formation_control::Fw_cmd_mode::ConstPtr &msg)
{
    fw_cmd_mode = *msg;
}

/**
 * @Input: void
 * @Output: void
 * @Description: ros的订阅发布声明函数
 */
void TASK_MAIN::ros_sub_pub() {

  fw_states_sub = nh.subscribe /* 【订阅】固定翼全部状态量 */
                  <fixed_wing_formation_control::FWstates>(
                      add2str(uavID, "fixed_wing_formation_control/fw_states"),
                      10, &TASK_MAIN::fw_state_cb, this);

  leader_states_sub =
      nh.subscribe /* 【订阅】领机信息 */
      <fixed_wing_formation_control::Leaderstates>(
          add2str(leaderID, "fixed_wing_formation_control/leader_states"), 10,
          &TASK_MAIN::leader_states_cb, this);

  fwmonitor_sub =
      nh.subscribe /* 【订阅】监控节点飞机以及任务状态 */
      <fixed_wing_formation_control::Fwmonitor>(
          add2str(uavID, "fixed_wing_formation_control/fwmonitor_flags"), 10,
          &TASK_MAIN::fw_fwmonitor_cb, this);

  fw_cmd_mode_sub =
      nh.subscribe /* 【订阅】commander指定比赛模式 */
      <fixed_wing_formation_control::Fw_cmd_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_cmd_mode"), 10,
          &TASK_MAIN::fw_cmd_mode_cb, this);

  fw_cmd_pub = nh.advertise /* 【发布】固定翼四通道控制量 */
               <fixed_wing_formation_control::FWcmd>(
                   add2str(uavID, "fixed_wing_formation_control/fw_cmd"), 10);

  formation_control_states_pub =
      nh.advertise /* 【发布】编队控制器状态 */
      <fixed_wing_formation_control::Formation_control_states>(
          add2str(uavID,
                  "fixed_wing_formation_control/formation_control_states"),
          10);

  fw_current_mode_pub =
      nh.advertise /* 【发布】比赛任务进所处阶段 */
      <fixed_wing_formation_control::Fw_current_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_current_mode"), 10);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 编队状态值发布，编队位置误差（机体系和ned），速度误差以及期望空速，gps，期望地速
 */
void TASK_MAIN::formation_states_pub() {
  formation_control_states.planeID = planeID;

  /* 本部分是关于编队的从机的自己与期望值的误差以及领从机偏差的赋值 */
  formation_control_states.err_P_N = formation_error.P_N;
  formation_control_states.err_P_E = formation_error.P_E;
  formation_control_states.err_P_D = formation_error.P_D;
  formation_control_states.err_P_NE = formation_error.P_NE;
  formation_control_states.err_PXb = formation_error.PXb;
  formation_control_states.err_PYb = formation_error.PYb;
  formation_control_states.err_PZb = formation_error.PZb;
  formation_control_states.err_VXb = formation_error.VXb;
  formation_control_states.err_VYb = formation_error.VYb;
  formation_control_states.err_VZb = formation_error.VZb;
  formation_control_states.led_fol_vxb = formation_error.led_fol_vxb;
  formation_control_states.led_fol_vyb = formation_error.led_fol_vyb;
  formation_control_states.led_fol_vzb = formation_error.led_fol_vzb;

  formation_control_states.err_PXk = formation_error.PXk;
  formation_control_states.err_PYk = formation_error.PYk;
  formation_control_states.err_PZk = formation_error.PZk;
  formation_control_states.err_VXk = formation_error.VXk;
  formation_control_states.err_VYk = formation_error.VYk;
  formation_control_states.err_VZk = formation_error.VZk;
  formation_control_states.led_fol_vxk = formation_error.led_fol_vxk;
  formation_control_states.led_fol_vyk = formation_error.led_fol_vyk;
  formation_control_states.led_fol_vzk = formation_error.led_fol_vzk;

  /* 本部分关于从机的期望值的赋值 */
  formation_control_states.sp_air_speed = formation_sp.air_speed;
  formation_control_states.sp_altitude = formation_sp.altitude;
  formation_control_states.sp_ground_speed = formation_sp.ground_speed;
  formation_control_states.sp_latitude = formation_sp.latitude;
  formation_control_states.sp_longitude = formation_sp.longitude;
  formation_control_states.sp_ned_vel_x = formation_sp.ned_vel_x;
  formation_control_states.sp_ned_vel_y = formation_sp.ned_vel_y;
  formation_control_states.sp_ned_vel_z = formation_sp.ned_vel_z;
  formation_control_states.sp_relative_alt = formation_sp.relative_alt;

  /* 发布编队控制器控制状态 */
  formation_control_states_pub.publish(formation_control_states);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 编队控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */
void TASK_MAIN::control_formation()
{
    fw_col_mode_current = fwstates.control_mode;
    /* 领机状态赋值 */
    leader_states.air_speed = leaderstates.airspeed;

    leader_states.altitude = leaderstates.altitude;
    leader_states.latitude = leaderstates.latitude;
    leader_states.longitude = leaderstates.longitude;
    leader_states.relative_alt = leaderstates.relative_alt;

    leader_states.global_vel_x = leaderstates.global_vel_x;
    leader_states.global_vel_y = leaderstates.global_vel_y;
    leader_states.global_vel_z = leaderstates.global_vel_z;

    leader_states.ned_vel_x = leaderstates.ned_vel_x;
    leader_states.ned_vel_y = leaderstates.ned_vel_y;
    leader_states.ned_vel_z = leaderstates.ned_vel_z;

    leader_states.pitch_angle = leaderstates.pitch_angle;
    leader_states.roll_angle = leaderstates.roll_angle;
    leader_states.yaw_angle = leaderstates.yaw_angle;
    leader_states.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */

    /* 从机状态赋值 */
    thisfw_states.flight_mode = fwstates.control_mode;

    thisfw_states.air_speed = fwstates.air_speed;
    thisfw_states.in_air = fwstates.in_air;

    thisfw_states.altitude = fwstates.altitude;
    thisfw_states.altitude_lock = true; /* 保证TECS */
    thisfw_states.in_air = true;        /* 保证tecs */
    thisfw_states.latitude = fwstates.latitude;
    thisfw_states.longitude = fwstates.longitude;

    thisfw_states.relative_alt = fwstates.relative_alt;

    thisfw_states.ned_vel_x = fwstates.ned_vel_x;
    thisfw_states.ned_vel_y = fwstates.ned_vel_y;
    thisfw_states.ned_vel_z = fwstates.ned_vel_z;

    thisfw_states.global_vel_x = fwstates.global_vel_x;
    thisfw_states.global_vel_y = fwstates.global_vel_y;
    thisfw_states.global_vel_z = fwstates.global_vel_z;

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

    /* 设定编队形状 */
    formation_controller.set_formation_type(2);

    /* 设定前向编队混合参数 */
    formation_controller.set_mix_Xerr_params(mix_Xerr_params);

    /* 设定侧向编队混合参数 */
    formation_controller.set_mix_Yerr_params(mix_Yerr_params);

    /* 模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！ */
    if (fw_col_mode_current != fw_col_mode_last)
    {
       formation_controller.reset_formation_controller();
    }
    /* 更新飞机状态，领机状态 */
    formation_controller.update_led_fol_states(&leader_states, &thisfw_states);
    /* 编队控制 */
    formation_controller.control_formation();

    /* 获得最终控制量 */
    formation_controller.get_formation_4cmd(formation_cmd);
    /* 获得编队控制期望值 */
    formation_controller.get_formation_sp(formation_sp);
    /* 获得编队误差信息 */
    formation_controller.get_formation_error(formation_error);

    /* 控制量赋值 */
    fw_4cmd.throttle_sp = formation_cmd.thrust;
    fw_4cmd.roll_angle_sp = formation_cmd.roll;
    fw_4cmd.pitch_angle_sp = formation_cmd.pitch;
    fw_4cmd.yaw_angle_sp = formation_cmd.yaw;

    fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */
    formation_states_pub();      /* 发布编队控制器状态 */

    fw_col_mode_last = fw_col_mode_current; /* 上一次模式的纪录 */
}

/**
 * @Input: void
 * @Output: void
 * @Description: 外部文件传入的ros_params向编队控制器（外壳，tecs+横侧向控制器）
 */
void TASK_MAIN::input_params()
{
    /* 速度产生的参数调参 */
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/kv_p"      , mix_Xerr_params.kv_p   , 0.2);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/kp_p"      , mix_Xerr_params.kp_p   , 0.5);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/mix_kp"    , mix_Xerr_params.mix_kp , 0.4);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/mix_ki"    , mix_Xerr_params.mix_ki , 0.0);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/mix_kd"    , mix_Xerr_params.mix_kd , 0.0);


    /* 滚转角产生的参数调参 */
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/keta_p" , mix_Yerr_params.keta_p , 0.5);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/kp_p"   , mix_Yerr_params.kp_p   , 0.2);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/mix_kp" , mix_Yerr_params.mix_kp , 0.4);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/mix_ki" , mix_Yerr_params.mix_ki , 0.0);
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/mix_kd" , mix_Yerr_params.mix_kd , 0.0);


    TASK_MAIN_INFO(mix_Xerr_params.kv_p  );
    TASK_MAIN_INFO(mix_Xerr_params.kp_p  );
    TASK_MAIN_INFO(mix_Xerr_params.mix_kp);
    TASK_MAIN_INFO(mix_Xerr_params.mix_ki);
    TASK_MAIN_INFO(mix_Xerr_params.mix_kd);

    TASK_MAIN_INFO(mix_Yerr_params.keta_p);
    TASK_MAIN_INFO(mix_Yerr_params.kp_p  );
    TASK_MAIN_INFO(mix_Yerr_params.mix_kp);
    TASK_MAIN_INFO(mix_Yerr_params.mix_ki);
    TASK_MAIN_INFO(mix_Yerr_params.mix_kd);


}

/**
 * @Input: struct FORMATION_CONTROLLER::_s_fw_states *p
 * @Output: void
 * @Description: TODO:飞机状态打印函数，应该写作模板函数
 */
void TASK_MAIN::print_data(const struct FORMATION_CONTROLLER::_s_fw_states *p)
{
    cout << "***************以下是本飞机状态******************" << endl;
    cout << "***************以下是本飞机状态******************" << endl;

    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;
    cout << "飞机当前姿态欧美系【roll，pitch，yaw】" << rad_2_deg(p->roll_angle) << " [deg] "
         << rad_2_deg(p->pitch_angle) << " [deg] "
         << rad_2_deg(p->yaw_angle) << " [deg] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "飞机当前姿态的旋转矩阵【第2行】" << p->rotmat[2][0] << " [] "
         << p->rotmat[2][1] << " [] "
         << p->rotmat[2][2] << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "body下的加速度【XYZ】" << p->body_acc[0] << " [m/ss] "
         << p->body_acc[1] << " [m/ss] "
         << p->body_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的速度【XYZ】" << p->ned_vel_x << " [m/s] "
         << p->ned_vel_y << " [m/s] "
         << p->ned_vel_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的加速度【XYZ】(由旋转矩阵得来)" << p->ned_acc[0] << " [m/ss] "
         << p->ned_acc[1] << " [m/ss] "
         << p->ned_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "GPS位置【lat,long,alt,rel_alt】" << p->latitude << " [] "
         << p->longitude << " [] "
         << p->altitude << " [] "
         << p->relative_alt << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "风估计【x,y,z】" << p->wind_estimate_x << " [m/s] "
         << p->wind_estimate_y << " [m/s] "
         << p->wind_estimate_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "***************以上是本飞机状态******************" << endl;
    cout << "***************以上是本飞机状态******************" << endl;
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 比赛总任务循环函数
 */
void TASK_MAIN::run()
{
    ros::Rate rate(50.0);
    begin_time = ros::Time::now(); /* 记录启控时间 */
    ros_sub_pub();

    /* input_params(); /1* 读取参数 *1/ */

    while (ros::ok())
    {
        /**
        * 比赛任务大循环，根据来自commander的控制指令来进行响应的控制动作
       */
        current_time = get_ros_time(begin_time); /*此时刻，只作为纪录，不用于控制*/
        TASK_MAIN_INFO("Time:" << current_time);

        if (!fw_cmd_mode.need_take_off &&
            !fw_cmd_mode.need_formation &&
            !fw_cmd_mode.need_land &&
            !fw_cmd_mode.need_idel &&
            fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("保护子程序");
            /**
             * TODO:保护子程序
             */
            fw_current_mode.mode =
                fixed_wing_formation_control::Fw_current_mode::FW_IN_PROTECT;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("空闲子程序");
            /**
                 * TODO:空闲子程序
                */
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_IDEL;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("降落子程序");
            /**
                 * TODO:降落子程序
                */
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_LANDING;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("编队子程序");
            /**
                 * TODO:虽然完成了节点参数的输入函数以及各个通路，但是节点的参数并没有加载进来
                */
            control_formation();

            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_FORMATION;
        }
        else if (fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("起飞子程序");
            /**
                 * TODO:起飞子程序
                */
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_TAKEOFF;
        }
        else
        {
            TASK_MAIN_INFO("错误，飞机当前状态有误");
        }

        /**
         * 发布飞机当前状态
        */
        fw_current_mode_pub.publish(fw_current_mode);

        ros::spinOnce();
        rate.sleep();
    }

    return;
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
