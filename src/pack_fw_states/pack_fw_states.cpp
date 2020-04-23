/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:
 *  * 本程序的作用是：
 *  1. 将来自于mavros的消息坐标变换后打包成Fw_state消息，便于以后使用。
 *  2. 将需要发送给飞机的四通道控制量消息坐标变换解包，发给mavros   
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-20 20:24:43
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "pack_fw_states.hpp"

void PACK_FW_STATES::set_planeID(int id) {
  planeID = id;
  switch (id) {
  case 0:
    uavID = "uav0/"; /* 领机 */
    break;
  case 1:
    uavID = "uav1/"; /*从机*/
    break;
  case 2:
    uavID = "uav2/"; /*从机*/
    break;
  case 3:
    uavID = "uav3/"; /*从机*/
    break;
  }
}
void PACK_FW_STATES::ros_sub_and_pub() {
  //##########################################订阅mavros消息+控制器指令cmd###################################################//
  fixed_wing_states_sub                  //
      = nh.subscribe<mavros_msgs::State> //
        (add2str(uavID, "mavros/state"), 10, &_FIXED_WING_SUB_PUB::state_cb,
         &fixed_wing_sub_pub);

  // 【订阅】无人机imu信息，
  fixed_wing_imu_sub                   //
      = nh.subscribe<sensor_msgs::Imu> //
        (add2str(uavID, "mavros/imu/data"), 10, &_FIXED_WING_SUB_PUB::imu_cb,
         &fixed_wing_sub_pub);

  // 【订阅】无人机gps位置
  fixed_wing_global_position_form_px4_sub    //
      = nh.subscribe<sensor_msgs::NavSatFix> //
        (add2str(uavID, "mavros/global_position/global"), 10,
         &_FIXED_WING_SUB_PUB::global_position_form_px4_cb,
         &fixed_wing_sub_pub);

  //【订阅】无人机gps相对alt
  fixed_wing_global_rel_alt_from_px4_sub //
      = nh.subscribe<std_msgs::Float64>  //
        (add2str(uavID, "mavros/global_position/rel_alt"), 10,
         &_FIXED_WING_SUB_PUB::fixed_wing_global_rel_alt_from_px4_cb,
         &fixed_wing_sub_pub);

  // 【订阅】无人机ump位置
  fixed_wing_umt_position_from_px4_sub   //
      = nh.subscribe<nav_msgs::Odometry> //
        (add2str(uavID, "mavros/global_position/local"), 10,
         &_FIXED_WING_SUB_PUB::umt_position_from_px4_cb, &fixed_wing_sub_pub);

  // 【订阅】无人机gps三向速度
  fixed_wing_velocity_global_fused_from_px4_sub   //
      = nh.subscribe<geometry_msgs::TwistStamped> //
        (add2str(uavID, "mavros/global_position/raw/gps_vel"), 10,
         &_FIXED_WING_SUB_PUB::velocity_global_fused_from_px4_cb,
         &fixed_wing_sub_pub);

  // 【订阅】无人机ned位置
  fixed_wing_local_position_from_px4             //
      = nh.subscribe<geometry_msgs::PoseStamped> //
        (add2str(uavID, "mavros/local_position/pose"), 10,
         &_FIXED_WING_SUB_PUB::local_position_from_px4_cb, &fixed_wing_sub_pub);

  // 【订阅】无人机ned三向速度
  fixed_wing_velocity_ned_fused_from_px4_sub      //
      = nh.subscribe<geometry_msgs::TwistStamped> //
        (add2str(uavID, "mavros/local_position/velocity_local"), 10,
         &_FIXED_WING_SUB_PUB::velocity_ned_fused_from_px4_cb,
         &fixed_wing_sub_pub);

  // 【订阅】无人机ned三向加速度
  fixed_wing_acc_ned_from_px4_sub                               //
      = nh.subscribe<geometry_msgs::AccelWithCovarianceStamped> //
        (add2str(uavID, "mavros/local_position/accel"), 10,
         &_FIXED_WING_SUB_PUB::acc_ned_from_px4_cb, &fixed_wing_sub_pub);

  // 【订阅】无人机ned三向加速度
  fixed_wing_wind_estimate_from_px4_sub                         //
      = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped> //
        (add2str(uavID, "mavros/wind_estimation"), 10,
         &_FIXED_WING_SUB_PUB::wind_estimate_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_battrey_state_from_px4_sub         //
      = nh.subscribe<sensor_msgs::BatteryState> //
        (add2str(uavID, "mavros/battery"), 10,
         &_FIXED_WING_SUB_PUB::battrey_state_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_waypoints_sub //// 【订阅】无人机当前航点
      = nh.subscribe<mavros_msgs::WaypointList> //
        (add2str(uavID, "mavros/mission/waypoints"), 10,
         &_FIXED_WING_SUB_PUB::waypointlist_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_waypointsreach_sub //// 【订阅】无人机到达的航点
      = nh.subscribe<mavros_msgs::WaypointReached> //
        (add2str(uavID, "mavros/mission/reached"), 10,
         &_FIXED_WING_SUB_PUB::waypoints_reached_from_px4_cb,
         &fixed_wing_sub_pub);

  fixed_wing_altitude_from_px4_sub          //订阅高度
      = nh.subscribe<mavros_msgs::Altitude> //
        (add2str(uavID, "mavros/altitude"), 10,
         &_FIXED_WING_SUB_PUB::altitude_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_air_ground_speed_from_px4_sub //订阅空速、地速
      = nh.subscribe<mavros_msgs::VFR_HUD> //
        (add2str(uavID, "mavros/vfr_hud"), 10,
         &_FIXED_WING_SUB_PUB::air_ground_speed_from_px4_cb,
         &fixed_wing_sub_pub);

  fixed_wing_cmd_from_controller_sub //订阅来自上层控制器的四通道控制量
      = nh.subscribe<fixed_wing_formation_control::FWcmd> //
        (add2str(uavID,"fixed_wing_formation_control/fw_cmd"), 10,
         &_FIXED_WING_SUB_PUB::cmd_from_controller_cb, &fixed_wing_sub_pub);

  //##########################################订阅mavros消息+控制器指令cmd###################################################//
  //
  //
  //
  //
  //##########################################发布mavros消息+飞机状态states###################################################//

  fixed_wing_local_pos_sp_pub = nh.advertise<mavros_msgs::PositionTarget>(
      add2str(uavID, "mavros/setpoint_raw/local"), 10);

  fixed_wing_global_pos_sp_pub =
      nh.advertise<mavros_msgs::GlobalPositionTarget>(
          add2str(uavID, "mavros/setpoint_raw/global"), 10);

  fixed_wing_local_att_sp_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      add2str(uavID, "mavros/setpoint_raw/attitude"), 10);

  fixed_wing_states_pub = nh.advertise<fixed_wing_formation_control::FWstates>(
          add2str(uavID,"fixed_wing_formation_control/fw_states"), 10);

  //##########################################发布消息###################################################//
  //
  //
  //
  //
  //##########################################服务###################################################//
  // 服务 修改系统模式
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
      add2str(uavID, "mavros/set_mode"));

  arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
      add2str(uavID, "mavros/cmd/arming"));

  waypoint_setcurrent_client =
      nh.serviceClient<mavros_msgs::WaypointSetCurrent>(
          add2str(uavID, "mavros/mission/set_current"));

  waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>(
      add2str(uavID, "mavros/mission/pull"));

  waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>(
      add2str(uavID, "mavros/mission/push"));

  waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>(
      add2str(uavID, "mavros/mission/clear"));
  //##########################################服务###################################################//
  //
}

void PACK_FW_STATES::pack_fw_states() {
  //给结构体赋值；更新飞机状态
  //控制模式
  fixed_wing_sub_pub.fw_states_form_mavros.control_mode =
      fixed_wing_sub_pub.current_state.mode;

  //以下为GPS信息
  fixed_wing_sub_pub.fw_states_form_mavros.altitude =
      fixed_wing_sub_pub.global_position_form_px4.altitude;
  fixed_wing_sub_pub.fw_states_form_mavros.latitude =
      fixed_wing_sub_pub.global_position_form_px4.latitude;
  fixed_wing_sub_pub.fw_states_form_mavros.longitude =
      fixed_wing_sub_pub.global_position_form_px4.longitude;

  // GPS速度是在ned下的，
  fixed_wing_sub_pub.fw_states_form_mavros.global_vel_x =
      fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.y;
  fixed_wing_sub_pub.fw_states_form_mavros.global_vel_y =
      fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.x;
  fixed_wing_sub_pub.fw_states_form_mavros.global_vel_z =
      -fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.z;

  fixed_wing_sub_pub.fw_states_form_mavros.relative_alt =
      fixed_wing_sub_pub.global_rel_alt_from_px4.data;

  //以下为机体系和地面系的夹角，姿态角
  fixed_wing_sub_pub.fw_states_form_mavros.roll_angle =
      fixed_wing_sub_pub.att_angle_Euler[0];
  fixed_wing_sub_pub.fw_states_form_mavros.pitch_angle =
      -fixed_wing_sub_pub.att_angle_Euler[1]; //添加负号转换到px4的系

  if (-fixed_wing_sub_pub.att_angle_Euler[2] + deg_2_rad(90.0) > 0)
    fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle =
        -fixed_wing_sub_pub.att_angle_Euler[2] +
        deg_2_rad(90.0); //添加符号使增加方向相同，而且领先于px490°
  else
    fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle =
        -fixed_wing_sub_pub.att_angle_Euler[2] + deg_2_rad(90.0) +
        deg_2_rad(360.0);

  //姿态四元数赋值
  att_angle[0] = fixed_wing_sub_pub.fw_states_form_mavros.roll_angle;
  att_angle[1] = fixed_wing_sub_pub.fw_states_form_mavros.pitch_angle;
  att_angle[2] = fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle;
  euler_2_quaternion(att_angle, att_quat);
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.w = att_quat[0];
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.x = att_quat[1];
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.y = att_quat[2];
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.z = att_quat[3];

  //以下为ned坐标系下的位置，速度
  fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_x =
      fixed_wing_sub_pub.local_position_from_px4.pose.position.y;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_y =
      fixed_wing_sub_pub.local_position_from_px4.pose.position.x;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_z =
      -fixed_wing_sub_pub.local_position_from_px4.pose.position.z;

  fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_x =
      fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.y;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_y =
      fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.x;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_z =
      -fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.z;

  //以下为体轴系加速度，体轴系当中的加速度是符合px4机体系的定义的
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc_x =
      fixed_wing_sub_pub.imu.linear_acceleration.x;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc_y =
      fixed_wing_sub_pub.imu.linear_acceleration.y;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc_z =
      fixed_wing_sub_pub.imu.linear_acceleration.z;

  fixed_wing_sub_pub.fw_states_form_mavros.body_acc.x =
      fixed_wing_sub_pub.imu.linear_acceleration.x;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc.y =
      fixed_wing_sub_pub.imu.linear_acceleration.y;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc.z =
      fixed_wing_sub_pub.imu.linear_acceleration.z;

  //以下来自altitude
  fixed_wing_sub_pub.fw_states_form_mavros.relative_hight =
      fixed_wing_sub_pub.altitude_from_px4.relative;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_altitude =
      fixed_wing_sub_pub.altitude_from_px4.local;

  //空速和地速
  fixed_wing_sub_pub.fw_states_form_mavros.air_speed =
      fixed_wing_sub_pub.air_ground_speed_from_px4.airspeed;
  fixed_wing_sub_pub.fw_states_form_mavros.ground_speed =
      fixed_wing_sub_pub.air_ground_speed_from_px4.groundspeed;

  //风估计
  fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_x =
      fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.y;
  fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_y =
      fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.x;
  fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_z =
      -fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.z;
  //电池状态
  fixed_wing_sub_pub.fw_states_form_mavros.battery_current =
      fixed_wing_sub_pub.battrey_state_from_px4.current;
  fixed_wing_sub_pub.fw_states_form_mavros.battery_precentage =
      fixed_wing_sub_pub.battrey_state_from_px4.percentage;
  fixed_wing_sub_pub.fw_states_form_mavros.battery_voltage =
      fixed_wing_sub_pub.battrey_state_from_px4.voltage;

  //打包的mavros发布

  fixed_wing_states_pub.publish(fixed_wing_sub_pub.fw_states_form_mavros);
}

void PACK_FW_STATES::msg_to_mavros() {
  //将期望值转换一下坐标系，并转化为四元数
  float angle[3], quat[4];

  angle[0] = fixed_wing_sub_pub.cmd_from_controller.roll_angle_sp;
  angle[1] = -fixed_wing_sub_pub.cmd_from_controller.pitch_angle_sp; //
  angle[2] =
      -fixed_wing_sub_pub.cmd_from_controller.yaw_angle_sp + deg_2_rad(90.0);

  euler_2_quaternion(angle, quat);

  fixed_wing_sub_pub.att_sp.type_mask =
      7; // 1+2+4+64+128 body.rate_x,body.rate_y,body.rate_z thrust..
  fixed_wing_sub_pub.att_sp.orientation.w = quat[0];
  fixed_wing_sub_pub.att_sp.orientation.x = quat[1];
  fixed_wing_sub_pub.att_sp.orientation.y = quat[2];
  fixed_wing_sub_pub.att_sp.orientation.z = quat[3];
  fixed_wing_sub_pub.att_sp.thrust =
      fixed_wing_sub_pub.cmd_from_controller.throttle_sp;

  fixed_wing_local_att_sp_pub.publish(fixed_wing_sub_pub.att_sp);
}

void PACK_FW_STATES::srv_to_mavros() {
  //设置模式
  fixed_wing_sub_pub.mode_cmd.request.custom_mode =
      fixed_wing_sub_pub.cmd_from_controller.cmd_mode;
  set_mode_client.call(fixed_wing_sub_pub.mode_cmd);
}

void PACK_FW_STATES::run(int argc, char **argv) {
  ros::Rate rate(50.0);

  ros_sub_and_pub();
  PACK_FW_STATES_INFO("打包开始");

  long begin_time = get_sys_time();

  while (ros::ok()) {

    print_counter++;

    if (print_counter >= 100) {
      PACK_FW_STATES_INFO("本机ID:" << planeID);
      PACK_FW_STATES_INFO(
          "历时:" << fixed << setprecision(5)
                  << double(get_time_from_begin(begin_time)) / 1000 << " 秒");
      print_counter = 0;
    }

    //一直在向mavros发，保持px4的链接不断
    pack_fw_states();
    msg_to_mavros();
    srv_to_mavros();

    ros::spinOnce(); //挂起一段时间，保证周期的速度
    rate.sleep();
  }
  PACK_FW_STATES_INFO("打包退出");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pack_fw_states");

  PACK_FW_STATES _pack;
  if (true) {
    _pack.set_planeID(1);
    _pack.run(argc, argv);
  }

  return 0;
}
