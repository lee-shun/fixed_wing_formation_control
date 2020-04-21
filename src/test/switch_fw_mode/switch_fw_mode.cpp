/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 * 本程序的作用是将飞控的模式切换为需要的模式 
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-23 14:11:14
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "switch_fw_mode.hpp"

/**
 * @Input: int
 * @Output:
 * @Description: 设定当前飞机的ID
 */
void SWITCH_FW_MODE::set_planeID(int id) {
  planeID = id;
  switch (planeID) {
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

void SWITCH_FW_MODE::run(int argc, char **argv) {

  ros::init(argc, argv, "switch_fw_mode");

  ros::Rate rate(5.0); /*  频率 [5Hz] */

  /* 【订阅】无人机当前状态 */
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      add2str(uavID, "mavros/state"), 10, &SWITCH_FW_MODE::state_cb, this);

  ros::Publisher task_cmd_mode_pub =
      nh.advertise<fixed_wing_formation_control::Fw_cmd_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_cmd_mode"), 10);

  /* 【服务】 修改系统模式 */
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>(add2str(uavID, "mavros/set_mode"));

  while (ros::ok() && (!outflag)) {
    float current_time = get_ros_time(begin_time); /* 当前时间 */

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>固定翼模式<<<<<<<<<<<<<<<<<<<<<<<<<<"
            "<<<"
         << endl;
    cout << "时刻: " << current_time << " [s] " << endl;
    cout << "模式 : [ " << current_state.mode << " ]" << endl;
    cout << "更换模式：1 offboard, 2 mission, 3 out" << endl;

    cin >> mode_type;
    if (mode_type == 1) {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_formation = true;
      task_cmd_mode_pub.publish(task_cmd_mode);
    } else if (mode_type == 2) {
      mode_cmd.request.custom_mode = "AUTO.MISSION";
    } else if (mode_type == 3) {
      outflag = true;
      break;
    }
    /*  如果当前模式与设定模式不一致，切换，（有次数限制） */
    while ((mode_cmd.request.custom_mode != current_state.mode) &&
           (counters <= times_out)) {

      counters++;
      set_mode_client.call(mode_cmd);

    }

    ros::spinOnce(); /*  挂起一段时间，等待切换结果 */
    rate.sleep();

    counters = 0;
  }
}

float SWITCH_FW_MODE::get_ros_time(ros::Time begin) {
  ros::Time time_now = ros::Time::now();
  float currTimeSec = time_now.sec - begin.sec;
  float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimeSec + currTimenSec);
}
