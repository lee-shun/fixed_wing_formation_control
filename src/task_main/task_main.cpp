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

void TASK_MAIN::run()
{
    ros::Rate rate(50.0);
    begin_time = ros::Time::now(); // 记录启控时间

    fw_states_sub = nh.subscribe<fixed_wing_formation_control::FWstates> //1.【订阅】固定翼全部状态量
                    ("fixed_wing_formation_control/fw_states", 10, &TASK_MAIN::fw_state_cb, this);
    //2. 【订阅】领机信息
    fw_cmd_pub = nh.advertise<fixed_wing_formation_control::FWcmd> //3.【发布】固定翼四通道控制量
                 ("/fixed_wing_formation_control/fw_cmd", 10);

    while (ros::ok())
    {
        current_time = get_ros_time(begin_time);

        cout << "飞机的空速：：" << fwstates.air_speed << endl;

        fw_4cmd.throttle_sp = 0.5;
        fw_cmd_pub.publish(fw_4cmd);
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