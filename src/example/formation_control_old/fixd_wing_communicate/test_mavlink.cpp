#include "test_mavlink.hpp"
void FIXED_WING_MAVLINK::handle_message(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {

    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_message_heartbeat(msg);
        break;

        
        
        
        // case MAVLINK_MSG_ID_BATTERY_STATUS:
        //     handle_message_battery_status(msg);
        //     break;

    default:
        break;
    }
}

void FIXED_WING_MAVLINK::handle_message_heartbeat(mavlink_message_t *msg)
{
    /* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */

    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);

    /* Accept only heartbeats from GCS or ONBOARD Controller, skip heartbeats from other vehicles */
    //if ((msg->sysid != mavlink_system.sysid && hb.type == MAV_TYPE_GCS) || (msg->sysid == mavlink_system.sysid && hb.type == MAV_TYPE_ONBOARD_CONTROLLER))
    //{
    cout << "in the handle heartbeat" << endl;
    //}
}
int FIXED_WING_MAVLINK::open_the_seial()
{
    //串口serial设置&开启

    serial::Timeout to = serial::Timeout::simpleTimeout(10); //创建timeout
    sp.setPort("/dev/ttyACM0");                              //设置要打开的串口名称
    sp.setBaudrate(115200);                                  //设置串口通信的波特率
    sp.setTimeout(to);                                       //串口设置timeout
    //打开串口
    try
    {
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return 0;
    }
    //判断串口是否打开成功
    if (sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyACM0 is opened.");
        return 1;
    }
    else
    {
        return 0;
    }
}
void FIXED_WING_MAVLINK::run()
{

    ros::Rate rate(1);

    int a = open_the_seial();

    while (ros::ok() && a)
    {
        size_t n = sp.available();
        if (n != 0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            mavlink_parse_char(MAVLINK_COMM_0, n, &msg, &_status);
        }
        // for (int i = 0; i < nread; i++)
        // {

        //     cout << hex << (buf[i] & 0xff) << "  ";

        //     if (mavlink_parse_char(_channel, buf[i], &msg, &_status))
        //     {
        //         handle_message(&msg);
        //     }
        // }

        ros::spinOnce();
        //挂起一段时间(rate为 50HZ
        rate.sleep();
    }

    sp.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fixed_wing_mavlink");

    FIXED_WING_MAVLINK _fixed_wing_mavlink;

    _fixed_wing_mavlink.run();

    return 0;
}