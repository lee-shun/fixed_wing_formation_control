#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <mavlink/v1.0/mavlink_types.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <mavlink/v1.0/mavlink_helpers.h>
#include <mavlink/v1.0/common/mavlink_msg_attitude.h>
#include <mavlink/v1.0/common/mavlink_msg_heartbeat.h>
#include <mavlink/v1.0/common/mavlink_msg_battery_status.h>

using namespace std;

class FIXED_WING_MAVLINK
{
private:
    mavlink_message_t msg;

    uint8_t buf[64];

    mavlink_status_t _status{}; ///< receiver status, used for mavlink_parse_char()

    ros::NodeHandle nh;

    mavlink_channel_t _channel{MAVLINK_COMM_0};

    serial::Serial sp; //创建一个serial类

    int nread;

public:
    int open_the_seial();

    void run();

    void handle_message(mavlink_message_t *msg);
    void handle_message_heartbeat(mavlink_message_t *msg);
};