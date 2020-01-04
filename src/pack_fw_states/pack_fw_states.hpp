/*
本程序的作用是：
1. 将来自于mavros的消息坐标变换后打包成Fw_state消息，便于以后使用。
2. 将需要发送给飞机的四通道控制量打包，坐标变换！！！
*/

class PACK_FW_STATES
{
    public:
    void send_to_mavros();
}