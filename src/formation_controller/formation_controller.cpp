/*本程序的作用是提供几个类型下的编队控制器
例如只有GPS位置，有相对位置以及相对速度，有绝对位置以及绝对速度等
*/

#include "formation_controller.hpp"

float FORMATION_CONTROL::get_sys_time()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    printf("second:%ld \n", tv.tv_sec);                                 //秒
    printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒
    printf("microsecond:%ld \n", tv.tv_sec * 1000000 + tv.tv_usec);     //微秒
    return tv.tv_sec;
}
