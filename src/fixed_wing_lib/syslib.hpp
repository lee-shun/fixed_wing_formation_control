#ifndef _SYSLIB_HPP_
#define _SYSLIB_HPP_
#include <iostream>
#include <sys/time.h>

using namespace std;

long get_sys_time()//返回的时间为毫秒，使用需要除以1000，为秒
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
    //printf("second:%ld \n", tv.tv_sec);                                 //秒
    //printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒
    //printf("microsecond:%ld \n", tv.tv_sec * 1000000 + tv.tv_usec);     //微秒
}

#endif