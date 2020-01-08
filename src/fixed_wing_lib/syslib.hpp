#ifndef _SYSLIB_HPP_
#define _SYSLIB_HPP_
#include <iostream>
#include <sys/time.h>

using namespace std;

long get_sys_time() //返回的时间为毫秒，使用需要除以1000，为秒
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

long get_time_from_begin(long begin_time) //进入、返回的时间为毫秒，使用需要除以1000，为秒
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000 + tv.tv_usec / 1000 - begin_time);
}

#endif