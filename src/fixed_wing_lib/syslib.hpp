#ifndef _SYSLIB_HPP_
#define _SYSLIB_HPP_
#include <iostream>
#include <fstream>
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

void write_to_files(string file_path_name, float data) //打开一个文件，将它的值以及时间戳写进去，文件命名为值的名字
{
    long time_stamp = get_sys_time(); //时间戳
    fstream oufile;                   //创建文件对象

    oufile.open(file_path_name.c_str(), ios::app | ios::out);
    oufile << fixed << time_stamp << "\t"
           << "\t" << data << endl;

    if (!oufile)
        cout << file_path_name << "-->"
             << "something wrong to open or write" << endl;
    oufile.close();
}

#endif