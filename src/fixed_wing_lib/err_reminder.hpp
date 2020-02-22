/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-22 23:20:10
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  错误信息提示头文件
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-22 23:29:18
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#ifndef _ERR_REMINDER_HPP_
#define _ERR_REMINDER_HPP_

/* 声明出错代码 */
#define ERR_NO_ERROR 0    /* No error */
#define ERR_OPEN_FILE 1   /* Open file error */
#define ERR_SEND_MESG 2   /* sending a message error */
#define ERR_BAD_ARGS 3    /* Bad arguments */
#define ERR_MEM_NONE 4    /* Memeroy is not enough */
#define ERR_SERV_DOWN 5   /* Service down try later */
#define ERR_UNKNOW_INFO 6 /* Unknow information */
#define ERR_SOCKET_ERR 7  /* Socket operation failed */
#define ERR_PERMISSION 8  /* Permission denied */
#define ERR_BAD_FORMAT 9  /* Bad configuration file */
#define ERR_TIME_OUT 10   /* Communication time out */

#include <iostream>

using namespace std;

class ERR_REMINDER
{
public:
    /* 声明出错信息 */
    char *errmsg[] = {
        /* 0 */ "No error",
        /* 1 */ "Open file error",
        /* 2 */ "Failed in sending/receiving a message",
        /* 3 */ "Bad arguments",
        /* 4 */ "Memeroy is not enough",
        /* 5 */ "Service is down; try later",
        /* 6 */ "Unknow information",
        /* 7 */ "A socket operation has failed",
        /* 8 */ "Permission denied",
        /* 9 */ "Bad configuration file format",
        /* 10 */ "Communication time out",
    };
};

#endif