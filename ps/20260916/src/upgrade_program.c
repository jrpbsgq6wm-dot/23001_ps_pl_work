/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : upgrade_program.c
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-10-10 13:33:26
 ******************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include "main.h"
#include "tcptrans_link.h"
#include "drytoupper.h"
#include "upgrade_program.h"

int send_flag = 0;


/*****************************************************************************
 * * description : 在线升级DRY程序
 * * return       {*}
 * * Date        : 2025-10-10 13:30:58
 * * Other
 ******************************************************************************/
void upgrade_program_dry(void)
{
    char   Update_DRY_Head[4] = { '#','#','S','U' };
    send_flag = 0;
    if(system("./run/updata_sys.sh") == 0){
        send_flag = 1;
    }else{
        send_flag = 0;
    }
    send(Connect_fd_upper, Update_DRY_Head, 4, 0);
    send(Connect_fd_upper, &((unsigned int){htonl((unsigned int)send_flag)}), 4, 0);
    printf("send_flag=%d\n", send_flag);
}
