/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : tcptrans_link.h
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-12-05 14:36:56
 ******************************************************************************/
#ifndef __TCPTRANS_LINK_H
#define __TCPTRANS_LINK_H

//#define DEFAULT_PORT_DRY 8000
//#define DEFAULT_PORT_WET 7000
#define DEFAULT_PORT_DRY 7000
#define DEFAULT_PORT_WET 8000
//#define INADDR_WET "192.168.0.5"
//#define INADDR_UPPER "192.168.0.6"

#include <stdio.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
extern int pc_link_flag;
extern struct sockaddr_in Servaddr_wet,Servaddr_upper;
extern int Socket_fd_upper, Connect_fd_upper, Socket_fd_wet;
extern void network_monitor(void);
extern void get_pc_ip(int sock_pc);
extern void test(void);
extern void init_tcptrans_socket(void);
extern void error_process(void);
extern unsigned int recv_socket(int fd, char* buf, unsigned int len);


#endif
