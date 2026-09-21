/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : drytoupper.c
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-11-17 11:08:24
 ******************************************************************************/
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <poll.h>
#include <time.h>
#include "main.h"
#include "tcptrans_link.h"
#include "drytoupper.h"
#include "uppertodry.h"
#include "armtofpga.h"
#include "armtodsp.h"
#include "fpga_init.h"
#include "tempsensor.h"
#include "wettodry.h"



struct timeval tv0;
struct timeval tv1 = {0};


int IQDataTotalSize = 0, IQDataSizeFromWetSend = 0,IQDataTotalSize_Last = 0, total_len_iqallsize = 0;

/*
 * 旧版本辅助发送函数仍保留在本文件中供历史代码兼容，但不再参与S231正式
 * NREAD上传。其缓存被限制为4MiB，避免再为可能达到196MiB的Frame申请30MiB
 * 的整帧缓存；正式S231路径直接从NREAD DDR分块发送，不经过该缓存。
 */
char Image_Data[S231_NREAD_RESULT_MAX_BYTES] = {0};
float Toa_Time_s[512] = {0};
SEND_UPPER_PACKAGE_FIRST bd_send_to_upper_package_first = {0};
SEND_UPPER_PACKAGE_FIRST *ptr_bd_send_to_upper_package_first = &bd_send_to_upper_package_first;

#if 0
char Image_Data[25000000];//接收FPGA上传的图像数据
char IQData[25000000];//接收湿端上传的IQ数据或者原始数据数组,在处理上1ping的数据,湿端压了1ping
char IQData_to_upper[25000000];//接收湿端上传的IQ数据或者原始数据数组,在处理上1ping的数据,湿端压了1ping
char IQData_Last[25000000];//波速下放时为了进行波束形成,又压了1ping
#endif
char IQData[30000000] = {0};//接收湿端上传的IQ数据或者原始数据数组,在处理上1ping的数据,湿端压了1ping
char IQData_to_upper[30000000] = {0};//接收湿端上传的IQ数据或者原始数据数组,在处理上1ping的数据,湿端压了1ping
char IQData_Last[30000000] = {0};//波速下放时为了进行波束形成,又压了1ping
//波束未下放的版本:参数上传到显控通过IQdata中的参数头文件
SEND_UPPER_SONAR_STATUS  send_upper_status_information = {0};//发送给上位机状态信息----结构体变量
SEND_UPPER_SONAR_STATUS* ptr_send_upper_status_information = &send_upper_status_information; //发送给上位机状态信息----结构体指针

SEND_UPPER_SENSOR_FIRST  send_to_upper_sensor = {0}, Recv_Wet_MemsSensorHead = {0};//ARM发送给上位机的传感器数据包头
SEND_UPPER_SENSOR_FIRST* ptr_send_to_upper_sensor = &send_to_upper_sensor;//ARM发送给上位机的传感器数据包头
SEND_UPPER_SENSOR_FIRST* ptr_Recv_Wet_MemsSensorHead = &Recv_Wet_MemsSensorHead;



	int count = 0;
	int Last_count = 0;
/*****************************************************************************
 * * description : 波束未下放的数据上传方式
 * * return       {*}
 * * Date        : 2022-10-12 10:35:21
 * * Other       : 与下放的不兼容
 ******************************************************************************/
void Send_IQdataToUpper(void)
{
    //数据包头
    char SendUpperHead[4] = { '<','<','S','T' };
    //数据包尾
    char SendUpperTail[6] = { '0','0','E','D','>','>' };
    pthread_mutex_lock(&mut);
    IQDataTotalSize = IQDataSizeFromWetSend;
    
    memcpy(IQData + IQDataTotalSize - 6, SendUpperTail, 6);
	memcpy(IQData_to_upper,SendUpperHead,4);
	memcpy(IQData_to_upper+4,&IQDataTotalSize,4);
	my_copy(IQData_to_upper+4+4,IQData,IQDataTotalSize);
    if (send(Connect_fd_upper, IQData_to_upper, IQDataTotalSize+8, 0) < 0)//all iqdata 此时的IQdata含参数头
    {
        pthread_mutex_unlock(&mut);
        error_process();
    }
    pthread_mutex_unlock(&mut);
}


/*****************************************************************************
 * * description : 向显控回复的硬件状态信息
 * * return       {*}
 * * Date        : 2022-10-10 17:42:14
 * * Other       : 只是干端\湿端硬件版本信息,其他的目前没有用到
 ******************************************************************************/
void send_status_to_upper(void)
{
    char   Send_Upper_DataHead[4] = { '@','@','S','S' };// 数据头标示“@@SS”,表示显控数据头
    char   DataTail_S[4] = { 'E','D','@','@' };//ARM与上位机通信的报尾“ED@@”
    memcpy(ptr_send_upper_status_information->head,Send_Upper_DataHead,4);//ARM与上位机通信的报头“@@SS”
    memcpy(ptr_send_upper_status_information->tail,DataTail_S,4);//ARM与上位机通信的报尾“ED@@”
    char* ptr_to_send;

    ptr_send_upper_status_information->pack_len = 136;
    ptr_send_upper_status_information->DRY_FPGAVersion = ptr_fpga_register_data->fpga_sta.date;
    ptr_send_upper_status_information->DRY_LinuxDriverVersion = LINUX_VERSION;
    ptr_send_upper_status_information->WET_EPLD_VERSION = 0;//暂无
  //  ptr_send_upper_status_information->DRY_DSP_VERSION = DSP_VERSION;
    ptr_send_upper_status_information->device_type = 1004;
    ptr_send_upper_status_information->sync_mode = ptr_wet_sonar->sync_state;
    ptr_send_upper_status_information->sync_type = ptr_wet_sonar->sync_selec;
    ptr_send_upper_status_information->sync_delay = ptr_wet_sonar->sync_delaytime;
    ptr_send_upper_status_information->wet_status = wet_status;
    if (ptr_send_upper_status_information->DRY_FPGAVersion > 2022000000 && ptr_send_upper_status_information->DRY_FPGAVersion < 2030000000)
        dry_fpga_status =1;
    ptr_send_upper_status_information->dry_fpga_status = dry_fpga_status;
    ptr_send_upper_status_information->dsp_status = dsp_status;
    ptr_send_upper_status_information->GGAZDA_status = GGAZDA_status;
    ptr_send_upper_status_information->Heading_status = Heading_status;
    ptr_send_upper_status_information->TSS1_status = TSS1_status;
    ptr_send_upper_status_information->SV_status = SV_status;
    ptr_send_upper_status_information->PPS_status = PPS_status;
    //发送硬件信息给显控软件
    ptr_to_send = (char*)ptr_send_upper_status_information;
    if (send(Connect_fd_upper, ptr_to_send, SIZE_OF_STATUS_SEND_UPPER, 0) < 0) //136byte
    {
        error_process();
        return;
    }
}

void Debug_pritf_sendtoupper_hardwarepara(SEND_UPPER_SONAR_STATUS* ptr_send_upper_status_information)
{
    Debug("+++++++++++++++++++++++arm to upper status(@@SS)++++++++++++++++++++++++++++++++\n");
    Debug("ptr_send_upper_status_information->DRY_FPGAVersion: %d\n",  ptr_send_upper_status_information->DRY_FPGAVersion);
    Debug("ptr_send_upper_status_information->DRY_LinuxDriverVersion: %d\n", ptr_send_upper_status_information->DRY_LinuxDriverVersion);
    Debug("ptr_send_upper_status_information->WET_LinuxDriverVersion: %d\n", ptr_send_upper_status_information->WET_LinuxDriverVersion);
    Debug("ptr_send_upper_status_information->WET_FPGAVersion: %d\n", ptr_send_upper_status_information->WET_FPGAVersion);
    Debug("ptr_send_upper_status_information->WET_EPLDVersion: %d\n", ptr_send_upper_status_information->WET_EPLD_VERSION);
    Debug("++++++++++++++++++++++arm to upper status(@@SS) end +++++++++++++++++++++++++++\n");
}


/************************************************波束下放版本新增************************************************************************/



/**
 * 确保发送完整的数据缓冲区
 * @param fd      socket 文件描述符
 * @param buf     要发送的数据缓冲区
 * @param len     要发送的数据长度
 * @param flags   通常为 0，与 send() 的 flags 参数相同
 * @return        成功返回实际发送的字节数（应等于 len），失败返回 -1
 */
ssize_t send_all(int fd, const void *buf, size_t len, int flags,int* sendcount) {
    const char *ptr = (const char *)buf;
    size_t remaining = len;
    ssize_t sent;
    while (remaining > 0) {
        sent = send(fd, ptr, remaining, flags);
        if (sent < 0) {
            // 被信号中断，继续发送
            if (errno == EINTR) {
                continue;
            }
            // 其他错误返回 -1
            return -1;
        }
        // 对端关闭连接，返回已发送字节数（可能少于 len）
        if (sent == 0) {
            break;
        }
        ptr += sent;
        remaining -= sent;
        if (sendcount) {
            (*sendcount)++;
        }
    }
    // 返回总共发送的字节数
    return len - remaining;
}


float Toa_Time_s[512] = {0};

/*****************************************************************************
 * * description : 
 * * return       {*}
 * * Date        : 2022-10-10 17:41:41
 * * Other
 ******************************************************************************/
void send_all_package_to_upper(void)
{
    int lenth_upper;
	char iq_head[4] = {'<','<','S','T'};
    memset(Toa_Time_s, 0L, sizeof(Toa_Time_s));
    memcpy(Toa_Time_s, (unsigned char*)((Image_Data) + 87 + 2208), 4 * 512);

    size_t total_len = (size_t)bd_send_to_upper_package_first.All_lenth + 8;
    int sendcount = 0;
    if (total_len > sizeof(Image_Data)) {
        DBG("legacy upload length %zu exceeds 4MiB compatibility buffer, drop it\n", total_len);
        return;
    }
    lenth_upper = send_all(Connect_fd_upper, (char*)Image_Data, total_len, 0, &sendcount);
    if (lenth_upper < 0) {
        LOG("send_all 失败: %s", strerror(errno));
        error_process();
        return;
    }
    if (lenth_upper != (ssize_t)total_len) {
        LOG("发送不完整: 期望 %zu, 实际 %zd", total_len, lenth_upper);
        // 根据业务决定是否重试或报错
        error_process();
        return;
    }
    // LOG("发送成功: %zu 字节, 共 %d 次 send 调用", total_len, sendcount);
    //  lenth_upper = send(Connect_fd_upper, (char*)(Image_Data), (bd_send_to_upper_package_first.All_lenth + 8), 0);//@@st + all_len
    //  if (lenth_upper < 0)
    //  {
    //     error_process();
    //     return;
    // }
	memset(Image_Data,0,sizeof(Image_Data));
}

void send_all_package_to_upper_lock(void)
{
    if (Fpga_start_mod == 1) //开始
    {
        pthread_mutex_lock(&mut);
        send_all_package_to_upper();
        pthread_mutex_unlock(&mut);
    }
}

void dry_to_upper(void)
{
    unsigned icount;
    int err;
    char Head[4];
    int num[5];
    int send_upper_buf_flag = 0;
    float Send_PingNum = 0;
    int lenth_upper = 0;

    /* 新设备树中，CBF节点同时提供寄存器映射和FPGA完成中断。 */
    int irq_fd = uio_cfb.fd;

    /* 保留原业务逻辑：通知FPGA把结果写入指定的DSP回读DDR地址。 */
    ptr_fpga_register_addr->recive_Base_Addr_0 = 0x3F400000;

    while (1)
    {
        /* 等待FPGA中断；收到中断后仍从原来的指定DDR地址取数据。 */
        err = read(irq_fd, &icount, 4);
        /* UIO读中断后写1重新使能中断，保持原来的中断处理时序。 */
        write(irq_fd, &irq_on, sizeof(irq_on));
        if (err != 4)
        {
            perror("uio read err\n");
        }
        else
        {
            /* 第一块DSP回读DDR的旧协议数据处理逻辑保持不变。 */
            if ((send_upper_buf_flag & (0x00000001)) == 0)
            {
                memcpy(&bd_send_to_upper_package_first, (void*)uio_nread.mem_ptr, SIZE_OF_SEND_UPPER_PACKAGE_FIRST);
                ptr_bd_send_to_upper_package_first->dry_tem = dry_temp;
                if ((bd_send_to_upper_package_first.head[0] == '@') && (bd_send_to_upper_package_first.head[1] == '@') && (bd_send_to_upper_package_first.head[2] == 'S') && (bd_send_to_upper_package_first.head[3] == 'T'))
                {
                    if (bd_send_to_upper_package_first.All_lenth > 5000000)
                    {
                        DBG("bd_send_to_upper_package_first.All_lenth = %d,to big error!!!\n",bd_send_to_upper_package_first.All_lenth);
                        continue;
                    }
                    memcpy((void*)uio_nread.mem_ptr, &bd_send_to_upper_package_first, SIZE_OF_SEND_UPPER_PACKAGE_FIRST);
                    my_copy(Image_Data , (unsigned char*)(uio_nread.mem_ptr ), bd_send_to_upper_package_first.All_lenth + 8 );
                    dsp_status = 1;
                    send_all_package_to_upper_lock();
                }
                else
                {
                    DBG("head error\n");
                    DBG("bd_send_to_upper_package_first.head=%x,%x,%x,%x\n", bd_send_to_upper_package_first.head[0], bd_send_to_upper_package_first.head[1], bd_send_to_upper_package_first.head[2], bd_send_to_upper_package_first.head[3]);
                }
                send_upper_buf_flag = 1;
            }
            else if ((send_upper_buf_flag & (0x00000001)) == 0x00000001)
            {
                /* 第二块DSP回读DDR的旧协议数据处理逻辑保持不变。 */
                memcpy(&bd_send_to_upper_package_first, (void*)uio_nread.mem_ptr, SIZE_OF_SEND_UPPER_PACKAGE_FIRST);
                ptr_bd_send_to_upper_package_first->dry_tem = dry_temp;
                if ((bd_send_to_upper_package_first.head[0] == '@') && (bd_send_to_upper_package_first.head[1] == '@') && (bd_send_to_upper_package_first.head[2] == 'S') && (bd_send_to_upper_package_first.head[3] == 'T'))
                {
                    if (bd_send_to_upper_package_first.All_lenth > 5000000)
                    {
                        DBG("bd_send_to_upper_package_first.All_lenth = %d,to big error!!!\n",bd_send_to_upper_package_first.All_lenth);
                        continue;
                    }
                    memcpy((void*)uio_nread.mem_ptr, &bd_send_to_upper_package_first, SIZE_OF_SEND_UPPER_PACKAGE_FIRST);
                    my_copy(Image_Data, (unsigned char*)(uio_nread.mem_ptr), bd_send_to_upper_package_first.All_lenth + 8);
                    dsp_status = 1;
                    send_all_package_to_upper_lock();
                }
                else 
                {
                    DBG("head error\n");
                    DBG("bd_send_to_upper_package_first.head=%x,%x,%x,%x\n", bd_send_to_upper_package_first.head[0], bd_send_to_upper_package_first.head[1], bd_send_to_upper_package_first.head[2], bd_send_to_upper_package_first.head[3]);
                }
                send_upper_buf_flag = 0;
            }
            else
            {
                ;
            }
        }
    }
}
