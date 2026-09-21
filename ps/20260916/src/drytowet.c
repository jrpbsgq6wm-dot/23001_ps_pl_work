/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : drytowet.c
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-10-12 16:15:46
 ******************************************************************************/

#include "main.h"
#include "tcptrans_link.h"
#include "drytoupper.h"
#include "drytowet.h"
#include "tempsensor.h" 
#include "armtofpga.h"
SEND_WET_PARAMETER send_wet_parameter = {0};//发送给湿端参数结构体
SEND_WET_PARAMETER* ptr_send_wet_parameter = &send_wet_parameter;//发送给湿端参数结构体指针
char markbuff[128] = {0};
char mark_tbuff[128] = {0};
char* pmarkbuf = markbuff;
unsigned int r_mark_len = 0;
/*****************************************************************************
 * * description : 接收到的上位机数据解析出湿端配置参数
 * * param        {RECV_UPPER_CONFIG_PARAMETERS*} ptr_recv_upper_package 接收到的上位机的结构体值
 * * param        {SEND_WET_PARAMETER*} ptr_wet_config_para 湿端的配置参数
 * * return       {*}
 * * Date        : 2022-10-09 14:47:16
 * * Other       : 总长度1674 将该参数发送给湿端
 ******************************************************************************/
void parsing_upperpara_to_wet(const RECV_UPPER_CONFIG_PARAMETERS* ptr_recv_upper_package, SEND_WET_PARAMETER* ptr_wet_config_para)
{
    ptr_wet_config_para->DataType = ptr_recv_upper_package->DataType >> 5 & 1;
    ptr_wet_config_para->WorkMode = ptr_recv_upper_package->WorkMode;
    ptr_wet_config_para->LFMMode = ptr_recv_upper_package->LFMMode;
    ptr_wet_config_para->PWMFreq = ptr_recv_upper_package->PWMFreq;
    ptr_wet_config_para->PWMBandWidth = ptr_recv_upper_package->PWMBandWidth;
    ptr_wet_config_para->SamplingRate = ptr_recv_upper_package->SamplingRate;
    ptr_wet_config_para->Range = ptr_recv_upper_package->Range;
    ptr_wet_config_para->ManualGain = ptr_recv_upper_package->ManualGain;
    ptr_wet_config_para->AbsorbGainCoef = ptr_recv_upper_package->AbsorbGainCoef;
    ptr_wet_config_para->SpreadGainCoef = ptr_recv_upper_package->SpreadGainCoef;
    memcpy(ptr_wet_config_para->tvgGain, ptr_recv_upper_package->tvgGain, SIZE_OF_TVG_MAX);
    ptr_wet_config_para->PulseWidth = ptr_recv_upper_package->PulseWidth;
    ptr_wet_config_para->PingRate = ptr_recv_upper_package->PingRate;
    ptr_wet_config_para->PWM_Start = ptr_recv_upper_package->PWM_Start;
    ptr_wet_config_para->AD_NUM = ptr_recv_upper_package->AD_NUM;
    ptr_wet_config_para->Pitch_stability = ptr_recv_upper_package->Pitch_stability;
    ptr_wet_config_para->TransGear = ptr_recv_upper_package->TransGear;
    ptr_wet_config_para->power_factor = ptr_recv_upper_package->power;
   
	ptr_wet_config_para->sensor_fh_GGA_ZDA = ptr_recv_upper_package->wet_sensor.sensor_fh_GGA_ZDA;
    ptr_wet_config_para->sensor_fh_SVT = ptr_recv_upper_package->wet_sensor.sensor_fh_SVT;
    ptr_wet_config_para->sensor_fh_HEADING = ptr_recv_upper_package->wet_sensor.sensor_fh_HEADING;
    ptr_wet_config_para->sensor_fh_AT = ptr_recv_upper_package->wet_sensor.sensor_fh_AT;
    
    memcpy(ptr_wet_config_para->sensor_ft_GGA_ZDA, ptr_recv_upper_package->wet_sensor.sensor_ft_GGA_ZDA,2);
    memcpy(ptr_wet_config_para->sensor_ft_SVT, ptr_recv_upper_package->wet_sensor.sensor_ft_SVT,2);
    memcpy(ptr_wet_config_para->sensor_ft_HEADING, ptr_recv_upper_package->wet_sensor.sensor_ft_HEADING,2);
    memcpy(ptr_wet_config_para->sensor_ft_AT, ptr_recv_upper_package->wet_sensor.sensor_ft_AT,2);
	
	ptr_wet_config_para->sensor_boud_GGA_ZDA = ptr_recv_upper_package->GPSBaud;
    ptr_wet_config_para->sensor_boud_SVT = ptr_recv_upper_package->SVPBaud;
    ptr_wet_config_para->sensor_boud_HEADING = ptr_recv_upper_package->HEDBaud;
    ptr_wet_config_para->sensor_boud_AT = ptr_recv_upper_package->POSBaud;
  
	ptr_wet_config_para->svp_select = ptr_recv_upper_package->svp_select;

    ptr_wet_config_para->gnss_in = ptr_recv_upper_package->gnss_in;
    ptr_wet_config_para->gnss_level = ptr_recv_upper_package->gnss_level;
    ptr_wet_config_para->heading_in = ptr_recv_upper_package->heading_in;
    ptr_wet_config_para->heading_level = ptr_recv_upper_package->heading_level;
    ptr_wet_config_para->motion_in = ptr_recv_upper_package->motion_in;
    ptr_wet_config_para->motion_level = ptr_recv_upper_package->motion_level;
    ptr_wet_config_para->pps_in = ptr_recv_upper_package->pps_in;
    ptr_wet_config_para->pps_level = ptr_recv_upper_package->pps_level;
    ptr_wet_config_para->svp_in = ptr_recv_upper_package->svp_in;
    ptr_wet_config_para->svp_level = ptr_recv_upper_package->svp_level;
    ptr_wet_config_para->fan_mode = ptr_recv_upper_package->fan_mode;
    ptr_wet_config_para->fan_speed = ptr_recv_upper_package->fan_speed;
    ptr_wet_config_para->sy_no = ptr_recv_upper_package->sy_no;
    ptr_wet_config_para->syncin_delay = ptr_recv_upper_package->syncin_delay;
    ptr_wet_config_para->syncin_mode = ptr_recv_upper_package->syncin_mode;
    ptr_wet_config_para->syncin_period = ptr_recv_upper_package->syncin_period;
    ptr_wet_config_para->syncin_pulse = ptr_recv_upper_package->syncin_pulse;
    ptr_wet_config_para->syncin_trig = ptr_recv_upper_package->syncin_trig;
    ptr_wet_config_para->syncout_after = ptr_recv_upper_package->syncout_after;
    ptr_wet_config_para->syncout_before = ptr_recv_upper_package->syncout_before;
    ptr_wet_config_para->syncout_mode = ptr_recv_upper_package->syncout_mode;
    ptr_wet_config_para->syncout_moment = ptr_recv_upper_package->syncout_moment;
    ptr_wet_config_para->syncout_pulse = ptr_recv_upper_package->syncout_pulse;
    ptr_wet_config_para->syncout_trig = ptr_recv_upper_package->syncout_trig;

}

void Debug_printf_upperpara_to_wet(SEND_WET_PARAMETER* ptr_wet_config_para)
{
    Debug("=================send para to wet start =================\n");
    Debug("ptr_wet_config_para->DataType = %d\n",ptr_wet_config_para->DataType);
    Debug("ptr_wet_config_para->WorkMode  = %d\n", ptr_wet_config_para->WorkMode );
    Debug("ptr_wet_config_para->LFMMode  = %d\n",ptr_wet_config_para->LFMMode );
    Debug("ptr_wet_config_para->PWMFreq  = %d\n", ptr_wet_config_para->PWMFreq);
    Debug("ptr_wet_config_para->PWMBandWidth  = %d\n",ptr_wet_config_para->PWMBandWidth );
    Debug("ptr_wet_config_para->SamplingRate  = %d\n", ptr_wet_config_para->SamplingRate );
    Debug("ptr_wet_config_para->Range  = %d\n",ptr_wet_config_para->Range );
    Debug("ptr_wet_config_para->ManualGain = %d\n",ptr_wet_config_para->ManualGain);
    Debug("ptr_wet_config_para->AbsorbGainCoef = %d\n",ptr_wet_config_para->AbsorbGainCoef);
    Debug("ptr_wet_config_para->SpreadGainCoef = %d\n",ptr_wet_config_para->SpreadGainCoef);
    Debug("ptr_wet_config_para->PulseWidth  = %f\n",ptr_wet_config_para->PulseWidth );
    Debug("ptr_wet_config_para->PingRate  = %f\n",ptr_wet_config_para->PingRate );
    Debug("ptr_wet_config_para->PWM_Start = %d\n",ptr_wet_config_para->PWM_Start );
    Debug("ptr_wet_config_para->AD_NUM  = %d\n",ptr_wet_config_para->AD_NUM );
    Debug("ptr_wet_config_para->Pitch_stability  = %d\n",ptr_wet_config_para->Pitch_stability );
    Debug("=================send para to wet end =================\n");
}


/*****************************************************************************
 * * description : 干端发送到湿端的配置参数
 * * param        {SEND_WET_PARAMETER*} ptr_wet_config 发送到湿端的参数
 * * return       {*}
 * * Date        : 2022-10-09 14:48:23
 * * Other       : 总长度1674
 ******************************************************************************/
void send_wet_parameter_func(SEND_WET_PARAMETER* ptr_wet_config)
{
    char wet_parameter_head[4] = { '<','<','S','P' };
    char wet_parameter_tail[4] = { 'E','D','>','>' };

    unsigned int crc;
    int send_num;
    if (send(Socket_fd_wet, wet_parameter_head, SIZE_OF_LONG, 0) < 0)//4byte
    {
        error_process();
        return;
    }
    if ((send_num = send(Socket_fd_wet, (char*)ptr_wet_config, SIZE_OF_WET_CONFIG_PARA, 0)) < 0)//1674byte
    {
        error_process();
        return;
    }
    crc = crc16((char*)(ptr_wet_config), SIZE_OF_WET_CONFIG_PARA, 0xFFFF);
    if (send(Socket_fd_wet, (char*)&crc, 4, 0) < 0)//4byte
    {
        error_process();
        return;
    }
    if (send(Socket_fd_wet, wet_parameter_tail, SIZE_OF_LONG, 0) < 0)//4byte
    {
        error_process();
        return;
    }
}

void send_mark_func(char* markbuff, unsigned int len)
{
    char mark_head[4] = { '@','@','M','K' };
    unsigned int crc;
    int send_num;
    if (send(Connect_fd_upper, mark_head, sizeof(mark_head), 0) < 0)//4byte
    {
        error_process();
        return;
    }
    if (send(Connect_fd_upper,  &((unsigned int){htonl((unsigned int)len)}), sizeof(len), 0) < 0)//4byte
    {
        error_process();
        return;
    }
    if (send(Connect_fd_upper, markbuff, len, 0) < 0)//4byte
    {
        error_process();
        return;
    }
}

/*****************************************************************************
 * * description : 干端发送到湿端的线程
 * * return       {*}
 * * Date        : 2022-10-09 14:49:11
 * * Other       : 接受到显控的参数后将显控的参数下发到湿端 @@SP
 ******************************************************************************/
void dry_to_wet(void)
{
    unsigned int i = 0;
    unsigned int  ist = 0;
    unsigned int len = 0;
    int ret;
    NST175_Init();
    setTimer(0, 500000);//500ms
    memset(markbuff, 0, 128);
    memset(ptr_mark_registers, 0, 128);
    unsigned int rmarktotal = 0;
    char *ptrhead, *ptrtail;
    while (1)
    {
        if (DryTOWet_Flag == 1)
        {
            parsing_upperpara_to_wet(&cmd_package, &send_wet_parameter); //解析显控配置信息
            send_wet_parameter_func(&send_wet_parameter);//发送给湿端
            //Debug_printf_upperpara_to_wet(&send_wet_parameter);
            DryTOWet_Flag = 0;

	    }
        if (i2c_read_reg != 0)
        {
            NST175_Init();
            i2c_read_reg = 0;
        }
        else if ((flag_timer == 1) && (Fpga_start_mod == 1))
        {
            flag_timer = 0;
            setTimer(0, 500000);//500ms
        //    i2c_read_reg = i2c_read(fd_icc, REG_TEMP, &remote_high_value);//读取远程温度值-高位
            get_sensor_data(); //从FPGA读取数据,需要知道GGA传感器协议。
            if (1)
            {
                memcpy(markbuff, ptr_mark_registers, 64);
                ptrhead = strstr(markbuff, "$EVENT");
                if (ptrhead != NULL)
                {
                    ptrtail = strchr(markbuff+6,'E');
                    if (ptrtail == NULL)
                    {
                        ptrtail = strchr(markbuff+6,'W');
                    }
                    if (ptrtail != NULL)
                    {
                        memcpy(mark_tbuff, ptrhead, ptrtail - ptrhead + 1);
                        len = (int)(ptrtail - ptrhead + 1);
                        send_mark_func(mark_tbuff, len);//发送给湿端
                        DBG("Mark Info:%s",mark_tbuff);
    //                    memset(ptr_mark_registers, 0, 128);
                    }
                }
            }
        }
        else
        {
            usleep(1000);
        }
	//	sleep(3);
 /*      if (flag_timer == 1)
        {
            flag_timer = 0;
            setTimer(0, 500000);//500ms 
        //    i2c_read_reg = i2c_read(fd_icc, REG_TEMP, &remote_high_value);//读取远程温度值-高位
            get_sensor_data(); //从FPGA读取数据,需要知道GGA传感器协议。
        }
        else
        {
            usleep(1000);
        }*/ 
    }
}



