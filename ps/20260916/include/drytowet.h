#ifndef __DRYTOWET_H
#define __DRYTOWET_H
#include "uppertodry.h"

#define _GNU_SOURCE     //在源文件开头定义_GNU_SOURCE宏
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <termios.h>

typedef struct uart_hardware_cfg {
    unsigned int baudrate;      /* 波特率 */
    unsigned char dbit;         /* 数据位 */
    char parity;                /* 奇偶校验 */
    unsigned char sbit;         /* 停止位 */
} uart_cfg_t;

struct termios old_cfg;     //用于保存终端的配置参数

/*************************干端发送给湿端参数配置信息40+3200+16+4+8+16+1+64+19=3368字节**********************/
typedef struct
{
	unsigned int DataType;//请求的数据类型0:原始1:IQ
	unsigned int WorkMode;//0:CW,1:LFM
	unsigned int LFMMode;//0:升频，1:降频
	unsigned int PWMFreq;//signal Freq
	unsigned int PWMBandWidth;//PWM带宽CW为0
	unsigned int SamplingRate;//current K采样率
	unsigned int Range;//5m/15m/30m/50m/100m/150m/200m/250m/300m 共9档
	int ManualGain;   //手动增益的值，-20dB 至60dB
	int AbsorbGainCoef;//吸收系数值，0 至50 dB/Km
	int SpreadGainCoef;//扩散系数值，0 至20 dB/Km
	int tvgGain[800];////1600
    unsigned int TransGear;//发射档位:最大，中间，常用
	float PulseWidth;//脉宽us
	float PingRate; //设置的帧率,如果这里的值为 1，则为最大频率，为 2，则最大帧率/2，3 则为最大帧频/3
    float power_factor;// 功率系数，目前是4.8-52.8每间隔1v一个，共49个，加上12、24、36、48这4个，共计53个档位
	unsigned char sensor_fh_GGA_ZDA;//GGA_ZDA帧头1字节
	unsigned char sensor_fh_SVT;//SVT帧头1字节
	unsigned char sensor_fh_HEADING;//HEADING帧头1字节
	unsigned char sensor_fh_AT;//AT帧头1字节
	unsigned char sensor_ft_GGA_ZDA[2];//GGA_ZDA帧尾2个字节
    unsigned char sensor_ft_SVT[2];//SVT帧尾2个字节
    unsigned char sensor_ft_HEADING[2];//HEADING帧尾2个字节
    unsigned char sensor_ft_AT[2];//AT帧尾2个字节
    unsigned int  sensor_boud_GGA_ZDA;//GGA_ZDA波特率
    unsigned int  sensor_boud_SVT;//SVT波特率
    unsigned int  sensor_boud_HEADING;//HEADING波特率
    unsigned int  sensor_boud_AT;//AT波特率
	char  PWM_Start; //PWM开关
    //2025-5-21 16:26:13添加同步输入输出等参数，共40个字节，占用pwm_ip数组中的10个成员位置
    char  sensor_config;
    char  gnss_in;
    char  svp_in;
    char  heading_in;

    char  motion_in;
    char  pps_in;
    char  gnss_level;
    char  svp_level;

    char  heading_level;
    char  motion_level;
    char  pps_level;
    char  rsvd;

    char syncin_trig;
    char syncin_mode;
    unsigned short syncin_pulse;

    unsigned short syncin_period;
    unsigned short syncin_delay;

    char syncout_trig;
    char syncout_mode;
    unsigned short syncout_pulse;

    unsigned int  syncout_moment;

    unsigned short syncout_before;
    unsigned short syncout_after;

    unsigned int  sy_no;

    char fan_mode;
    char fan_speed;
    unsigned short usrsvd;

    float  pwm_ip[6]; //初始相位，修改为float型
	unsigned int AD_NUM;//原始数据通道组数选择
    char Pitch_stability;//纵摇稳定 0：关 1：开
	char  svp_select;
	char Reserved[13];//预留
}__attribute__((packed)) SEND_WET_PARAMETER;
extern unsigned char mark_baudrate;
extern SEND_WET_PARAMETER send_wet_parameter;//发送给湿端参数结构体
extern SEND_WET_PARAMETER* ptr_send_wet_parameter;//发送给湿端参数结构体指针

extern void parsing_upperpara_to_wet(const RECV_UPPER_CONFIG_PARAMETERS* ptr_recv_upper_package, SEND_WET_PARAMETER* ptr_wet_config_para);
extern void send_wet_parameter_func(SEND_WET_PARAMETER* ptr_wet_config);
extern void dry_to_wet(void);




#endif
