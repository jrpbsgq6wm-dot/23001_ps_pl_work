/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : main.h
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-11-17 10:58:41
 * * Alter		  ：HouXinBo
 * * AlterTime	  : 2025-8-4
 ******************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H
#include <pthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <semaphore.h>

/******20240820*****/
extern unsigned char wet_status;			//湿端状态
extern unsigned char rs_flag;
extern unsigned char dry_fpga_status;		//干端FPGA状态
extern unsigned char dsp_status;			//DSP状态
extern unsigned char GGAZDA_status;			
extern unsigned char Heading_status;
extern unsigned char TSS1_status;
extern unsigned char SV_status;
extern unsigned char PPS_status;

/*****GeoBeam400M头文件2021.03.20*****/

extern volatile int  server_netStatus ;
extern volatile int  client_netStatus ;
extern volatile int  DryTOWet_Flag;			//干端到湿端标志位
extern pthread_mutex_t mut,mut2;			//互斥锁

extern int irq_on;
extern int Fpga_start_mod;
extern int Flag_ExtSenserBuf ;
extern unsigned int counter;
extern char ZYNQ_VERSION[4];
extern int flag_timer;

extern int Sonar_FristPing_flag;
extern int Bd_Date_TransBuf_Flag;			//波束下放buf切换标志位

extern void thread_wait(void);
extern void thread_create(void);
extern void *thread_tcptrans_link(void *arg);
extern void *thread_wet_to_dry(void *arg);
extern void *thread_dry_to_wet(void *arg);
extern void *thread_dry_to_upper(void *arg);
extern void *thread_upper_to_dry(void *arg);
extern void *thread_temp_sensor(void *arg);
extern void *thread_net_check(void *arg);

extern void my_copy(volatile unsigned char* dst, volatile unsigned char* src, int sz);
extern unsigned short crc16(void* data, unsigned int len, unsigned short crc);
extern void setTimer(int seconds, int mseconds);

extern void *thread_send_to_upper(void *arg);
//extern FILE* fb;


//定义了DEBUG宏，则为调试模式，若注释此行，则不会打印任何的提示信息，最终产品会注释此行
#define DEBUG

/*******************************DEBUG宏定义的说明*****************************************/
#ifdef DEBUG           
#define DBG(...) fprintf(stderr, " DBG(%s, %s(), %d): ", __FILE__, __FUNCTION__, __LINE__); fprintf(stderr, __VA_ARGS__)
#define Debug(fmt,...) printf(fmt,##__VA_ARGS__)
#else                  
#define DBG(...)       
#define Debug(fmt,...) 
#endif       

#define MYSWAP16(x)     ((((*(short int *)&x) & 0xff00)>>8) | (((*(short int *)&x) & 0x00ff)<<8))
#define MYSWAP32(x)     ((((*(long int *)&x) & 0xff000000) >>24) | \
                         (((*(long int *)&x) & 0x00ff0000) >> 8) | \
                         (((*(long int *)&x) & 0x0000ff00) << 8) | \
                         (((*(long int *)&x) & 0x000000ff) << 24))

/*************************FPGA时钟定义*************************************/
#define     FPGA_CLK_FREQUENCY         100000000               //FPGA CLK FREQUENCY   

/*************************大小定义*************************************/
#define   SIZE_OF_USHORT                        2// unsigned short 2字节
#define   SIZE_OF_LONG                          4// unsigned long 和 unsigned long 均为4字节
#define   SIZE_OF_TVG_MAX                       3200//上位机下发的TVG数据的最大字节数800*4
#define   SIZE_OF_STATUS_SEND_UPPER             144//ARM发送给上位机的硬件状态信息共计80字节
#define   SIZE_OF_STATUS_RECV_WET               68//湿端发送给干端硬件信息状态信息共计68字节
#define   SIZE_OF_STATUS_RECV_DSP               24//DSP发送给干端硬件信息状态信息共计24字节
#define   SIZE_OF_CONFIG_PARA                   3683//上位机下发给ARM的参数配置数据包字节数
#define   SIZE_OF_WET_CONFIG_PARA				3368//干端发给湿端配置参数1674字节
#define   SIZE_OF_WET_SONAR_FIRST				128//接收湿端声呐数据参数头长度132字节
#define   SIZE_OF_FPGA_IMAGE_FIRST				58//FPGA声呐图像数据数据头长度
#define   SIZE_OF_ONE_PING_SENSOR               256//一帧传感器包邮256字节
#define   SIZE_OF_SENSOR_FIRST                  16//ARM给上位机上传的传感器前面的固定字节数是16，4字节的总字节数+4字节同步状态+4字节预留+4字节GGA_ZDA
#define   SIZE_OF_LEN_OF_SENSOR_DATA            4//4字节传感器数据总长度
#define   SIZE_OF_SENSOR_WITHOUT_SENSOR_DATA    24//4字节同步状态+4字节预留+4字节GGA_ZDA等条数*4
#define   SIZE_OF_CHANNEL_NUM                   192 //AD的IQ数据通道个数
#define   SIZE_OF_CHANNEL_NUM_ORIGINAL          16  //AD原始数据通道数
#define   SAMPLE_FACTOR                         2  //抽样因子
#define   SAMPLE_FACTOR_8                       8    //原始数据的AD
//#define   SIZE_OF_RAPIDIO_FIRST_FPGA			256	//波束数据参数头fpga写入大小
#define   SIZE_OF_RAPIDIO_FIRST_FPGA			260	//波束数据参数头fpga写入大小
#define   SIZE_OF_RAPIDIO_FIRST                 (260+240+1600+1600+4) //RAPIDIO参数头

#define   SIZE_OF_DATA_FIRST                    256    //FPGA给ARM上传的参数数据字节数
#define   SIZE_OF_AD_SN_MAX                     160020//160000补齐到35的倍数
#define   SIZE_OF_AD_DATA_MAX_BYTE              3511296//最大字节数
#define   SIZE_OF_SEND_UPPER_PACKAGE_FIRST      83    //ARM发送给上位机的声呐数据包前面的128字节参数

#define   SIZE_OF_UART_OEM_RETURN  				16000 //串口配置OEM718D返回数据的长度
#define   SIZE_OF_UART_SENSOR_RETURN  			400   //串口配置其他传感器返回数据的长度
#define   UART_CFG_LEN_FLAG1    				1 //惯导配置字符串长度1(uart_cfg_length1)
#define   UART_CFG_LEN_FLAG2    				2 //惯导配置字符串长度2(uart_cfg_length2)

#define   V_SOUND                               1500   //声速1500m/s
#define   PWM_IP_NUM                            16     //PWM初始相位通道数
#define   SONAR_DATA_OFFSET                     77     //ARM发送给上位机的声呐数据包前面有128字节参数


#define     FPGA_2_30		       1073741824//2^30
#define     FPGA_2_29		       536870912//2^29



#define     FPGA_CLK_FREQUENCY         100000000               //FPGA CLK FREQUENCY    
//#define     PWM_LFM_FACTOR           1801439851  //N2_32*N2_22*D1000000000/FPGA_CLK_FREQUENCY/FPGA_CLK_FREQUENCY
#define     PWM_LFM_FACTOR             3602879702UL  //N2_32*N2_23*D1000000000/FPGA_CLK_FREQUENCY/FPGA_CLK_FREQUENCY
#define     PWM_CW_FACTOR             42.94967296         //N2_32 / FPGA_CLK_FREQUENCY
#define     N2_32                      4294967296              //2的32次方
#define     N2_23                      8388608                 //2的23次方
#define     ENABLE                     1                       
#define     DISABLE                    0                      
#define     TRUE                       0                        
#define     FALSE                      -1  
//#define     BAUD                       38400    

//#define   DDR_SONAR_DATA_OFFSET                 16                     
#define   NUM_OF_IP                             16        //PWM初始相位通道数
#define   CODE_VERSION                          0x00000001//FPGA版本信息
#define   LINUX_VERSION                         2026050701//ARM版本信息
//#define   LINUX_VERSION                         2024101501//ARM版本信息
//#define   LINUX_VERSION                         2024092301//ARM版本信息

#define FAIL -1
#define OK	  0

extern sem_t sem_WET,sem_UPPER;

/****************************FPGA给ARM的DDR图像数据前面的共58字节的参数数据**********************/
typedef struct
{
    unsigned short      Data_Type;//bit0:1声呐图数据，bit1:1侧扫数据，bit2:1底检测数据，bit3:1伪三维数据，bit4:1水柱数据，bit5:1 IQ解调数据，bit6:1原始数据
    char				Work_Mode;//工作模式0-cw       1-LFM
    char				Image_Work_Mode;//显示模式 0:多波束模式，1：前视模式
    char				LFMMode;//0:升频，1:降频
    unsigned int 		PWMFreq;//signal Freq 
    unsigned int		PWMBandWidth;//PWM带宽CW为0
    unsigned int		frameNum;//帧数
    unsigned short		Range;//量程
    unsigned short		SamplingRate;//current K采样率
    unsigned int		PPS_ns;//PPS纳秒计数
    unsigned int        PPS_s;//PPS秒计数
    float				PWMPulseWidth;//CW为0
    char				PWM_Start; //  PWM开关
    char				Image_ratio;//声呐图像对比度
    char				Side_ratio;//侧扫对比度
    unsigned int		Beam_Num;//波束个数
    char				Beam_Type;//等角等距模式选择 0：等角模式1：等距模式
    float				Open_angle;//开角度数
    char				Roll_stability;//横摇稳定 0：关 1：开
    unsigned int		Wet_TMP;//湿端温度
    unsigned int		Dry_TMP;//干端温度
    unsigned int		Sonar_Image_Size;//声呐图像数据长度
}FPGA_DDR_IMAGE_FIRST;//暂时未用




/***************************接收DSP硬件信息结构体24字节**********************/
typedef struct
{
	char DRY_DSP_VERSION[4];//干端DSP版本号
	char Reserved[20];//20 Byte预留
}
__attribute__((packed)) RECV_DSP_SONAR_STATUS;

/**************************波束数据头_字节**********************/
typedef struct
{
	int ping_head;//帧头表示0xaaaa_aaaa
	char  BASH_TEST;//底检测开关
	char SONAR_SHOW;//声呐图显示开关
	char SIDE_MOD;//侧扫模式
	char PTD_MOD;//伪三维模式开关
	float ClolorRatio;//图像对比度
	char INT_MOD;//1024插值模式
	char WATER_DETECTION;//水体检测控制
	char Beamtype;//等角等距模式选择
	char FOCUS;//近场聚焦
	char Water_Dro;//水柱图像模式
	char Image_Work_Mod;//声呐工作模式
	char INS_MOD;//惯导模式选择寄存器
	char Data_Type;//数据类型
	char Roll_Oe;//横摇补偿开关
	char Ctrl_Bow;//艏摇控制
	char Ctrl_Heave;//升沉控制
	float SIDELOBE_FACTOR;//旁瓣因子
	int THR_CON;//手动门限控制
}RAPIDIO_HEAD;


/***************************串口配置结构体**********************/
typedef struct
{
	char              * dev;
	int               nSpeed;
	int               nBits;
	char              nEvent;
	int               nStop;
}COM_CONFIG_PARAMETER;//暂时未用

/***********************************crc16_tab******************************/
static unsigned short crc16_tab[]={
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441, 
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

#endif

