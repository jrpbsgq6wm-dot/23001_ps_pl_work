#ifndef __DRYTOUPPER_H
#define __DRTTOUPPER_H

extern int IQDataTotalSize, IQDataSizeFromWetSend, IQDataTotalSize_Last, total_len_iqallsize;
/* 旧整帧Image_Data上传路径已由S231 NREAD分块路径替代，不再导出旧缓存。 */
#if 0
extern char Image_Data[25000000];//接收FPGA上传的图像数据
extern char IQData[25000000];//接收湿端上传的IQ数据或者原始数据数组
extern char IQData_to_upper[25000000];//接收湿端上传的IQ数据或者原始数据数组,在处理上1ping的数据,湿端压了1ping
extern char IQData_Last[25000000];
#endif
extern char IQData[30000000];//接收湿端上传的IQ数据或者原始数据数组
extern char IQData_to_upper[30000000];//接收湿端上传的IQ数据或者原始数据数组,在处理上1ping的数据,湿端压了1ping
extern char IQData_Last[30000000];


/****************************回复给上位机的硬件状态信息SS 共计80 字节****************************/
typedef struct
{
	char head[4]; 
	unsigned int pack_len;
	unsigned int DRY_FPGAVersion;//干端FPGA版本号
	unsigned int DRY_LinuxDriverVersion;//干端Linux驱动版本号//-8
	unsigned int DRY_DSP_VERSION;//干端DSP版本号
	unsigned int WET_FPGAVersion;//湿端FPGA版本号 该数据从IQdata中获取
	unsigned int WET_LinuxDriverVersion;//湿端Linux 驱动版本号 该数据从IQdata中获取
	unsigned int WET_EPLD_VERSION;//湿端ZYQN版本号 目前没有使用
	short device_type;
	char sync_mode;
	char sync_type;
	float sync_delay;
/*20240820*/
	char wet_status;
	char dsp_status;
	char dry_fpga_status;
/*20250505*/	
	char GGAZDA_status;
	char Heading_status;
	char TSS1_status;
	char SV_status;
	char PPS_status;
/**********/
	char  Reserved[92];// Byte预留
	char tail[4]; 
}__attribute__((packed)) SEND_UPPER_SONAR_STATUS;


/********************回复给上位机的传感器数据包前16字节******************/
typedef struct
{ 
	unsigned int ExtSensorTotalSize;//总个数
	unsigned int ExtSensorSynchState;//同步状态
	char   Reserved[4];// 预留字节
	unsigned int GGA_ZDA_NUM;
}__attribute__((packed)) SEND_UPPER_SENSOR_FIRST;

/*++++++++++++++++++++++++++++++++++++++波束下放+++++++++++++++++++++++++++++++++++++++*/


/********************ARM回复给上位机的参数数据包 83******************/
typedef struct
{ 
	char head[4];//数据头@@ST
	unsigned int All_lenth;//后面数据总字节数
	unsigned short DataType; //2 Byte  数据类型 0-波形，1-原始数据
	int  device_type;//设备类型
	float ins_angle;//安装倾角
	float Blind_area;//盲区比例
	char Beamform;//Beamform方法
	char Pitch_stability;//纵摇稳定 0：关 1：开
	float quality_filter;//质量滤波
	char Base_Test;//底检测开关 0：关，1开
	char Water_Column;//水柱图像模式 0：关 1:开
    char Image_show_mode;//声呐图显示模式：0：多波束模式 1：前视模式
    float Sidelobe_Factor;//旁瓣因子
	float Ping_Dis;//Ping间距离
	char pitch_com;//纵摇补偿
	char Median_Filter;//中值滤波
	char Threshold_control;//手动门限控制 0：关 1：开
    float Treshold_upper;//门限上限
    float Treshold_lower;//门限下限
    float Treshold_angle;//门限角度
	char INS_mod;//惯导模式选择0：内置 1：外置
	char Beam_Type;//等角等距模式选择 0：等角模式1：等距模式
	char Water_Detection;//水体检测控制 0：禁止 1：中央波束检测 2:全波束检测
	char Focus;//进场聚焦 0：关 1：开
	char roll_com;//横摇补偿
	int ManualGain;   //手动增益的值，-20dB 至60dB
    int AbsorbGainCoef;//吸收系数值，0 至50 dB/Km
    int SpreadGainCoef;//扩散系数值，0 至20 dB/Km
	int wet_tem;//目前该值是从IQdata中取出来
	float dry_tem;//干端传感器温度 目前咩有用到
	float dry_zynq_tem;//干端zynq温度 目前没有用
}__attribute__((packed)) SEND_UPPER_PACKAGE_FIRST;

// Linux 专用：带时间戳的打印宏
#define LOG(fmt, ...) \
    do { \
        struct timeval tv; \
        gettimeofday(&tv, NULL); \
        struct tm* t = localtime(&tv.tv_sec); \
        /* 输出格式：[2025-12-29 10:00:00.123] main.c:20 信息 */ \
        printf("[%04d-%02d-%02d %02d:%02d:%02d.%03ld] %s:%d: " fmt "\n", \
               t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, \
               t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec / 1000, \
               __FILE__, __LINE__, ##__VA_ARGS__); \
    } while (0)


extern SEND_UPPER_SONAR_STATUS  send_upper_status_information;//发送给上位机状态信息----结构体变量
extern SEND_UPPER_SONAR_STATUS* ptr_send_upper_status_information; //发送给上位机状态信息----结构体指针

extern SEND_UPPER_SENSOR_FIRST  send_to_upper_sensor, Recv_Wet_MemsSensorHead;//ARM发送给上位机的传感器数据包头
extern SEND_UPPER_SENSOR_FIRST* ptr_send_to_upper_sensor;//ARM发送给上位机的传感器数据包头
extern SEND_UPPER_SENSOR_FIRST* ptr_Recv_Wet_MemsSensorHead;


extern void dry_to_upper(void);
extern void send_status_to_upper(void);
extern void Debug_pritf_sendtoupper_hardwarepara(SEND_UPPER_SONAR_STATUS* ptr_send_upper_status_information);
extern void Send_IQdataToUpper(void);
/*
 * S231新链路的单帧所有权控制接口。
 * Set_Dsp_TransBuf()在复用单一Frame/BD/NREAD缓冲区前等待上一帧结果发送完成；
 * dry_to_upper()在收到并处理当前PL中断后释放这一帧的NREAD所有权。
 */
extern int S231_WaitNreadIdle(unsigned int timeout_ms);
extern void S231_MarkNreadInFlight(void);
extern void S231_MarkNreadIdle(void);

#endif
