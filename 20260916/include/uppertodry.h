#ifndef __UPPERTODRY_H
#define __UPPERTODRY_H 


/************************************************************************************/
typedef struct
{
//    float power_factor;// 功率系数，目前是4.8-52.8每间隔1v一个，共49个，加上12、24、36、48这4个，共计53个档位
	unsigned char sensor_fh_GGA_ZDA;//GGA_ZDA帧头1字节
	unsigned char sensor_fh_SVT;//SVT帧头1字节
	unsigned char sensor_fh_HEADING;//HEADING帧头1字节
	unsigned char sensor_fh_AT;//AT帧头1字节
	unsigned char sensor_ft_GGA_ZDA[2];//GGA_ZDA帧尾2个字节
    unsigned char sensor_ft_SVT[2];//SVT帧尾2个字节
    unsigned char sensor_ft_HEADING[2];//HEADING帧尾2个字节
    unsigned char sensor_ft_AT[2];//AT帧尾2个字节
//    float  pwm_ip[16]; //初始相位，修改为float型
}__attribute__((packed)) SEND_WET_PAPT;

/******************************同步输入*******************************************/
typedef struct
{
    char syncin_trig;//
    unsigned short syncin_pulse;//
    unsigned short syncin_delay;//
    char syncin_mode;
    unsigned short syncin_period;
    unsigned char rsvd[5];
}__attribute__((packed)) SY_IN_PACK;

/*****************************同步输出**********************************************/
typedef struct
{
    char syncout_trig;
    unsigned short syncout_pulse;
    char syncout_mode;
    char  syncout_moment;
    unsigned short syncout_before;
    unsigned short syncout_after;
    unsigned char rsvd[4];
}__attribute__((packed)) SY_OUT_PACK;

/*****************************风扇工作模式*********************************************/
typedef struct
{
    char fan_mode;
    char fan_speed;
}__attribute__((packed)) FAN_CTRL;

/****************************ARM接收到上位机的参数数据包2086字节 @@SP****************************/
typedef struct
{       
    unsigned int DataType;//bit0:1声呐图数据，bit1:1侧扫数据，bit2:1底检测数据，bit3:1伪三维数据，bit4:1水柱数据，bit5:1 IQ解调数据，bit6:1原始数据
    unsigned int WorkMode;//0:CW,1:LFM
    unsigned int LFMMode;//0:升频，1:降频
    unsigned int PWMFreq;//signal Freq
    unsigned int PWMBandWidth;//PWM带宽CW为0
    unsigned int SamplingRate;//current K采样率
    unsigned int Range;//5m/15m/30m/50m/100m/150m/200m/250m/300m 共9档
    int ManualGain;   //手动增益的值，-20dB 至60dB
    int AbsorbGainCoef;//吸收系数值，0 至50 dB/Km
    int SpreadGainCoef;//扩散系数值，0 至20 dB/Km
    int tvgGain[800];//1600
    float PulseWidth;//脉宽us
    float PingRate; //设置的帧率,如果这里的值为 1，则为最大频率，为 2，则最大帧率/2，3 则为最大帧频/3
    char GNSS_protocol;//GNSS协议
    char SVP_protocol;//SVP协议
    char HEADING_protocol;//HEADING协议
    char MOTION_protocol;//MOTION协议
    char  PWM_Start; //PWM开关
    char INS_mod;//惯导模式选择0：内置 1：外置
    char Base_Test;//底检测开关 0：关，1开
    char Sonar_image;//声呐图像开关 0：关 1：开
    char Side_scan;//侧扫 0：关 1：开
    char TD_mode;//伪三维模式 0：关 1：开
    char Water_Column;//水柱图像模式 0：关 1:开
    char Image_show_mode;//声呐图显示模式：0：多波束模式 1：前视模式
    float Sidelobe_Factor;//旁瓣因子
    char Threshold_control;//手动门限控制 0：关 1：开
    float Treshold_upper;//门限上限
    float Treshold_lower;//门限下限
    float Treshold_angle;//门限角度
    unsigned int Image_width;//声呐图像宽度
    unsigned int Image_heith;//声呐图像高度
    char Focus;//进场聚焦 0：关 1：开
    char interpolation;//1024插值模式 0：关 1：开
    char Water_Detection;//水体检测控制 0：禁止 1：中央波束检测 2:全波束检测
    float Side_ratio;//侧扫对比度
    float Image_ratio;//声呐图像对比度
    unsigned int Beam_Num;//波束个数
    char Beam_Type;//等角等距模式选择 0：等角模式1：等距模式
    char Roll_stability;//横摇稳定 0：关 1：开
    char Pitch_stability;//纵摇稳定 0：关 1：开
    float Manual_SoundSpeed;//1:手动声速 0:自动
    unsigned int AD_NUM;//原始数据通道组数选择
    char roll_com;//横摇补偿
    char pitch_com;//纵摇补偿
    char Beamform;//Beamform方法
    float quality_filter;//质量滤波
    char Median_Filter;//中值滤波
    char Bow_control;//艏摇控制
    char Heave_control;//升沉控制
    float Start_Angle;//开始角度
    float Finish_Angle;//终止角度
    unsigned int CH_beam;//指定波束号
    char CH_test;//通道自检
    char CH_error[240];//异常通道号
    int  device_type;//设备类型
    float ins_angle;//安装倾角
    float Blind_area;//盲区比例
    char device_mode;//设备工作模式0 表示不使用fpga和dsp，1表示设备使用fpga和dsp
    unsigned int GPSBaud;
    unsigned int HEDBaud;  //hangxiang
    unsigned int POSBaud;  //zitai
    unsigned int SVPBaud;  //shengsu
    float power;
	
   	unsigned int TransGear;//发射档位:最大，中间，常用

	char up_iq;//是否上传IQ 默认0不传 1传
	char math_mode;//检测模式
	unsigned int iq_num;//IQ抽样因子 35,70默认70
	unsigned int side_width;
	unsigned int Sgram_num;
	float mid_angle;
	char detection_mode;
	char kernel_num;

	SEND_WET_PAPT wet_sensor;
	char  svp_select;
//以下2025-5-21 15:08:52添加
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

	char  sy_no;

	char syncin_trig;
	unsigned short syncin_pulse;
	unsigned short syncin_delay;
	char syncin_mode;
	unsigned short syncin_period;

    char syncout_trig;
    unsigned short syncout_pulse;
    char syncout_mode;
    char  syncout_moment;
    unsigned short syncout_before;
    unsigned short syncout_after;

    char fan_mode;
    char fan_speed;
    unsigned char mark_baud;
    char  Reserved[7];//8 Byte预留
}__attribute__((packed)) RECV_UPPER_CONFIG_PARAMETERS;


typedef enum
{
    gga = 0,   
    heading,       
    motion,        
    svp,       
    pps,       
    oem,       
} SERSOR_CFG_OFFSET;

typedef enum 
{
    gga_len_offset = 0,   
    hdt_len_offset = 8,       
    mot_len_offset = 16,        
    svp_len_offset = 24,       
    pps_len_offset = 0,       
    oem_len_offset = 8,       
} SERSOR_LEN_OFFSET;


extern SY_IN_PACK  sync_in;
extern SY_OUT_PACK  sync_out;
extern RECV_UPPER_CONFIG_PARAMETERS  recv_upper_package;//接收到上位机的参数配置包----结构体变量
extern RECV_UPPER_CONFIG_PARAMETERS* ptr_recv_upper_package;//接收到上位机的参数配置包----结构体指针
extern RECV_UPPER_CONFIG_PARAMETERS  cmd_package;//接收到上位机的参数配置包保存本地----结构体变量


extern int IPCheck(char *CheckData);
extern void uppertodry(void);
extern void Debug_pritf_receive_para(RECV_UPPER_CONFIG_PARAMETERS* receive_upper_package);

#endif
