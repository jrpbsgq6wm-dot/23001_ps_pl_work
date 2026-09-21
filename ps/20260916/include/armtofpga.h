#ifndef __ARMTOFPGA_H
#define __ARMTOFPGA_H

#include "uppertodry.h"
#include "drytowet.h"
/****************************FPGA参数配置的寄存器总和280+32字节**********************/
typedef struct
{       
    
	unsigned int    Ins_Mod;//惯导模式选择 0：内置惯导 1：外置惯导
	unsigned int    Image_Work_Mode;//显示模式 0:多波束模式，1：前视模式
	unsigned int    Beam_Num;//波束个数
	float           Pitch;//纵摇
	unsigned int    Roll_Oe;//横摇补偿开关
	unsigned int    Focus;//进场聚焦
	unsigned int    Base_Test;//底检测开关
	unsigned int    Sonar_Show;//声呐图显示开关
	unsigned int    Side_Mod;//侧扫模式开关
	unsigned int    PTD_Mod;//伪三维模式开关
	unsigned int    Thr_Con;//手动门限控制开关
	unsigned int    Water_Column;//水柱图像模式 0：关 1:开
	unsigned int    INT_Mod;//1024插值模式
	unsigned int    Water_Detection;//水体检测控制
	unsigned int    Beam_Type;//等角等距模式选择
	float           Up_Limit;//手动门限上限
	float           Down_Limit;//手动门限下限
	float           Angle_Limit;//手动门限倾斜角度
	float           Sidelobe_Factor;//旁瓣因子
	unsigned int    Image_H;//声呐图像高
	unsigned int    Image_W;//声呐图像宽
	float           Image_Rotio;//声呐图像对比度
	float           Side_Rotio;//侧扫对比度
	float           Manual_SoundSpeed;//手动声速
	unsigned int    Beam_Angle;//开角角度值最小的那个值
	unsigned int    Angle_k;//开角角度递增系数
	unsigned int    Data_Type;// 接收数据类型 0 ：IQ数据，1：原始数据
	unsigned int    Mems_Num;//每帧数据下包含的mems个数
	unsigned int    UART_Cfg;//惯导配置标志
	unsigned int    UART_Length1;//惯导配置字符串长度1
	unsigned int    UART_Length2;//惯导配置字符串长度2
	unsigned int    Ctrl_Bow;//艏摇控制
	unsigned int    Ctrl_Heave;//升沉控制
	float           Start_Angle;//起始角度
	float           Finish_Angle;//终止角度
	float           Roll_Value;//横摇值
	unsigned int    Ch_Beam;//波束显示的指定波束波束号
	float           Sync_Longitude;//同步经度
	float           Sync_Latitude;//同步纬度
	float           Sync_Height;//同步高程
    float           Sync_Factor;//同步品质因子
	float           Sync_Factorync_Speed;//同步航速
	float           Sync_Heave;//同步升沉
	float           Ping_Dis;//Ping间距离
	float			Heading;//航向
	unsigned int	Time_Year;//年- time tag
	unsigned int	Time_Date;//日- time tag
	float			Time_Sec;//秒- time tag
	unsigned int	Time_Hours;//时- time tag
	unsigned int	Time_Min;//分- time tag
	unsigned int	Ch_Test;//通道自检
	unsigned int	Ch_Error;//异常通道
	unsigned int	TVG_Gain;//固定增益
	unsigned int	TVG_Absorb;//吸收
	unsigned int	TVG_Spread;//扩散
	unsigned int	Ad_Sn;//采样次数
	unsigned int	Roll_St;//横摇稳定
	unsigned int	Pitch_St;//纵摇稳定
	unsigned int	Beamform_Type;//Beamform方法
	unsigned int	Quality_Filter;//质量滤波
	unsigned int	Device_Type;//设备型号
	float			Install_Angle;//安装倾角
	float			Blind;//盲区比例
	unsigned int	Pitch_Oe;//纵摇补偿
	unsigned int	Median_Filter;//中值滤波
	int				Roll_Coe;//用于横摇补偿系数计算
	unsigned int	Tao_Coe;//用于卷积计算的系数
	unsigned int	Ad_Sn_After;//抽样后采样次数
	unsigned int	Recv_SensorSumNum;//接收的roll值数组现有个数
	unsigned int	up_iq;//是否上传IQ 默认0不传 1传
	unsigned int	math_mode;//检测模式
	unsigned int	iq_num;//IQ抽样因子 35,70默认70
	unsigned int	side_width;
	unsigned int	Sgram_num;
	float			mid_angle;
	unsigned int	detection_mode;
	unsigned int	kernel_num;
}FPGA_CONFIG_PARAMETERS;

/****************************FPGA预留字节**********************/
typedef struct
{
    unsigned int    rsv[32];//预留字节
    unsigned int    repat_neam;//波束形成参数
    unsigned int    beam_num;//波束形成参数
    unsigned int    pulse_dely;//波束形成参数
    unsigned int    read_num;//波束形成参数
    unsigned int    res1;//预留
}FPGA_RSV;

/****************************ARM下发给FPGA的传感器配置**********************/
typedef struct
{
	unsigned int                    GGA_ZDA_BAUD;//GGA和ZDA波特率（预留）
	unsigned int                    HEADING_BAUD;//Heading波特率（预留）
	unsigned int                    AT_BAUD; //AT波特率（预留）
	unsigned int                    SVT_BAUD;//SVT波特率（预留）
	unsigned int			        GNSS_protocol;//GNSS传感器协议
	unsigned int                    SVP_protocol;//SVP速度传感器协议
	unsigned int			        HEADING_protocol;//HEADING航向传感器协议
	unsigned int			        MOTION_protocol;//MOTION姿态传感器协议
	//unsigned int			        SVP_protocol;//SVP速度传感器协议
}FPGA_CONFIG_SENSER_PARAMETERS;

/***************FPGA状态寄存器(当fpga完成本帧的数据传输时此寄存器为0)**************/
typedef struct 
{
	unsigned int  status;//FPGA工作状态寄存器
	unsigned int  date;//FPGA版本日期
	unsigned int  SYSTEM_STATUS;//系统工作状态
}FPGA_STATUS;

/****************************FPGA所有寄存器 共512字节**********************/
typedef struct
{
    unsigned int                    wsm_con;//工作控制寄存器 0：停止 1：开始 2：2：重启
    unsigned int                    set_pr;//参数更新中断
    unsigned int                    IQDataUpdata;//IQ数据更新中断
    FPGA_CONFIG_PARAMETERS          fpga_registers_parameters;// 参数配置寄存器
    FPGA_RSV                        rsv_data;//字节预留
    FPGA_CONFIG_SENSER_PARAMETERS   fpga_sensor;//8*4=32字节
    FPGA_STATUS                     fpga_sta;//12字节
}FPGA_REGISTERS;

/*****************发射上移后湿端的部分PL端功能移植到干端后的寄存器参数 ********/

/****************************湿端ARM接收到上位机的参数数据包 40+3200+16+4+8+16+1+64+19 =3368字节****************************/
typedef struct
{
    unsigned int DataType;//0- 表示带通采样数据，1- 表示 IQ 解调数据
    unsigned int WorkMode;//0:CW,1:LFM
    unsigned int LFMMode;//0:升频，1:降频
    unsigned int PWMFreq;//signal Freq
    unsigned int PWMBandWidth;//CW为0
    unsigned int SamplingRate;//current K
    unsigned int Range;//5m/15m/30m/50m/100m/150m/200m/250m/300m 共9档
    int ManualGain;   //手动增益的值，-40dB 至40dB
    int AbsorbGainCoef;//吸收系数值，0 至120 dB/Km
    int SpreadGainCoef;//扩散系数值，0 至60 dB/Km
    int tvgGain[800];//tvgGain[666];//1600
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
    char  pwm_start; //PWM开关

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
//	float	fsv;
	char	pitch_oe;
	char	Svp_Select;//声速选择寄存器 1:干端 0：湿端 默认0
	char  Reserved[13];//13 Byte
}__attribute__((packed)) WET_RECV_UPPER_CONFIG_PARAMETERS;


/************************工作状态机WSM_SS_REGISTERS******************/
typedef struct 
{
    unsigned int  wsm_mod;//工作模式寄存器 Bit0，0：CW，1：LFM Bit1，0：升频，1：降频 Bit2，0：原始数据，1：处理数据

    unsigned int  wsm_ct;//工作周期寄存器
}WSM_REGISTERS;

/**************************PWM寄存器PWM_SS_REGISTERS 30*4=120个字节***********************/
typedef struct 
{
    unsigned int  pwm_bf;//PWM基频寄存器
    unsigned int  pwm_lfm;//PWM调频寄存器
    unsigned int  pwm_pulse;//PWM脉冲时宽寄存器
    unsigned int	 TransGear;
    unsigned int  sync_state;//0x48
    unsigned int  sync_selec;//0x50
    unsigned int  sync_delaytime;//0x58
    unsigned int	 svp_select;//声速选择寄存器，bit0: 1干端 0：湿端 默认0
    //2025-5-21 16:01:13添加同步输入输出的相关配置
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

    unsigned int  pwm_ip;//PWM初始相位寄存器
    unsigned int  pwm_start;//PWM开关
    unsigned int  power_factor;//功率系数
    unsigned int  pwm_frequency;//PWM中心频率
    unsigned int  sample_rate;//AD采样率
    unsigned int  range;//量程
    unsigned int  bandwidth;//脉宽
    unsigned int  AD_sn_after;//35抽样后的AD_SN
    unsigned int  adc_num_fpga;//ad通道组数选择
    int	phcoe;
    int	final_coe;
    unsigned int	 pitch_oe;
}PWM_REGISTERS;

/***********************************ADC_REGISTERS******************************/
typedef struct 
{
   unsigned int  adc_sct;//adc采样周期寄存器
   unsigned int  adc_sn;//adc采样次数寄存器
}ADC_REGISTERS;

/***********************************DAC_REGISTERS******************************/
typedef struct 
{
   unsigned int  dac_sct;//dac采样周期寄存器
   unsigned int  dac_sn;//dac采样次数寄存器
}DAC_REGISTERS;

/***************FPGA状态寄存器(当fpga完成本帧的数据传输时此寄存器为0)**************/
typedef struct 
{
    unsigned int  wstatus;//FPGA工作状态寄存器
    unsigned int  date;//FPGA版本日期
    unsigned int  version;//FPGA版本信息
}WET_FPGA_STATUS;

/****************************FPGA参数配置的寄存器总和**********************/
typedef struct
{
	WSM_REGISTERS    wsm_registers_value;//2*4
	ADC_REGISTERS    adc_registers_value;//2*4
	DAC_REGISTERS    dac_registers_value;//2*4
	PWM_REGISTERS    pwm_registers_value;//92,不是92应该是108
}WET_FPGA_CONFIG_PARAMETERS;

/****************************ARM下发给FPGA的传感器配置**********************/
typedef struct
{
    unsigned int                    GGA_ZDA_BAUD;//GGA和ZDA波特率
    unsigned int                    HEADING_BAUD;//Heading波特率
    unsigned int                    AT_BAUD; //AT波特率
    unsigned int                    SVT_BAUD;//SVT波特率
    unsigned int                    GGA_ZDA_FH;//GGA和ZDA帧头
    unsigned int                    HEADING_FH;//Heading帧头
    unsigned int                    AT_FH;   //AT帧头
    unsigned int                    SVT_FH;  //SVT帧头
    unsigned int                    GGA_ZDA_FT;//GGA和ZDA帧尾
    unsigned int                    HEADING_FT;//Heading帧尾
    unsigned int                    AT_FT;   //AT帧尾
    unsigned int                    SVT_FT;  //SVT帧尾
}WET_FPGA_CONFIG_SENSER_PARAMETERS;

/****************************FPGA预留字节**********************/
typedef struct
{
    unsigned int                    rsv[11];//预留字节
}WET_FPGA_RSV;



/****************************FPGA所有寄存器 共256字节**********************/
typedef struct
{
	unsigned int                    wsm_con;//4
    unsigned int                    set_pr;//4
    WET_FPGA_CONFIG_PARAMETERS          fpga_registers_200k;// 寄存器结构体变量132字节
    WET_FPGA_RSV                        rsv_data;//56字节
    WET_FPGA_CONFIG_SENSER_PARAMETERS   fpga_sensor;//12*4=48字节
    WET_FPGA_STATUS                     fpga_sta;//12字节
}WET_FPGA_REGISTERS;


/**************************************与FPGA协议相关结构体**************************************/
extern FPGA_REGISTERS fpga_register_data;//下发给FPGA的参数配置寄存器
extern FPGA_REGISTERS* ptr_fpga_register_data;

extern WET_FPGA_REGISTERS wet_fpga_register_data;//下发给FPGA的参数配置寄存器
extern WET_FPGA_REGISTERS *ptr_wet_fpga_register_data;
extern char *ptr_mark_registers;
extern void parsing_upperpara_to_fpga(const RECV_UPPER_CONFIG_PARAMETERS* ptr_recv_upper_package, FPGA_REGISTERS* ptr_fpga_config_para);
extern int parsing_instructions_200k(const WET_RECV_UPPER_CONFIG_PARAMETERS *ptr_recv_upper_package, WET_FPGA_CONFIG_PARAMETERS *ptr_fpga_config_para);
extern void Debug_pritf_convert_fpga_parameter(FPGA_REGISTERS* ptr_fpga_config_para);
extern void Debug_pritf_convert_wet_fpga_parameter(WET_FPGA_CONFIG_PARAMETERS *ptr_fpga_config_para );
extern void Test_SYNC_Time_Roll_Func(void);
#endif
