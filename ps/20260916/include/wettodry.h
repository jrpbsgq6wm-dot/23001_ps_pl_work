/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : wettodry.h
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-10-27 15:01:12
 ******************************************************************************/
#ifndef __WETTODRY_H
#define __WETTODRY_H

/***************************Dry TO Wet Agreement Date frist head 60bit <<st **********************/
typedef struct
{
	unsigned short DataType;//0:primitive data 1:iq data
	unsigned short WorkMode;// 0:CW,1:LFM
	unsigned short LFMMode;
	unsigned int PWMFreq;//signal Freq
	unsigned int PWMBandWidth;
	unsigned int PingCount;//4  Byte 
	unsigned short Range;//2 Byte 
	unsigned short Sampling;
	unsigned int TimeStamp;//4 Byte 纳秒计数
	unsigned int PPS_Number;//4 Byte pps
	float PWMPulseWidth;//CW
	float power_factor;
	char PWM_Start;//PWM
	int WET_FPGAVersion;
	int WET_LinuxDriverVersion;
	float buf[14];//预留
	float temperature;
	float roll;
	float pitch;
	float heading;
	unsigned int  Svp_Value;
	char Transgear;
	unsigned int IQ_Data_Length;//IQ声学数据长度，不包含前面的128字节参数头，也不表示传感器数据长度
}__attribute__((packed)) RECV_WET_SONAR_FIRST;


typedef struct
{
	unsigned int WET_FPGAVersion;
	unsigned int WET_LinuxDriverVersion;
	char PPSWorkStatus;
	char GPSWorkStatus;
	char TimeSysWorkStatus;
	char PostureWorkStatus;
	char SoundVelocityWorkStatus;
	char CourseWorkStatus;
	int  Range;
	int ManualGain;
	int AbsorbGainCoef;
	int SpreadGainCoef;
	int Transgear;
	char SignalType;
	float PulseWidth;
	int BoardTemperature;
	int ChamberTemperature;
	int power_factor;
	char PWM_Start; 
	unsigned int EPLD_VERSIONS;
	char sync_state;
	char sync_selec;
	float sync_delaytime;
	char  Reserved[6];//20 Byte
}__attribute__((packed)) RECV_WET_SONAR_STATUS;

extern int Sensor1_Num, Sensor2_Num, Sensor3_Num, Sensor4_Num, sensor4_num_last;

extern RECV_WET_SONAR_FIRST  IQData_ParaHead;
extern RECV_WET_SONAR_FIRST* ptr_IQData_ParaHead;
extern RECV_WET_SONAR_FIRST  IQData_ParaHead_Last;
extern RECV_WET_SONAR_FIRST* ptr_IQData_ParaHead_Last;
extern RECV_WET_SONAR_STATUS  wet_sonar;
extern RECV_WET_SONAR_STATUS* ptr_wet_sonar;
extern int Wet_to_Dry_flag;


extern void ReadWetMemsDatetoUpper(void);
extern void UpdateWetFlagtoUpper(void);
extern void wet_to_dry(void);
extern void RevIQData_SetIQDataParaHead(void);
extern void Lookfor_Four_Sensor_Num(const int* ptr_sensor1, const int* ptr_sensor2, const int* ptr_sensor3, const int* ptr_sensor4);
extern void ReadWetIPDatetoUpper(void);
extern void ReadWetMACDatetoUpper(void);
extern void ReadWetIPDatetoUpper1(void);
extern void ReadWetMACDatetoUpper1(void);
extern void WriteWetIPF();
extern void WriteWetMACF();
extern void recvwetsonarstatus();

extern void iq_to_upper();
#endif
