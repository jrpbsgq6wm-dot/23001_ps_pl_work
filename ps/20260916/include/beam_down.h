/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : beam_down.h
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-10-13 17:05:24
 * * 
 * * LastEditTime : 2022-11-02 15:31:25
 ******************************************************************************/
#ifndef __BEAM_DOWN_H
#define __BEAM_DOWN_H

#include "main.h"



typedef struct
{
	unsigned char   SenSorTitle[4];			/* 帧头    0xdddd_dddd 4*/
	unsigned int	Frame_cnt;				/* 帧号            4*/
	unsigned int	PPS_s;    				/* pps计数    	   4*/
	unsigned int	PPS_ms;    				/* pps ms计数      4*/
}INS_SENSOR_DATA_HEAD;


typedef struct
{
	unsigned char Ins_Head[10];
	unsigned char Ins_GnggaHead[10];
	unsigned char Ins_UTCtime[15];
	unsigned char Ins_Lat[20];
	unsigned char Ins_NorS[2];
	unsigned char Ins_Long[20];
	unsigned char Ins_WorE[2];
	unsigned char Ins_Sat_State[2];
	unsigned char Ins_Sat_Num[5];
	unsigned char Ins_HDOP[10];
	unsigned char Ins_Alt[10];
	unsigned char Ins_Alt_Unit[2];
	unsigned char Ins_Undulation[10];
	unsigned char Ins_Ins_Undulation_Unit[2];
	unsigned char Ins_Age[2];
	unsigned char Ins_Age_ID[5];
	unsigned char Ins_GpzdaHead[10];
	unsigned char Ins_Zdatime[15];
	unsigned char Ins_Day[5];
	unsigned char Ins_Mounth[5];
	unsigned char Ins_Year[5];
	unsigned char Ins_Hour[5];
	unsigned char Ins_Min[5];
	unsigned char Ins_HehdtHead[10];
	unsigned char Ins_Heading[10];
	unsigned char Ins_Tss1Head[5];
	unsigned char Ins_Hor_Acc[5];
	unsigned char Ins_Ver_Acc[5];
	unsigned char Ins_Space[2];
	unsigned char Ins_Heave[10];
	unsigned char Ins_horH[2];
	unsigned char Ins_Roll[10];
	unsigned char Ins_Space1[2];
	unsigned char Ins_Pitch[10];
	unsigned char Ins_Ant1_Num[5];
	unsigned char Ins_Ant2_Num[5];
	unsigned char Ins_Imu_State[2];
	unsigned char Ins_Sys_State[2];
	unsigned char Ins_GpVtgHead[10];
	unsigned char Ins_Spd_Speed[10];
	unsigned char Ins_Spd_Unit[2];
}__attribute__((packed)) INS_SENSOR_DATA;

typedef struct
{
	unsigned char   SenSorTitle[4];			/* 帧头    0xdddd_dddd 4*/
	unsigned int	Frame_cnt;				/* 帧号            4*/
	unsigned int	PPS_s;    				/* pps计数    	   4*/
	unsigned int	PPS_ms;    				/* pps ms计数      4*/
}EXT_SENSOR_DATA_HEAD;


typedef struct
{
	unsigned char Ext_GpzdaHead[10];
	unsigned char Ext_Zdatime[15];
	unsigned char Ext_Day[5];
	unsigned char Ext_Mounth[5];
	unsigned char Ext_Year[5];
	unsigned char Ext_Hour[5];
	unsigned char Ext_Min[5];
}__attribute__((packed)) EXT_GPZDA_DATA;

#define INSSENSOR_DATA_PARA_LEN  sizeof(INS_SENSOR_DATA_HEAD)
#define INSSENSOR_DATA_LEN       (1024-16)
//#define INSSENSOR_DATA_LEN       (256-16)
#define EXTSENSOR_DATA_PARA_LEN  sizeof(EXT_SENSOR_DATA_HEAD)
extern INS_SENSOR_DATA *ptr_Ins_Sensor_Data;
extern INS_SENSOR_DATA_HEAD *ptr_InsSensor_Data_Para;


extern void Beam_Down_Data_Manage(void);
void Change_InsModeState_ClearBuf(void);
extern int flag;

#endif
