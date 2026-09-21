/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : eeprom.h
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-12-08 10:07:16
 ******************************************************************************/

#ifndef __EEPROM_H__
#define __EEPROM_H__

#define I2C_SLAVE_FORCE     0x0706			//IIC从器件的地址设置
#define EEPROM_DEV_ADDR		0x50	//地址（设备地址）
#define EEPROM_W		0xA0
#define EEPROM_R		0xA1

#define DRY_DEVICE 		0x0000
#define DRY_IP 			0x000a
#define DRY_IP_LEN 		0x001a
#define DRY_MAC 		0x001d

#define WET_DEVICE 		64
#define WET_IP 			74
#define WET_IP_LEN 		90
#define WET_MAC 		93


#define SN 128 


typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;

extern int fd_eeprom;

typedef struct
{ 
	uint8_t head[4];
	char flag;
	int size;
	uint8_t device[10];
	uint8_t	address[16];
	uint8_t len[3];
	uint8_t mac[18];
}TCPDataTypeT;


extern TCPDataTypeT ifcfgIP_DRY,ifcfgIP_WET;

extern int EEPROM_Write_Byte(uint16_t Addr, uint8_t recv_data[128]);
extern uint8_t EEPROM_Read_Byte(uint16_t Addr);
extern uint8_t EEP_Read_Data[128];
extern void WriteRcConfFile(const char* cfg,const char* value);
extern void WR_Device();
extern void Eeprom_Init(void);
extern int WriteWetIP(char *newip);
extern int WriteDryIP(char *newip);
extern void ReadAll(void);
extern void ReadDry(void);

extern int WriteDryMAC(char *newmac);
extern int WriteWetMAC(char *newmac);

#endif 
