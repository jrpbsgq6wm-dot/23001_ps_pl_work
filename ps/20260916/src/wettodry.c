/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : wettodry.c
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-12-08 10:06:21
 ******************************************************************************/

#include "main.h"
#include "tcptrans_link.h"
#include "uppertodry.h"
#include "drytoupper.h"
#include "upgrade_program.h"
#include "uppertodry.h"
#include "wettodry.h"
#include "fpga_init.h"
#include "tempsensor.h"
#include "armtodsp.h"
#include "beam_down.h"
#include "eeprom.h"



int Sensor1_Num = 0, Sensor2_Num = 0, Sensor3_Num = 0, Sensor4_Num = 0;
int Wet_to_Dry_flag = 0;

RECV_WET_SONAR_FIRST  IQData_ParaHead = {0};//接收湿端声呐数据参数头结构体
RECV_WET_SONAR_FIRST* ptr_IQData_ParaHead = &IQData_ParaHead;

RECV_WET_SONAR_FIRST  IQData_ParaHead_Last = {0};
RECV_WET_SONAR_FIRST* ptr_IQData_ParaHead_Last = &IQData_ParaHead_Last;

RECV_WET_SONAR_STATUS  wet_sonar = {0};
RECV_WET_SONAR_STATUS* ptr_wet_sonar = &wet_sonar;

/*****************************************************************************
 * * description : 配置MEMS参数,将接收到的MEMS信息发送给显控
 * * return       {*}
 * * Date        : 2022-10-09 17:12:53
 * * Other       : 对应的显控下发的命令为@@SC_ME_BG
 ******************************************************************************/
void ReadWetMemsDatetoUpper(void)
{
    int Sensor_Mems_Lenth;
    int Sensor_Mems_Return[20000];
    char Sensor_Mems_Head[4] = { '#','#','S','E' };

    DBG("##SE\n");
    
    if (recv(Socket_fd_wet, (char*)&Sensor_Mems_Lenth, 4, 0) <= 0)
    {
        error_process();
    }
    if (recv(Socket_fd_wet, Sensor_Mems_Return, Sensor_Mems_Lenth, 0) <= 0)
    {
        error_process();
    }
    pthread_mutex_lock(&mut);
    if (send(Connect_fd_upper, Sensor_Mems_Head, SIZE_OF_LONG, 0) < 0)//##SE
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    if (send(Connect_fd_upper, (char*)&Sensor_Mems_Lenth, SIZE_OF_LONG, 0) < 0)
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    if (send(Connect_fd_upper, Sensor_Mems_Return, Sensor_Mems_Lenth, 0) < 0)
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
				//		ifcfgIP_DRY.size[1] = '8'; 
    pthread_mutex_unlock(&mut);
    Debug("**********Sensor Mems Return End****************\n");
}

/*****************************************************************************
 * * description : 接收到湿端更新程序成功标志位后将该标志位发送给显控
 * * return       {*}
 * * Date        : 2022-10-09 17:11:36
 * * Other       : 含湿端MEMS与湿端ZYNQ
 ******************************************************************************/
void UpdateWetFlagtoUpper(void)
{
    int UpdateFlag;
    char Update_WET_Head[4] = { '#','#','S','U' };

    DBG("##SU\n");
    if (recv(Socket_fd_wet, (char*)&UpdateFlag, 4, 0) <= 0)
    {
        error_process();
    }
    pthread_mutex_lock(&mut);
    if (send(Connect_fd_upper, Update_WET_Head, SIZE_OF_LONG, 0) < 0)
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    if (send(Connect_fd_upper, (char*)&UpdateFlag, SIZE_OF_LONG, 0) < 0) //4byte
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    pthread_mutex_unlock(&mut);
    printf("**********Send WET Update Flag End****************\n");
}

void ReadWetIPDatetoUpper(void)
{
    char Update_WET_Head[4] = { '<','<','I','P' };
    TCPDataTypeT GetifcfgIP;

    DBG("<<IP\n");
    
    memset(&GetifcfgIP, 0x00, sizeof(GetifcfgIP));
    if (recv(Socket_fd_wet, (char*)&GetifcfgIP, sizeof(GetifcfgIP), 0) <= 0)
    {
        error_process();
    }
    WriteWetIP(GetifcfgIP.address);
    pthread_mutex_lock(&mut);
//	GetifcfgIP.size[0] = '4'; 
//	GetifcfgIP.size[1] = '8';
	GetifcfgIP.flag = 'R';
	GetifcfgIP.size = 48; 
	strncpy(GetifcfgIP.head,Update_WET_Head,4);//hw_dry_head
//    if (send(Connect_fd_upper, Update_WET_Head, SIZE_OF_LONG, 0) < 0)//ip_wet_head
//    {
//        error_process();
//        pthread_mutex_unlock(&mut);
//    }
    if (send(Connect_fd_upper, (char*)&GetifcfgIP, sizeof(GetifcfgIP), 0) < 0) //ipaddr
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    pthread_mutex_unlock(&mut);
    printf("*****DEVICE_WEY*****%s\n",GetifcfgIP.device);
	printf("*****IP_LEN_WEY*****%s\n",GetifcfgIP.len);
	printf("******IP_WEY********%s\n",GetifcfgIP.address);
	printf("******MAC_WEY*******%s\n",GetifcfgIP.mac);
    printf("**********Send WET IP Addr End****************\n");
}

void ReadWetMACDatetoUpper(void)
{
    char Update_WET_Head[4] = { '<','<','H','W' };
    TCPDataTypeT GetifcfgHW;

    DBG("<<HW\n");
    
    memset(&GetifcfgHW, 0x00, sizeof(GetifcfgHW));
    if (recv(Socket_fd_wet, (char*)&GetifcfgHW, sizeof(GetifcfgHW), 0) <= 0)
    {
        error_process();
    }
    WriteWetMAC(GetifcfgHW.mac);
    pthread_mutex_lock(&mut);
	GetifcfgHW.flag = 'R';
	GetifcfgHW.size = 48; 
	strncpy(GetifcfgHW.head,Update_WET_Head,4);//hw_dry_head
//    if (send(Connect_fd_upper, Update_WET_Head, SIZE_OF_LONG, 0) < 0)//hw_wet_head
//    {
//        error_process();
//       pthread_mutex_unlock(&mut);
//    }
    if (send(Connect_fd_upper, (char*)&GetifcfgHW, sizeof(GetifcfgHW), 0) < 0) //ipaddr
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    pthread_mutex_unlock(&mut);
    printf("*****DEVICE_WEY*****%s\n",GetifcfgHW.device);
	printf("*****IP_LEN_WEY*****%s\n",GetifcfgHW.len);
	printf("******IP_WEY********%s\n",GetifcfgHW.address);
	printf("******MAC_WEY*******%s\n",GetifcfgHW.mac);
    printf("**********Send WET IP Addr End****************\n");
}

void ReadWetIPDatetoUpper1(void)
{
	char WriteTUpper[8] = {'<','<','I','P','W'};
    if (send(Connect_fd_upper, WriteTUpper, 8, 0) < 0) //写WET IP
    {
        error_process();
    }
    if (send(Connect_fd_upper, "T", 1, 0) < 0) //写成功返回T
    {
        error_process();
    }
}

void ReadWetMACDatetoUpper1(void)
{
    
	char WriteTUpper[8] = {'<','<','H','W','W'};
    if (send(Connect_fd_upper, WriteTUpper, 8, 0) < 0) //
    {
        error_process();
    }
    if (send(Connect_fd_upper, "T", 1, 0) < 0) //
    {
        error_process();
    }
}

void WriteWetIPF(void)
{
	char WriteTUpper[8] = {'<','<','I','P','W'};
    if (send(Connect_fd_upper, WriteTUpper, 8, 0) < 0) //
    {
        error_process();
    }
    if (send(Connect_fd_upper, "F", 1, 0) < 0) //写失败返回F
    {
        error_process();
    }
}

void WriteWetMACF(void)
{
	char WriteTUpper[8] = {'<','<','H','W','W'};
    if (send(Connect_fd_upper, WriteTUpper, 8, 0) < 0) //
    {
        error_process();
    }
    if (send(Connect_fd_upper, "F", 1, 0) < 0) //写失败返回F
    {
        error_process();
	}
}
void recvwetsonarstatus(void)
{
	char status[200]; 
	char tail[4]; 
    if (recv_socket(Socket_fd_wet, status, SIZE_OF_WET_SONAR_FIRST) <= 0)//回波数据IQdata
    {
        error_process();
	}
    if (recv(Socket_fd_wet,tail,4,0) <= 0)
    {
        error_process();
	}
	memcpy(&wet_sonar,status,SIZE_OF_WET_SONAR_FIRST);
} 

/*****************************************************************************
 * * description : 读取IQ数据参数头,修改后又重新打包
 * * return       {*}
 * * Date        : 2022-10-20 13:37:59
 * * Other
 ******************************************************************************/
void RevIQData_SetIQDataParaHead(void)
{
    if (recv(Socket_fd_wet, (char*)&IQDataSizeFromWetSend, 4, 0) <= 0)//<<ST后4byte长度,没算外置传感器与SVP的长度
    {
        error_process();
    }
    if (recv_socket(Socket_fd_wet, IQData, IQDataSizeFromWetSend) <= 0)//回波数据IQdata
    {
        error_process();
    }
    memcpy(&IQData_ParaHead, IQData, SIZE_OF_WET_SONAR_FIRST);//60byte IQDataSizeFromWetSend -> 预留3byte

    if (counter == 0)//湿端版本号由干端上传,从IQdate里取
    {
        ptr_send_upper_status_information->WET_FPGAVersion = IQData_ParaHead.WET_FPGAVersion;
        ptr_send_upper_status_information->WET_LinuxDriverVersion = IQData_ParaHead.WET_LinuxDriverVersion;
		++counter;
	}
    memcpy(IQData, ptr_IQData_ParaHead, SIZE_OF_WET_SONAR_FIRST);
}


/*****************************************************************************
 * * description : 遍历外置传感器及声速
 * * param        {int*} ptr_sensor1
 * * param        {int*} ptr_sensor2
 * * param        {int*} ptr_sensor3
 * * param        {int*} ptr_sensor4  SVP
 * * return       {*}
 * * Date        : 2022-10-20 13:39:06
 * * Other
 ******************************************************************************/
void Lookfor_Four_Sensor_Num(const int* ptr_sensor1, const int* ptr_sensor2, const int* ptr_sensor3, const int* ptr_sensor4)
{
    int m;
    for (m = 0; m < 50; m++)
    {
        if ((*(ptr_sensor1 + m * 64) == 0xDDDDDDDD))
        {
            Sensor1_Num++;
        }
        if ((*(ptr_sensor2 + m * 64) == 0xDDDDDDDD))
        {
            Sensor2_Num++;
        }
        if ((*(ptr_sensor3 + m * 64) == 0xDDDDDDDD))
        {
            Sensor3_Num++;
        }
        if ((*(ptr_sensor4 + m * 64) == 0xDDDDDDDD))
        {
            Sensor4_Num++;
        }
    }
	   DBG("Sensor1_Num is %d\n",Sensor1_Num);
       DBG("Sensor3_Num is %d\n",Sensor3_Num);
}
/*****************************************************************************
 * * description : 写接收到的各外置传感器的数量,GGA_ZDA_NUM数量这里置为0,后面写
 * * return       {*}
 * * Date        : 2022-10-20 13:39:47
 * * Other
 ******************************************************************************/
void Copy_ExtSensor_NumToFpgaDdr(void)
{
    *((int*)(IQData + IQDataSizeFromWetSend - 6 + SIZE_OF_SENSOR_FIRST + Sensor1_Num * SIZE_OF_ONE_PING_SENSOR)) = 0;
    *((int*)(IQData + IQDataSizeFromWetSend - 6 + SIZE_OF_SENSOR_FIRST + SIZE_OF_LONG + Sensor1_Num * SIZE_OF_ONE_PING_SENSOR)) = Sensor2_Num;
    *((int*)(IQData + IQDataSizeFromWetSend - 6 + SIZE_OF_SENSOR_FIRST + SIZE_OF_LONG * 2 + (Sensor1_Num + Sensor2_Num) * SIZE_OF_ONE_PING_SENSOR)) =Sensor3_Num;
    *((int*)(IQData + IQDataSizeFromWetSend - 6 + SIZE_OF_SENSOR_FIRST + SIZE_OF_LONG * 3 + (Sensor1_Num + Sensor2_Num +Sensor3_Num) * SIZE_OF_ONE_PING_SENSOR)) = Sensor4_Num;
}

/*****************************************************************************
 * * description : 湿端发送给干端的线程函数
 * * return       {*}
 * * Date        : 2022-10-20 13:31:36
 * * Other       : 融合了波束下放及波束未下放代码
 ******************************************************************************/
void wet_to_dry(void)
{
    char receive_Wet_DataHead[4];
    int Len_Recv_Head;

    //fb = fopen("roll.dat", "wb");

	Change_InsModeState_ClearBuf();
	while (1)
	{
		if ((client_netStatus == 1) && (server_netStatus == 1))
		{
			memset(receive_Wet_DataHead,0,sizeof(receive_Wet_DataHead));
			while ((receive_Wet_DataHead[0] != '<') || (receive_Wet_DataHead[1] != '<') || (receive_Wet_DataHead[2] != 'S') || (receive_Wet_DataHead[3] != 'T'))
			{
				if (recv(Socket_fd_wet, receive_Wet_DataHead, 4, 0) <= 0)
				{
					error_process();
					break;
				}
				if ((receive_Wet_DataHead[0] == '<') && (receive_Wet_DataHead[1] == '<') && (receive_Wet_DataHead[2] == 'S') && (receive_Wet_DataHead[3] == 'T'))
				{
					continue;
				} 
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == '#') && (receive_Wet_DataHead[2] == 'S') && (receive_Wet_DataHead[3] == 'E'))
				{
					ReadWetMemsDatetoUpper();
				}
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == '#') && (receive_Wet_DataHead[2] == 'S') && (receive_Wet_DataHead[3] == 'U'))
				{
					UpdateWetFlagtoUpper();
				}
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == '#') && (receive_Wet_DataHead[2] == 'I') && (receive_Wet_DataHead[3] == 'P'))
				{
					ReadWetIPDatetoUpper();
				}
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == '#') && (receive_Wet_DataHead[2] == 'H') && (receive_Wet_DataHead[3] == 'W'))
				{
					ReadWetMACDatetoUpper();
				}
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == 'I') && (receive_Wet_DataHead[2] == 'P') && (receive_Wet_DataHead[3] == 'W'))
				{
					ReadWetIPDatetoUpper1();
				}
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == 'H') && (receive_Wet_DataHead[2] == 'W') && (receive_Wet_DataHead[3] == 'W'))
				{
					ReadWetMACDatetoUpper1();
				}
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == 'F') && (receive_Wet_DataHead[2] == 'I') && (receive_Wet_DataHead[3] == 'P'))
				{
					WriteWetIPF();
				}
				if ((receive_Wet_DataHead[0] == '#') && (receive_Wet_DataHead[1] == 'F') && (receive_Wet_DataHead[2] == 'H') && (receive_Wet_DataHead[3] == 'W'))
				{
					WriteWetMACF();
				}
				if ((receive_Wet_DataHead[0] == '<') && (receive_Wet_DataHead[1] == '<') && (receive_Wet_DataHead[2] == 'S') && (receive_Wet_DataHead[3] == 'S'))
				{
					recvwetsonarstatus();
				}
        		wet_status = 0;
			}
			RevIQData_SetIQDataParaHead();//已接收到<<st
        		wet_status = 1;
			if(ptr_recv_upper_package->up_iq == 1)
			{
				sem_post(&sem_UPPER);
			} 
			if(Wet_to_Dry_flag == 1)
			{
				{
					//波束下放程序,将所有数据通过FPGA的SRIO发送给DSP
					if((ptr_recv_upper_package->DataType >> 5) == 1)
					{
						Beam_Down_Data_Manage();
					} 
				}
			}
		}
	}
}


void iq_to_upper()
{
	while(1)
	{
        //等待信号量
		sem_wait(&sem_UPPER);
		Send_IQdataToUpper();
	} 
} 

