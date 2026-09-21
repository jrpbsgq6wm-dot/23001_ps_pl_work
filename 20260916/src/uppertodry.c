/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : uppertodry.c
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-12-08 10:01:55
 ******************************************************************************/

#include <errno.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include "main.h"
#include "tcptrans_link.h"
#include "uppertodry.h"
#include "wettodry.h"
#include "drytoupper.h"
#include "upgrade_program.h"
#include "uppertodry.h"
#include "drytowet.h"
#include "armtofpga.h"
#include "armtodsp.h"
#include "fpga_init.h"
#include "eeprom.h"

RECV_UPPER_CONFIG_PARAMETERS  recv_upper_package = {0};//接收到上位机的参数配置包----结构体变量
RECV_UPPER_CONFIG_PARAMETERS* ptr_recv_upper_package = &recv_upper_package;//接收到上位机的参数配置包----结构体指针
RECV_UPPER_CONFIG_PARAMETERS  cmd_package = {0};//接收到上位机的参数配置包保存本地----结构体变量
WET_RECV_UPPER_CONFIG_PARAMETERS wet_recv_parameter = {0};//原湿端接收到的干端转发的参数
SY_IN_PACK  sync_in = {0};
SY_OUT_PACK  sync_out = {0};

int Sersor_Lenth;
int Sersor_Return[20000] = {0};//max : 0x4000
unsigned char Sersor_Config[50] = {0};

/*****************************************************************************
 * * description : 接收显控下发的传感器配置命令信息
 * * return       {*}
 * * Date        : 2022-10-09 14:12:04
 * * Other
 ******************************************************************************/
void Recv_SersorData(void)
{
    memset(Sersor_Config, 0, (sizeof(Sersor_Config) / sizeof(Sersor_Config[0]))); //清空接收BUF
    if (recv(Connect_fd_upper, (char*)&Sersor_Lenth, 4, 0)< 0)
    {
        error_process();
    }
    if (recv(Connect_fd_upper, Sersor_Config, Sersor_Lenth, 0) < 0)
    {

        error_process();
    }
}

/*****************************************************************************
 * * description : 显控下发串口配置命令,配置对应传感器设备
 * * param        {int} len_offset  下发长度偏移地址
 * * param        {int} cfg_offset  下发设备号偏移地址
 * * param        {int} uart_flag   串口号
 * * return       {*}
 * * Date        : 2022-10-09 14:07:49
 * * Other       : uio起始地址:0x40000000
 ******************************************************************************/
void ConfigUart_GetSersorReturn(int len_offset,int cfg_offset,int uart_flag)
{
    memset(uio_tvg.mem_ptr, 0, 0x4000); //剩余tvg置0 0x4000为UIO_TVG总大小
    memcpy((char*)uio_tvg.mem_ptr, Sersor_Config, Sersor_Lenth);
   
    ptr_fpga_register_data->fpga_registers_parameters.UART_Length1 = 0;
    ptr_fpga_register_data->fpga_registers_parameters.UART_Length2 = 0; 
    if(uart_flag == 1)//0-7:gga,8-15:hdt,16-23:mot,24-31:svp
    {
        ptr_fpga_register_data->fpga_registers_parameters.UART_Length1 = Sersor_Lenth << len_offset;
        ptr_fpga_register_data->fpga_registers_parameters.UART_Cfg = 1 << cfg_offset;//0:GGA,1:HDT,2:MOT,3:SVP,4:PPS,5:718
    }
    else if(uart_flag == 2)//0-7:pps,8-15:718,16-23:resv,24-31:resv
    {
        ptr_fpga_register_data->fpga_registers_parameters.UART_Length2 = Sersor_Lenth << len_offset;
        ptr_fpga_register_data->fpga_registers_parameters.UART_Cfg = 1 << cfg_offset;//0:GGA,1:HDT,2:MOT,3:SVP,4:PPS,5:718
    }
    else
    {
        ;//空语句
    }
    ptr_fpga_register_data->set_pr = 1;
    usleep(1000);
    ptr_fpga_register_data->set_pr = 0;
    if((Sersor_Config[0] == 'l' )&& (Sersor_Config[1] == 'o') && (Sersor_Config[2] == 'g'))  sleep(2);
    else  usleep(500000);
    ptr_fpga_register_data->fpga_registers_parameters.UART_Cfg = 0;
    ptr_fpga_register_data->set_pr = 1;
    usleep(1000);
    ptr_fpga_register_data->set_pr = 0;
}

/*****************************************************************************
 * * description : 发送传感器设备应答的串口信息
 * * param        {int} sersor_len  应答信息长度
 * * return       {*}
 * * Date        : 2022-10-09 14:07:31
 * * Other
 ******************************************************************************/
void Send_SersorData(int sersor_len)
{
    char Sersor_Head[4] = { '#','#','S','E' };

    memset(Sersor_Return, 0, sersor_len);
    memcpy(Sersor_Return, (char*)uio_tvg.mem_ptr, sersor_len);
    pthread_mutex_lock(&mut);
    if (send(Connect_fd_upper, Sersor_Head, SIZE_OF_LONG, 0) < 0) //4byte
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }   
    if (send(Connect_fd_upper, (char*)&sersor_len, SIZE_OF_LONG, 0) < 0) //4byte
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    if (send(Connect_fd_upper, (Sersor_Return + 3), sersor_len, 0) < 0)//+3 is bug
    {
        error_process();
        pthread_mutex_unlock(&mut);
    }
    pthread_mutex_unlock(&mut);
}

/*****************************************************************************
 * * description : 将显控下发的参数保存到本地
 * * param        {RECV_UPPER_CONFIG_PARAMETERS*} ptr_cmd_des
 * * param        {RECV_UPPER_CONFIG_PARAMETERS*} ptr_cmd_source
 * * return       {*}
 * * Date        : 2022-10-09 14:53:32
 * * Other       : @@sp
 ******************************************************************************/
void copy_recv_cmd(RECV_UPPER_CONFIG_PARAMETERS* ptr_cmd_des, const RECV_UPPER_CONFIG_PARAMETERS* ptr_cmd_source)
{
    memcpy((char*)(&ptr_cmd_des->DataType), (char*)(&(ptr_cmd_source->DataType)), SIZE_OF_CONFIG_PARA);
}

int IPCheck(char *CheckData)
{
	char *arg[4];
	int num[4];
	arg[0] = strtok(CheckData,".");
	arg[1] = strtok(NULL,".");
	arg[2] = strtok(NULL,".");
	arg[3] = strtok(NULL,".");
	strtok(NULL,"");
	int i;
	for(i=0;i< 4; ++i)
	{
		num[i] = atoi(arg[i]);
	}
	if((num[0] == 192) && (num[1] == 168) && (num[2] >= 0 && num[2] <= 255) && (num[3] >= 1 && num[3] <= 255))
	{
		return 1;
	}
	else
	{
		printf("num[0] = %d,num[1] = %d,num[2] = %d, num[3] = %d\n",num[0],num[1],num[2],num[3]);
		strtok(NULL,"");
		return -1;
	}
}
/*****************************************************************************
 * * description : 显控下发到干端的命令
 * * return       {*}
 * * Date        : 2022-10-09 14:54:40
 * * Other       : 独立线程
 ******************************************************************************/
void upper_to_dry(void)
{
    int ret = 0;
    char DataHead1[2];
    char DataHead2[2];
    char RevTcpDev[4],RevTcpMode[3],RevTcpData[16],RevTcpData1[18];
    int  len_recv_upperpara,len_recv_tail,len_recv_head4, len_recv_total;
    unsigned int sy_len;
    char UpdateType[4];
    char SCType[3];
    char SCStates[3];
    char Send_Wet_Head[2] = { '<','<' };
    char Send_IP_Head[4] = { '<','<','I','P'};
    char Send_HW_Head[4] = { '<','<','H','W'};
    char  DataTail[4];//数据尾标识“ED>>”
    char DEV_SYNC[3];
    unsigned short recv_crc_calculate, recv_crc;
    int len_recv_crc;
    char Send_WetHead_Buf[20];
    char Send_Updata_Data[20];

	while(1)
	{
		if ((server_netStatus == 1)&&(client_netStatus == 1))
		{
			DataHead1[0] = 0;
			DataHead1[1] = 0;
			while ((DataHead1[0] != '@') || (DataHead1[1] != '@'))
			{
			    ret = recv(Connect_fd_upper, DataHead1, 2, 0);
				if (ret <= 0)
				{
				    //DBG("ret = %d!!!\n", ret);
				    //Debug("DataHead1[0]=%d,DataHead1[1]=%d\n", DataHead1[0], DataHead1[1]);
					error_process();
					break;
				}
				//Debug("DataHead1[0]=%d,DataHead1[1]=%d\n", DataHead1[0], DataHead1[1]);
			}
			if(server_netStatus == 0)   continue;
			if(recv(Connect_fd_upper,DataHead2,2,0) < 0)
			{
				error_process();
				continue;
			}
			if ((DataHead2[0] == 'S') && (DataHead2[1] == 'R'))//清空声呐缓存，声呐恢复成初始状态
			{
				//DBG("@@SR\n");
				//memset(uio_share_mem_IQ.mem_ptr+SIZE_OF_DATA_FIRST , 0,  SIZE_OF_AD_DATA_MAX_BYTE ); 
			}
			if ((DataHead2[0] == 'R') && (DataHead2[1] == 'S'))//3.请求硬件工作状态信息指令(arm回复)@@SS
			{
				//DBG("@@RS\n");
				rs_flag = 0;
				int wait_sec = 0;
				int Last_Fpga_start_mod = 0;
				int Last_Wet_to_Dry_flag = 0;
				int Last_wsm_con = 0;
				int Last_Wet_wsm_con = 0;
				ptr_send_upper_status_information->wet_status = 0;
				ptr_send_upper_status_information->dsp_status = 0;
				ptr_send_upper_status_information->dry_fpga_status = 0;    
				ptr_send_upper_status_information->GGAZDA_status = 0; 
				ptr_send_upper_status_information->Heading_status = 0; 
				ptr_send_upper_status_information->TSS1_status = 0; 
				ptr_send_upper_status_information->SV_status = 0; 
				ptr_send_upper_status_information->PPS_status = 0;

				dry_fpga_status = 0;
				dsp_status = 0;
				GGAZDA_status = 0;
				Heading_status = 0;
				TSS1_status = 0;
				SV_status = 0;
				PPS_status = 0;
				rs_flag = 0;
				if(wet_status == 0)
				{		
					Last_Fpga_start_mod = Fpga_start_mod;
					Last_Wet_to_Dry_flag = Wet_to_Dry_flag;
					Last_wsm_con = ptr_fpga_register_data->wsm_con;
					Last_Wet_wsm_con = ptr_wet_fpga_register_data->wsm_con;

					ptr_fpga_register_data->wsm_con = 1;
					ptr_wet_fpga_register_data->wsm_con=1;
					Fpga_start_mod = 0;
					Wet_to_Dry_flag = 1;
					DataHead2[0] = 'B';
					DataHead2[1] = 'R';
					if( send(Socket_fd_wet, Send_Wet_Head, 2, 0) < 0)
					{
						error_process();
						continue;
					}
					if( send(Socket_fd_wet, DataHead2, 2, 0) < 0)
					{
						error_process();
						continue;
					}
	//				sleep(1);
					while(rs_flag == 0 && wait_sec < 3)
					{
						sleep(1);
						dry_fpga_status = (ptr_fpga_register_data->fpga_sta.status >> 1) & 1;
						rs_flag =wet_status*dsp_status*dry_fpga_status*GGAZDA_status* Heading_status*TSS1_status*SV_status*PPS_status;					
						wait_sec++;
					}
					DataHead2[0] = 'E';
					DataHead2[1] = 'R';
					if( send(Socket_fd_wet, Send_Wet_Head, 2, 0) < 0)
					{
						error_process();
						continue;
					}
					if( send(Socket_fd_wet, DataHead2, 2, 0) < 0)
					{
						error_process();
						continue;
					}
					// Debug("wet_status: %d\n", wet_status);
					// Debug("dsp_status: %d\n", dsp_status);
					// Debug("dry_fpga_status: %d\n", dry_fpga_status);
					// Debug("GGAZDA_status: %d\n", GGAZDA_status);
					// Debug("Heading_status: %d\n", Heading_status);
					// Debug("TSS1_status: %d\n", TSS1_status);
					// Debug("SV_status: %d\n", SV_status);
					// Debug("PPS_status: %d\n", PPS_status);
					// Debug("Fpga_start_mod: %d\n", Fpga_start_mod);
					pthread_mutex_lock(&mut);
					send_status_to_upper();
					pthread_mutex_unlock(&mut);
					// Debug_pritf_sendtoupper_hardwarepara(ptr_send_upper_status_information);
					rs_flag = 0;		
					wet_status = 0;
					Fpga_start_mod = Last_Fpga_start_mod;
					Wet_to_Dry_flag = Last_Wet_to_Dry_flag;
					ptr_fpga_register_data->wsm_con = Last_wsm_con;
					ptr_wet_fpga_register_data->wsm_con= Last_Wet_wsm_con;
					DataHead2[0] = 'R';
					DataHead2[1] = 'S';
				}else{
					while(rs_flag == 0 && wait_sec < 3)
					{
						sleep(1);
						dry_fpga_status = (ptr_fpga_register_data->fpga_sta.status >> 1) & 1;
						rs_flag =wet_status*dsp_status*dry_fpga_status*GGAZDA_status* Heading_status*TSS1_status*SV_status*PPS_status;						
						wait_sec++;
					}
					// Debug("wet_status: %d\n", wet_status);
					// Debug("dsp_status: %d\n", dsp_status);
					// Debug("dry_fpga_status: %d\n", dry_fpga_status);
					// Debug("GGAZDA_status: %d\n", GGAZDA_status);
					// Debug("Heading_status: %d\n", Heading_status);
					// Debug("TSS1_status: %d\n", TSS1_status);
					// Debug("SV_status: %d\n", SV_status);
					// Debug("PPS_status: %d\n", PPS_status);
					// Debug("Fpga_start_mod: %d\n", Fpga_start_mod);
					pthread_mutex_lock(&mut);
					send_status_to_upper();
					pthread_mutex_unlock(&mut);
					// Debug_pritf_sendtoupper_hardwarepara(ptr_send_upper_status_information);
					rs_flag = 0;		
	
					DataHead2[0] = 'R';
					DataHead2[1] = 'S';
				}

			}
			if ((DataHead2[0] == 'U') && (DataHead2[1] == 'P'))//更新BOOT.BIN和image.ub
			{
				if (recv(Connect_fd_upper, UpdateType, 4, 0) < 0)
				{
					error_process();
					continue;
				}
				if (UpdateType[0] == '_' && UpdateType[1] == 'D' && UpdateType[2] == 'R' && UpdateType[3] == 'Y')
				{
					DBG("@@UP_DRY\n");
					upgrade_program_dry();  
					memset(Send_Updata_Data,0,sizeof(Send_Updata_Data));
					sprintf(Send_Updata_Data, "%.2s%.2s%.4s",Send_Wet_Head,DataHead2,UpdateType);//<<UP_DRY
					if (send(Socket_fd_wet, Send_Updata_Data, 8, 0) < 0)
					{
						error_process();
						continue;
					}
				}
				if (UpdateType[0] == '_' && UpdateType[1] == 'W' && UpdateType[2] == 'E' && UpdateType[3] == 'T')
				{

					DBG("@@UP_WET\n");
					memset(Send_Updata_Data,0,sizeof(Send_Updata_Data));
					sprintf(Send_Updata_Data, "%.2s%.2s%.4s",Send_Wet_Head,DataHead2,UpdateType);//<<UP_WET
					if (send(Socket_fd_wet, Send_Updata_Data, 8, 0) < 0)
					{
						error_process();
						continue;
					} 
				} 
				if (UpdateType[0] == '_' && UpdateType[1] == 'M' && UpdateType[2] == 'E' && UpdateType[3] == 'S')//mems.bin
				{
					DBG("@@UP_MES\n");
					memset(Send_Updata_Data,0,sizeof(Send_Updata_Data));
					sprintf(Send_Updata_Data, "%.2s%.2s%.4s",Send_Wet_Head,DataHead2,UpdateType);//<<UP_MES
					if (send(Socket_fd_wet, Send_Updata_Data, 8, 0) < 0)
					{
						error_process();
						continue;
					}
				}
				if (UpdateType[0] == '_' && UpdateType[1] == 'D' && UpdateType[2] == 'S' && UpdateType[3] == 'P')
				{

					DBG("@@UP_DSP\n");
					/*send(Socket_fd_dsp,Send_DSP_Head,2,0);
					  send(Socket_fd_dsp,DataHead2,2,0); 
					  send(Socket_fd_dsp,UpdateType,4,0); 
					  */
				}
			}
			if ((DataHead2[0] == 'B') && (DataHead2[1] == 'R'))//开始请求数据 (回复)
			{
				//DBG("@@BR\n");

				ptr_fpga_register_data->wsm_con = 1;
				ptr_wet_fpga_register_data->wsm_con=1;
				Fpga_start_mod = 1;
				Wet_to_Dry_flag = 1;
				if( send(Socket_fd_wet, Send_Wet_Head, 2, 0) < 0)
				{
				    DBG("send Socket_fd_wet err01!!!\n");
					error_process();
					continue;
				}
				if( send(Socket_fd_wet, DataHead2, 2, 0) < 0)
				{
				    DBG("send Socket_fd_wet err23!!!\n");
					error_process();
					continue;
				}
			}
			if ((DataHead2[0] == 'E') && (DataHead2[1] == 'R'))//停止请求波形数据
			{
				//DBG("@@ER\n");
				ptr_fpga_register_data->wsm_con = 0;
				ptr_wet_fpga_register_data->wsm_con=0;
				Flag_ExtSenserBuf = 0;
				Fpga_start_mod = 0;
				Wet_to_Dry_flag = 0;
				rs_flag = 0;
        		wet_status = 0;
				if(send(Socket_fd_wet, Send_Wet_Head, 2, 0) < 0)
				{
					DBG("<<error\n");
					error_process();
					continue;
				}
				if(send(Socket_fd_wet, DataHead2, 2, 0) < 0)
				{
					DBG("ER error\n");
					error_process();
					continue;
				}
			}
			if ((DataHead2[0] == 'S') && (DataHead2[1] == 'C'))//串口配置命令
			{
				//DBG("@@SC\n");
				if (recv(Connect_fd_upper, SCType, 3, 0) < 0)
				{
					error_process();
					continue;
				}
				if ((SCType[0] == '_') && (SCType[1] == 'S') && (SCType[2] == 'V'))//SVP传感器配置命令
				{

					//DBG("@@SC_SV\n");
					if (recv(Connect_fd_upper, SCStates, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'B') && (SCStates[2] == 'G'))
					{
						//DBG("@@SC_SV_BG\n");
						Recv_SersorData();
						ConfigUart_GetSersorReturn(svp_len_offset,svp,UART_CFG_LEN_FLAG1);
						Send_SersorData(SIZE_OF_UART_SENSOR_RETURN);
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'E') && (SCStates[2] == 'D'))
					{
						//DBG("@@SC_SV_ED\n");
					}
				}
				if ((SCType[0] == '_') && (SCType[1] == 'G') && (SCType[2] == 'N'))//GNSS传感器配置命令
				{
					if (recv(Connect_fd_upper, SCStates, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'B') && (SCStates[2] == 'G'))
					{

						DBG("@@SC_GN_BG\n");
						Recv_SersorData();
						ConfigUart_GetSersorReturn(gga_len_offset, gga,UART_CFG_LEN_FLAG1);
						Send_SersorData(SIZE_OF_UART_SENSOR_RETURN);
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'E') && (SCStates[2] == 'D'))
					{
						DBG("@@SC_GN_ED\n");
					}
				}
				if ((SCType[0] == '_') && (SCType[1] == 'H') && (SCType[2] == 'D'))//HEADING传感器配置命令
				{
					if (recv(Connect_fd_upper, SCStates, 3, 0)< 0)
					{
						error_process();
						continue;
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'B') && (SCStates[2] == 'G'))
					{
						DBG("@@SC_HD_BG\n");
						Recv_SersorData();
						ConfigUart_GetSersorReturn(hdt_len_offset, heading,UART_CFG_LEN_FLAG1);
						Send_SersorData(SIZE_OF_UART_SENSOR_RETURN);
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'E') && (SCStates[2] == 'D'))
					{
						DBG("@@SC_HD_ED\n");
					}
				}
				if ((SCType[0] == '_') && (SCType[1] == 'M') && (SCType[2] == 'T'))//MOTION传感器配置命令
				{
					if (recv(Connect_fd_upper, SCStates, 3, 0)< 0)
					{
						error_process();
						continue;
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'B') && (SCStates[2] == 'G'))
					{
						DBG("@@SC_MT_BG\n");
						Recv_SersorData();
						ConfigUart_GetSersorReturn(mot_len_offset, motion, UART_CFG_LEN_FLAG1);
						Send_SersorData(SIZE_OF_UART_SENSOR_RETURN);
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'E') && (SCStates[2] == 'D'))
					{
						DBG("@@SC_MT_ED\n");
					}
				}
				if ((SCType[0] == '_') && (SCType[1] == 'P') && (SCType[2] == 'P'))//PPS传感器配置命令
				{
					if (recv(Connect_fd_upper, SCStates, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'B') && (SCStates[2] == 'G'))
					{
						DBG("@@SC_PP_BG\n");
						Recv_SersorData();
						ConfigUart_GetSersorReturn(pps_len_offset, pps,  UART_CFG_LEN_FLAG2);
						Send_SersorData(SIZE_OF_UART_SENSOR_RETURN);
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'E') && (SCStates[2] == 'D'))
					{
						DBG("@@SC_PP_ED\n");
					}
				}
				if ((SCType[0] == '_') && (SCType[1] == 'O') && (SCType[2] == 'E'))	//OEM718D板卡配置命令
				{
					if (recv(Connect_fd_upper, SCStates, 3, 0)< 0)
					{
						error_process();
						continue;
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'B') && (SCStates[2] == 'G'))
					{
						DBG("@@SC_OE_BG\n");
						Recv_SersorData();
						ConfigUart_GetSersorReturn(oem_len_offset, oem, UART_CFG_LEN_FLAG2);
						Send_SersorData(SIZE_OF_UART_OEM_RETURN);
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'E') && (SCStates[2] == 'D'))
					{
						DBG("@@SC_OE_ED\n");
					}
				}
				if ((SCType[0] == '_') && (SCType[1] == 'M') && (SCType[2] == 'E'))//MEMS配置命令
				{
					if ( recv(Connect_fd_upper, SCStates, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'B') && (SCStates[2] == 'G'))
					{
						DBG("@@SC_ME_BG\n");
						Recv_SersorData();
						memset(Send_WetHead_Buf,0,sizeof(Send_WetHead_Buf));
						sprintf(Send_WetHead_Buf, "%.2s%.2s%.3s%.3s",Send_Wet_Head,DataHead2,SCType,SCStates);//<<SC_ME_BG
						if (send(Socket_fd_wet, Send_WetHead_Buf, 10, 0) < 0)
						{
							error_process();
							continue;
						}
						if (send(Socket_fd_wet, (char*)&Sersor_Lenth, 4, 0) < 0)
						{
							error_process();
							continue;
						}
						if (send(Socket_fd_wet, Sersor_Config, Sersor_Lenth, 0) < 0)
						{
							error_process();
							continue;
						}
					}
					if ((SCStates[0] == '_') && (SCStates[1] == 'E') && (SCStates[2] == 'D'))
					{
						DBG("@@SC_ME_ED\n");
						memset(Send_WetHead_Buf,0,sizeof(Send_WetHead_Buf));
						sprintf(Send_WetHead_Buf, "%.2s%.2s%.3s%.3s",Send_Wet_Head,DataHead2,SCType,SCStates);//<<SC_ME_ED
						if (send(Socket_fd_wet, Send_WetHead_Buf, 10, 0) < 0)
						{
							error_process();
							continue;
						}
					}
				}
			}

			if ((DataHead2[0] == 'S') && (DataHead2[1] == 'P'))//设置声呐的各项参数 (arm接收)
			{
				//DBG("@@SP\n");
				len_recv_upperpara = recv_socket(Connect_fd_upper, (char*)(ptr_recv_upper_package), SIZE_OF_CONFIG_PARA);//2086
				if (len_recv_upperpara < 0)
				{
					error_process();
					continue;
				}
				len_recv_crc = recv_socket(Connect_fd_upper, (char*)&recv_crc, SIZE_OF_LONG);
				if (len_recv_crc < 0)
				{
					error_process();
					continue;
				}
				len_recv_tail = recv_socket(Connect_fd_upper, DataTail, SIZE_OF_LONG);
				if (len_recv_tail < 0)
				{
					error_process();
					continue;
				}
				recv_crc_calculate = crc16((char*)(ptr_recv_upper_package), sizeof(recv_upper_package), 0xFFFF);
				if (recv_crc_calculate != recv_crc)
				{
					printf("recv_crc_calculate:%x \n", recv_crc_calculate);
					printf("recv_crc:%x \n", recv_crc);
					printf("crc error! \n");
					continue;
				}

				copy_recv_cmd(&cmd_package, ptr_recv_upper_package);//显控参数拷贝到本地
				// Debug_pritf_receive_para(&cmd_package);//打印
				// Debug("DataTail:1%c 2%c 3%c 4%c\n", DataTail[0], DataTail[1], DataTail[2], DataTail[3]);//ED@@  

				DryTOWet_Flag = 1;//将解析完成的参数下发至湿端ARM

				pthread_mutex_lock(&mut2); 
				parsing_upperpara_to_fpga(&cmd_package, ptr_fpga_register_data); //解析收到的显控参数信息并配置给干端FPGA
				//湿端发射部分上移的参数配置
				
				
				parsing_upperpara_to_wet(&cmd_package, (SEND_WET_PARAMETER*)&wet_recv_parameter); //将发送到干端的显控配置命令转换为下发到湿端的配置指令
				parsing_instructions_200k(&wet_recv_parameter, &(wet_fpga_register_data.fpga_registers_200k));//从下发到湿端的配置指令提取出原本写到湿端PL端的数组
                                //Debug_pritf_convert_fpga_parameter(&(wet_fpga_register_data.fpga_registers_200k));
				//DBG("ptr_wet_fpga_register_data is %x\n", ptr_wet_fpga_register_data);//打印PL端寄存器地址，调试用
                		memcpy(&(ptr_wet_fpga_register_data->fpga_registers_200k), &(wet_fpga_register_data.fpga_registers_200k), sizeof(WET_FPGA_CONFIG_PARAMETERS)); //将提取的参数发送到PL端
				config_sensor_baud_and_framehead(&wet_recv_parameter, &wet_fpga_register_data);
				memcpy(&(ptr_wet_fpga_register_data->fpga_sensor), &(wet_fpga_register_data.fpga_sensor), sizeof(WET_FPGA_CONFIG_SENSER_PARAMETERS)); 

				pthread_mutex_unlock(&mut2);
				// DBG("********************************WET UP TEST************************\n");
				// Debug_pritf_convert_wet_fpga_parameter(&(ptr_wet_fpga_register_data->fpga_registers_200k));//打印
				// Debug_pritf_sensor_baud_and_framehead(&(ptr_wet_fpga_register_data->fpga_sensor)); //打印
				//ptr_wet_fpga_register_data->fpga_sensor = wet_fpga_register_data.fpga_sensor; //下发传感器参数配置命令
				ptr_wet_fpga_register_data->set_pr=1;
				usleep(1000);//1ms 
				ptr_wet_fpga_register_data->set_pr=0;
				// Debug_pritf_convert_fpga_parameter(ptr_fpga_register_data);//打印

				ptr_fpga_register_data->set_pr = 1;//参数更新中断
				usleep(1000);//1ms 
				ptr_fpga_register_data->set_pr = 0;
			}
			if ((DataHead2[0] == 'I') && (DataHead2[1] == 'P'))//7.get ipaddr
			{
				DBG("@@IP\n");
				if (recv(Connect_fd_upper, RevTcpDev, 4, 0) < 0)
				{
					error_process();
					continue;
				}
				if ((RevTcpDev[0] == '_') && (RevTcpDev[1] == 'D') && (RevTcpDev[2] == 'R') && (RevTcpDev[3] == 'Y'))
				{
					DBG("_DRY\n");
					if (recv(Connect_fd_upper, RevTcpMode, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'R') && (RevTcpMode[2] == 'D'))
					{
						DBG("_RD\n");
						ReadDry();
						ifcfgIP_DRY.flag = 'R';
						ifcfgIP_DRY.size = 48; 
						strncpy(ifcfgIP_DRY.head,Send_IP_Head,4);//ip_dry_head
						if (send(Connect_fd_upper, (char *)&ifcfgIP_DRY,sizeof(ifcfgIP_DRY), 0) < 0)   //发送
						{
							error_process();
							continue;
						}
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'W') && (RevTcpMode[2] == 'R'))
					{
						DBG("_WR\n");
						bzero(RevTcpData,sizeof(RevTcpData));
						if ( recv(Connect_fd_upper, RevTcpData,sizeof(RevTcpData),0) < 0)
						{
							error_process();
							continue;
						}
						char CheckData[16];
						strcpy(CheckData,RevTcpData);
						ret = IPCheck(CheckData);
						if(ret == -1)
						{
							continue;
						}
						printf("set dry ipaddr = %s\n",RevTcpData);
						int ret0 = WriteDryIP(RevTcpData);
						char WriteTUpper[8] = {'<','<','I','P','W'};
						if(-1 == ret0)
						{
							if (send(Connect_fd_upper, WriteTUpper,8, 0) < 0)   //发送
							{
								error_process();
								continue;
							}
							if (send(Connect_fd_upper, "F",1, 0) < 0)   //发送
							{
								error_process();
								continue;
							}
						}
						else
						{
							if (send(Connect_fd_upper, WriteTUpper,8, 0) < 0)   //发送
							{
								error_process();
								continue;
							}
							if (send(Connect_fd_upper, "T",1, 0) < 0)   //发送
							{
								error_process();
								continue;
							}

						}
					}
				}
				if ((RevTcpDev[0] == '_') && (RevTcpDev[1] == 'W') && (RevTcpDev[2] == 'E') && (RevTcpDev[3] == 'T'))
				{
					DBG("_WET\n");
					if (recv(Connect_fd_upper, RevTcpMode, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'R') && (RevTcpMode[2] == 'D'))
					{
						DBG("@@IP_WET_RD\n");
						memset(Send_Updata_Data,0,sizeof(Send_Updata_Data));
						sprintf(Send_Updata_Data, "%.2s%.2s%.4s%.3s",Send_Wet_Head,DataHead2,RevTcpDev,RevTcpMode);//<<IP_WET_RD
						if(send(Socket_fd_wet, Send_Updata_Data, 11, 0) < 0)
						{
							error_process();
							continue;
						}
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'W') && (RevTcpMode[2] == 'R'))
					{
						DBG("@@IP_WET_WR\n");
						bzero(RevTcpData,sizeof(RevTcpData));
						if (recv(Connect_fd_upper, RevTcpData,sizeof(RevTcpData) ,0) < 0)
						{
							error_process();
							continue;
						}
						char CheckData[16];
						strcpy(CheckData,RevTcpData);
						ret = IPCheck(CheckData);
						if(ret == -1)
						{
							continue;
						}
						printf("set wet ipaddr = %s\n",RevTcpData);
						memset(Send_Updata_Data,0,sizeof(Send_Updata_Data));
						sprintf(Send_Updata_Data, "%.2s%.2s%.4s%.3s",Send_Wet_Head,DataHead2,RevTcpDev,RevTcpMode);//<<IP_WET_WR
						if (send(Socket_fd_wet, Send_Updata_Data, 11, 0) < 0)
						{
							error_process();
							continue;
						}
						if ((send(Socket_fd_wet, RevTcpData, sizeof(RevTcpData), 0)) < 0)//wet ip addr
						{
							error_process();
							continue;
						}
					}
				}
			}
			if ((DataHead2[0] == 'H') && (DataHead2[1] == 'W'))//7.get hw
			{
				DBG("@@HW\n");
				if (recv(Connect_fd_upper, RevTcpDev, 4, 0) < 0)
				{
					error_process();
					continue;
				}
				if ((RevTcpDev[0] == '_') && (RevTcpDev[1] == 'D') && (RevTcpDev[2] == 'R') && (RevTcpDev[3] == 'Y'))
				{
					DBG("_DRY\n");
					if (recv(Connect_fd_upper, RevTcpMode, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'R') && (RevTcpMode[2] == 'D'))
					{
						DBG("_RD\n");
						ReadDry();
						ifcfgIP_DRY.flag = 'R';
						ifcfgIP_DRY.size = 48; 
						strncpy(ifcfgIP_DRY.head,Send_HW_Head,4);//hw_dry_head
						if (send(Connect_fd_upper, (char *)&ifcfgIP_DRY,sizeof(ifcfgIP_DRY), 0) < 0)   //发送
						{
							error_process();
							continue;
						}
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'W') && (RevTcpMode[2] == 'R'))
					{
						DBG("_WR\n");
						bzero(RevTcpData1,sizeof(RevTcpData1));
						if ( recv(Connect_fd_upper, RevTcpData1,sizeof(RevTcpData1),0) <= 0)
						{
							error_process();
							continue;
						}

						printf("set dry mac = %s\n",RevTcpData1);
						char WriteTUpper[8] = {'<','<','H','W','W'};
						int ret0 = WriteDryMAC(RevTcpData1);
						if(-1 == ret0)
						{
							if (send(Connect_fd_upper, WriteTUpper,8, 0) < 0)   //发送
							{
								error_process();
								continue;
							}
							if (send(Connect_fd_upper, "F",1, 0) < 0)   //发送失败F
							{
								error_process();
								continue;
							}
						}
						else
						{
							if (send(Connect_fd_upper, WriteTUpper,8, 0) < 0)   //发送
							{
								error_process();
								continue;
							}
							if (send(Connect_fd_upper, "T",1, 0) < 0)   //发送
							{
								error_process();
								continue;
							}
						}
					}
				}
				if ((RevTcpDev[0] == '_') && (RevTcpDev[1] == 'W') && (RevTcpDev[2] == 'E') && (RevTcpDev[3] == 'T'))
				{
					DBG("_WET\n");
					if (recv(Connect_fd_upper, RevTcpMode, 3, 0) < 0)
					{
						error_process();
						continue;
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'R') && (RevTcpMode[2] == 'D'))
					{
						DBG("@@HW_WET_RD\n");
						memset(Send_Updata_Data,0,sizeof(Send_Updata_Data));
						sprintf(Send_Updata_Data, "%.2s%.2s%.4s%.3s",Send_Wet_Head,DataHead2,RevTcpDev,RevTcpMode);//<<IP_WET_RD
						if(send(Socket_fd_wet, Send_Updata_Data, 11, 0) < 0)
						{
							error_process();
							continue;
						}
					}
					if ((RevTcpMode[0] == '_') && (RevTcpMode[1] == 'W') && (RevTcpMode[2] == 'R'))
					{
						DBG("@@HW_WET_WR\n");
						bzero(RevTcpData1,sizeof(RevTcpData1));
						if (recv(Connect_fd_upper, RevTcpData1,sizeof(RevTcpData1) ,0) <= 0)
						{
							error_process();
							continue;
						}
						printf("set wet mac = %s\n",RevTcpData1);
						memset(Send_Updata_Data,0,sizeof(Send_Updata_Data));
						sprintf(Send_Updata_Data, "%.2s%.2s%.4s%.3s",Send_Wet_Head,DataHead2,RevTcpDev,RevTcpMode);//<<IP_WET
						if (send(Socket_fd_wet, Send_Updata_Data, 11, 0) < 0)
						{
							error_process();
							continue;
						}
						if ((send(Socket_fd_wet, RevTcpData1, sizeof(RevTcpData1), 0)) < 0)//wet ip addr
						{
							error_process();
							continue;
						}
					}
				}
			}
			
		}
		 else {
			DBG("WAIT PC Link... ...\r\n");
             int  addr_len = 0;
            addr_len = sizeof(struct sockaddr_in);
			// 1. 
			Connect_fd_upper = accept(Socket_fd_upper, (struct sockaddr *)&Servaddr_upper, &addr_len);
			if (Connect_fd_upper < 0) {
				// 只打印真错误（EAGAIN/EWOULDBLOCK是正常无连接）
				if (errno != EAGAIN && errno != EWOULDBLOCK) {
					printf("Connect_fd_upper error: %s(errno: %d)\n", strerror(errno), errno);
				}
				usleep(10000); // 休眠10ms，避免空循环占满CPU
				continue;
			}else{
				get_pc_ip(Connect_fd_upper);
				pc_link_flag = 1;
				DBG("PC Link DRY Successfull:Socket_fd_upper%d  Connect_fd_upper%d \r\n",Socket_fd_upper,Connect_fd_upper);
			}
			DBG("WAIT PC Link... ...\r\n");
			// 2. 如果Dry还没连Wet，主动连Wet
			if (client_netStatus == 0) {
				// 先关闭旧socket，避免泄漏
				if (Socket_fd_wet >= 0) {
					close(Socket_fd_wet);
					Socket_fd_wet = -1;
				}
				// 创建socket（失败重试，不退出）
				Socket_fd_wet = socket(AF_INET, SOCK_STREAM, 0);
				if (Socket_fd_wet == -1) {
					printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
					usleep(1000000); // 休眠1秒重试
					continue;
				}
				// 初始化Wet地址
				bzero(&Servaddr_wet, sizeof(Servaddr_wet));
				Servaddr_wet.sin_family = AF_INET;
				Servaddr_wet.sin_addr.s_addr = inet_addr("192.168.0.5");
				Servaddr_wet.sin_port = htons(DEFAULT_PORT_WET);	//8000
				// 连接Wet（失败关闭socket，不泄漏）
				if (connect(Socket_fd_wet, (struct sockaddr *)&Servaddr_wet, sizeof(Servaddr_wet)) < 0) {
					printf("Socket_fd_wet error: %s(errno: %d)\n", strerror(errno), errno);
					close(Socket_fd_wet);
					Socket_fd_wet = -1;
					usleep(1000000); // 休眠1秒重试
					continue;
				}else{
					DBG("DRY Link WET Successfully: Socket_fd_wet %d\r\n",Socket_fd_wet);
				}
				client_netStatus = 1;
			}
			// 3. 标记Dry和PC的连接状态
			server_netStatus = 1;
		}
	}
}


/*****************************************************************************
 * * description : 打印显控下发的参数信息
 * * param        {RECV_UPPER_CONFIG_PARAMETERS*} receive_upper_package
 * * return       {*}
 * * Date        : 2022-10-09 14:55:55
 * * Other
 ******************************************************************************/
void Debug_pritf_receive_para(RECV_UPPER_CONFIG_PARAMETERS* receive_upper_package)
{
	Debug("********************* upper para to dry start****************************\n");
	Debug("cmd_package.DataType: %d\n", receive_upper_package->DataType);
	Debug("cmd_package.WorkMode: %d\n", receive_upper_package->WorkMode);
	Debug("cmd_package.LFMMode: %d\n", receive_upper_package->LFMMode);
	Debug("cmd_package.PWMFreq: %d\n", receive_upper_package->PWMFreq);
	Debug("cmd_package.PWMBandWidth: %d\n", receive_upper_package->PWMBandWidth);
	Debug("cmd_package.SamplingRate: %d\n", receive_upper_package->SamplingRate);
	Debug("cmd_package.Range: %d\n", receive_upper_package->Range);
	Debug("cmd_package.ManualGain: %d\n", receive_upper_package->ManualGain);
	Debug("cmd_package.AbsorbGainCoef: %d\n", receive_upper_package->AbsorbGainCoef);
	Debug("cmd_package.SpreadGainCoef: %d\n", receive_upper_package->SpreadGainCoef);
	Debug("cmd_package.PulseWidth: %f\n", receive_upper_package->PulseWidth);
	Debug("cmd_package.PingRate: %f\n", receive_upper_package->PingRate);
	Debug("cmd_package.GNSS_protocol: %d\n", receive_upper_package->GNSS_protocol);
	Debug("cmd_package.SVP_protocol: %d\n", receive_upper_package->SVP_protocol);
	Debug("cmd_package.HEADINGH_protocol: %d\n", receive_upper_package->HEADING_protocol);
	Debug("cmd_package.MOTION_protocol: %d\n", receive_upper_package->MOTION_protocol);
	Debug("cmd_package.PWM_Start: %d\n", receive_upper_package->PWM_Start);
	Debug("cmd_package.INS_mod: %d\n", receive_upper_package->INS_mod);
	Debug("cmd_package.Base_Test: %d\n", receive_upper_package->Base_Test);
	Debug("cmd_package.Sonar_image: %d\n", receive_upper_package->Sonar_image);
	Debug("cmd_package.Side_scan: %d\n", receive_upper_package->Side_scan);
	Debug("cmd_package.TD_mode: %d\n", receive_upper_package->TD_mode);
	Debug("cmd_package.Water_Column: %d\n", receive_upper_package->Water_Column);
	Debug("cmd_package.Image_show_mode: %d\n", receive_upper_package->Image_show_mode);
	Debug("cmd_package.Sidelobe_Factor: %f\n", receive_upper_package->Sidelobe_Factor);
	Debug("cmd_package.Threshold_control: %d\n", receive_upper_package->Threshold_control);
	Debug("cmd_package.Treshold_upper: %f\n", receive_upper_package->Treshold_upper);
	Debug("cmd_package.Treshold_lower: %f\n", receive_upper_package->Treshold_lower);
	Debug("cmd_package.Treshold_angle: %f\n", receive_upper_package->Treshold_angle);
	Debug("cmd_package.Image_width: %d\n", receive_upper_package->Image_width);
	Debug("cmd_package.Image_heith: %d\n", receive_upper_package->Image_heith);
	Debug("cmd_package.Focus: %d\n", receive_upper_package->Focus);
	Debug("cmd_package.interpolation: %d\n", receive_upper_package->interpolation);
	Debug("cmd_package.Water_Detection: %d\n", receive_upper_package->Water_Detection);
	Debug("cmd_package.Side_ratio: %d\n", receive_upper_package->Side_ratio);
	Debug("cmd_package.Image_ratio: %d\n", receive_upper_package->Image_ratio);
	Debug("cmd_package.Beam_Num: %d\n", receive_upper_package->Beam_Num);
	Debug("cmd_package.Beam_Type: %d\n", receive_upper_package->Beam_Type);
	//Debug("cmd_package.Open_angle: %f\n", receive_upper_package->Open_angle);
	Debug("cmd_package.Roll_stability: %d\n", receive_upper_package->Roll_stability);
	Debug("cmd_package.Pitch_stability: %d\n", receive_upper_package->Pitch_stability);
	Debug("cmd_package.Manual_SoundSpeed: %f\n", receive_upper_package->Manual_SoundSpeed);
	Debug("cmd_package.AD_NUM: %d\n", receive_upper_package->AD_NUM);
	Debug("cmd_package.roll_com: %d\n", receive_upper_package->roll_com);
	Debug("cmd_package.pitch_com: %d\n", receive_upper_package->pitch_com);
	Debug("cmd_package.Beamform: %d\n", receive_upper_package->Beamform);
	Debug("cmd_package.quality_filter: %f\n", receive_upper_package->quality_filter);
	Debug("cmd_package.Median_Filter: %d\n", receive_upper_package->Median_Filter);
	Debug("cmd_package.Bow_control: %d\n", receive_upper_package->Bow_control);
	Debug("cmd_package.Heave_control: %d\n", receive_upper_package->Heave_control);
	Debug("cmd_package.Start_Angle: %f\n", receive_upper_package->Start_Angle);
	Debug("cmd_package.Finish_Angle: %f\n", receive_upper_package->Finish_Angle);
	Debug("cmd_package.CH_beam: %d\n", receive_upper_package->CH_beam);
	Debug("cmd_package.CH_test: %d\n", receive_upper_package->CH_test);
	//Debug("cmd_package.CH_error: %d\n", receive_upper_package->CH_error);
	Debug("cmd_package.device_type: %d\n", receive_upper_package->device_type);
	Debug("cmd_package.Blind_area: %f\n", receive_upper_package->Blind_area);
	Debug("cmd_package.ins_angle: %f\n", receive_upper_package->ins_angle);
	Debug("cmd_package.device_mode: %d\n", receive_upper_package->device_mode);
	Debug("cmd_package.GPSBaud: %d\n", receive_upper_package->GPSBaud);
	Debug("cmd_package.HEDBaud: %d\n", receive_upper_package->HEDBaud);
	Debug("cmd_package.POSBaud: %d\n", receive_upper_package->POSBaud);
	Debug("cmd_package.SVPBaud: %d\n", receive_upper_package->SVPBaud);
	Debug("cmd_package.power: %f\n", receive_upper_package->power);
	Debug("cmd_package.TransGear: %d\n", receive_upper_package->TransGear);
	Debug("cmd_package.up_iq: %d\n", receive_upper_package->up_iq);
	Debug("cmd_package.math_mode: %d\n", receive_upper_package->math_mode);
	Debug("cmd_package.iq_num: %d\n", receive_upper_package->iq_num);
	Debug("cmd_package.side_width: %d\n", receive_upper_package->side_width);
	Debug("cmd_package.Sgram_num: %d\n", receive_upper_package->Sgram_num);
	Debug("cmd_package.mid_angle: %d\n", receive_upper_package->mid_angle);
	Debug("cmd_package.detection_mode: %d\n", receive_upper_package->detection_mode);
	Debug("cmd_package.kernel_num: %d\n", receive_upper_package->kernel_num);
	Debug("cmd_package.mark_baud: %d\n", receive_upper_package->mark_baud);
	Debug("********************* upper para to dry end****************************\n");
}
