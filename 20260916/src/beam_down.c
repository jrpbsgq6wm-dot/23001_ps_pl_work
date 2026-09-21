/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : beam_down.c
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-10-13 17:05:04
 * * 
 * * LastEditTime : 2022-11-17 11:06:09
 ******************************************************************************/
#include <stdlib.h>
#include <math.h> /* S231动态横摇需要判断浮点数有效性，并转换为Q3.29格式。 */
#include <sys/time.h>
#include <unistd.h> /* S231流程里需要usleep()和write()，显式包含避免交叉编译时出现隐式声明。 */
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
#include "armtofpga.h"
#include "beam_down.h"

struct timeval tv2 = {0};
struct timeval tv3 = {0};
int BeamDataSize = 0, BeamDataTotalSize = 0;
int All_Rapidio_SendToDSP_Size = 0, Rapidio1_Send_Size = 0, Rapidio2_Send_Size = 0;
float Overall_Angle = 0.0f, Beam_Angle = 0.0f;
char Ins_Mod_State = 0;
char Svp_Data[10] = {0};

/* S231 PL规定：一个IQ快照点固定占用832字节；IQ_Data_Length表示IQ声学数据长度，用它换算IQ点数。 */
#define S231_IQ_BYTES_PER_POINT 832U /* 每个IQ点对应832字节，PL按这个固定步长解析IQA/IQB输入。 */
#define S231_FRAME_MIN_POINTS 10U /* CBF至少需要10个IQ点才能形成有效输出，少于该值没有有效波束结果。 */
#define S231_FRAME_MAX_POINTS 31992U /* 参考程序给出的最大IQ点数上限，防止异常长度把DDR窗口写爆。 */
#define S231_FRAME_HEADER_OFFSET 12U /* frame+0..11由PL生成，frame+12开始是PS需要准备的参数区。 */
#define S231_PS_HEADER_BYTES 3700U /* PS写入区总长度：260+240+1600+1600字节。 */
#define S231_PARAM_BLOCK_BYTES 260U /* frame+12..271为参数区，含AAAA、57个参数word、EEEE标记和保留word。 */
#define S231_CH_ERROR_BYTES 240U /* frame+272..511为通道错误表，长度固定240字节。 */
#define S231_ROLL_BYTES 1600U /* frame+512..2111和frame+2112..3711各保存400个float。 */
#define S231_FRAME_PARAM_OFFSET 12U /* 参数区绝对偏移，对应FRAME_BASE+12。 */
#define S231_FRAME_CH_ERROR_OFFSET 272U /* CH_error绝对偏移，对应FRAME_BASE+272。 */
#define S231_FRAME_ROLL_TIME_OFFSET 512U /* Roll_Time_Diff绝对偏移，对应FRAME_BASE+512。 */
#define S231_FRAME_ROLL_SEQ_OFFSET 2112U /* Roll_Sequential_Value绝对偏移，对应FRAME_BASE+2112。 */
#define S231_FRAME_BEAM_SIZE_OFFSET 3712U /* frame+3712由PL写入BeamDataSize，PS不能改写。 */
#define S231_FRAME_PREFIX_BYTES (S231_FRAME_HEADER_OFFSET + S231_PS_HEADER_BYTES) /* PS写入区为frame+12..+3711，结束偏移为3712。 */
#define S231_FRAME_BEAM_SIZE_BYTES 4U /* frame+3712..+3715由PL写入BeamDataSize。 */
#define S231_FRAME_PL_TAIL_BYTES 14U /* 波束记录后由PL生成IQ子包长度、子包头和$$等尾部字段。 */
#define S231_BEAM_RECORD_BYTES 24U /* 每条波束结果记录24字节，来自PL参考实现。 */
#define S231_BEAM_RECORDS_PER_SNAPSHOT 256U /* 每个有效快照产生256条波束记录。 */
#define S231_FIXED_VALID_BYTES (S231_FRAME_PREFIX_BYTES + S231_FRAME_BEAM_SIZE_BYTES + S231_FRAME_PL_TAIL_BYTES) /* 有效帧固定开销为3712+4+14=3730字节。 */
#define S231_DMA_ALIGN_BYTES 8U /* AXI DMA SG的每个BD传输长度按8字节对齐。 */
#define S231_NREAD_ALIGN_BYTES 256U /* NREAD回读长度按协议/PL约定向上对齐到256字节。 */
#define S231_FRAME_MAGIC_WORD 0xAAAAAAAAU /* 参数区首word固定为0xAAAAAAAA，供PL/DSP识别参数块。 */
#define S231_PARAM_END_MARK 0xEEEEEEEEU /* 参数区尾部6个word固定为0xEEEEEEEE，供PL/DSP检查边界。 */
#define S231_INPUT_FORMAT_WORD 0x00011010U /* CBF输入格式固定值，和PL参考程序保持一致。 */
#define S231_DSP_BANK0 0x80000000U /* DSP侧bank0地址，W2发送目标和NREAD读取目标都使用它。 */
#define S231_DSP_BANK1 0x84000000U /* DSP侧bank1地址，W2发送目标和NREAD读取目标都使用它。 */
#define S231_NREAD_RESULT_BYTES (4U * 1024U * 1024U) /* 21点模式下DSP固定回读4MiB结果。 */
#define S231_DMA_BD_BYTES 64U /* AXI DMA SG每个BD描述符固定64字节。 */
#define S231_DMA_BD_BTT_MASK ((1U << 23) - 1U) /* AXI DMA BD控制字低23bit为BTT传输长度。 */
#define S231_DMA_MAX_SEGMENT (S231_DMA_BD_BTT_MASK & ~7U) /* 单个BD最大传输长度按8字节对齐。 */
#define S231_DMA_DMASR_ERROR_MASK 0x00004070U /* AXI DMA S2MM错误状态位集合。 */
#define S231_DMA_S2MM_IRQ_W1C 0x00007000U /* AXI DMA中断状态清除位，写1清除。 */
#define S231_DMA_S2MM_IOC_IRQ_EN (1U << 12) /* AXI DMA完成中断使能位。 */
#define S231_DMA_S2MM_RESET (1U << 2) /* AXI DMA复位位，写1后等待硬件自动清0。 */
#define S231_DMA_S2MM_RUNSTOP 1U /* AXI DMA运行使能位，置1后S2MM开始取BD。 */
#define S231_DMA_BD_COMPLETE 0x80000000U /* BD状态bit31：该描述符已经完成。 */
#define S231_DMA_BD_STATUS_ERROR 0x70000000U /* BD状态中的DMA内部、从机和解码错误标志集合。 */
#define S231_DMA_WAIT_TIMEOUT_US 1000000U /* 单帧DMA最多等待1秒，防止湿端接收线程永久阻塞。 */
#define S231_CBF_MAGIC_VALUE 0x43424633U /* CBF寄存器MAGIC值，ASCII含义为CBF3。 */
#define S231_CBF_BUILD_ID_VALUE 0x00000124U /* 当前PL参考设计要求的BUILD_ID。 */
#define S231_CBF_STATUS_BUSY 0x00000001U /* CBF STATUS bit0表示当前正在处理。 */
#define S231_CBF_STATUS_ERROR 0x00000010U /* CBF STATUS bit4表示当前处理出错。 */
#define S231_CBF_CTRL_START 0x00000001U /* CBF CONTROL bit0写1启动本帧处理。 */
#define S231_CBF_CTRL_CLEAR_ERRORS 0x00000004U /* CBF CONTROL bit2写1清除错误状态。 */
#define S231_SR_LINK_READY_MASK 0xC0000000U /* SRIO链路状态高两位都为1表示两路链路就绪。 */
#define S231_DMA_RESET_TIMEOUT_US 1000000U /* DMA复位最多等待1秒，防止硬件异常时死循环。 */

/***********内置获取同步Roll的参数*********************/
INS_SENSOR_DATA_HEAD *ptr_InsSensor_Data_Para = NULL;
INS_SENSOR_DATA *ptr_Ins_Sensor_Data = NULL;
char Mems_Data[200000] = {0};//接收湿端MEMS数据，DDDD开头
unsigned int Recv_WetMemsNum = 0;//1ping 接收到湿端MEMS的个数
unsigned int Recv_WetMemsSumNum = 0;//接收到湿端MEMS的总个数，每一ping都会累加，最大不超过400个；
/********************************************/

/***********内置获取同步时刻的参数*********************/
unsigned int Mems_PPS_Num_Last = 0;
float Ins_Zda_Time = 0.0f;
char Ins_zda_time_hour[2] = {0}, Ins_zda_time_min[2] = {0},Ins_zda_time_sec[6] = {0};
char* pIns_zda_sec = Ins_zda_time_sec + 6;
char Ins_flag_date = 0;
/********************************************/

/***********外置获取同步Roll的参数*********************/
EXT_GPZDA_DATA *ptr_Ext_Gpzda_Data = NULL;
char Ext_Gpzda_Data[128000] = {0};//接收干端数据，DDDD开头
char Ext_Tss1_Data[128000] = {0};//接收干端数据，DDDD开头
char Ext_Handing_Data[128000] = {0};
unsigned int Recv_ExtSensorNum = {0};//1ping 接收到湿端MEMS的个数
unsigned int Recv_ExtSensor_SumNum = 0;//接收到湿端MEMS的总个数，每一ping都会累加，最大不超过400个；
/********************************************/

/***********外置获取同步时刻的参数*********************/
unsigned int ExtSensor_PPS_Num_Last = 0;
float Ext_Zda_Time = 0.0f;
char Ext_zda_time_hour[2] = {0}, Ext_zda_time_min[2] = {0}, Ext_zda_time_sec[6] = {0};
char* pExt_zda_sec = Ext_zda_time_sec + 6;
char Ext_flag_date = 0;
/********************************************/

float Sonar_Sync_Time = 0.0f;
float sonar_stime = 0.0f;
float Roll_Sequential_Value[400] = {0};//橫摇序列值
float Motion_Time_Sync[400] = {0};
float Roll_Time_Diff[400] = {0};//橫摇与声学数据时间差预留
float roll_value = 0.0f;


#define MAX_SIZE 10000

int memory[MAX_SIZE] = {0};
int top = -1;

void* my_malloc(int size) {
    if (top + size > MAX_SIZE) {
        return NULL;
    }
    void* ptr = memory + top;
    top += size;
    return ptr;
}

void my_free(void* ptr) {

}

//动态内存分配
void * tmalloc(int size_t)
{
    static int array[5120] = {0};/*空间不能大于65535*/
    int * head = 0; /* 表头，组成格式为0xcdxxxxac，其中xx代表已经分配了的空间长度 */
    int * nextHead = 0;
    int *ret = 0;
    int status = 0;
    int i = 0;
    int intSize;
    
    if(size_t > sizeof(array) || size_t < 1)
    {
        return ret;
    }
    
    head = array;   /*初始化表指针*/
    intSize =  size_t / 4;   /*计算需要多少int型空间*/
    if(size_t % 4 != 0)
    {
        intSize += 1;
    }
    
    do
    {
        if((head + intSize) >= array + (sizeof(array) / 4)) /*内存不足，分配失败*/
        {
            status = 1;
            break;
        }
        if((head[0] >> 24) == 0xcd) /*内存被使用*/
        {
            if((head[0] & 0xff) == 0xac)
            {
                 head +=  ((head[0] >> 8) & 0xffff) + 1;
            }   
        }
        else  /*内存没有被使用*/
        {
            nextHead = head + 1;
            for(i = 0;i < intSize;i++)
            {
                if((nextHead[0] >> 24) != 0xcd) 
                {
                    nextHead++;
                }
                else /*找到下一个表头*/
                {
                   head = nextHead;
                   break;
                }
            }
            if(i == intSize)//找到未被分配的空间
            {
                break;
            }
        }
    
    }while(1);
    
    if(status == 0) /*更新表头，返回地址*/
    {
        head[0] =(0xcd0000ac | (intSize << 8));
        ret = head + 1;
    }
    else
    {
        ret = 0;
    }
    
    return ret;
}

//内存释放
int tfree(void * ptemp)
{
    int *head = 0;
    int size = 0;
    int i = 0;

    /*
     * 调用方在异常路径下可能没有拿到临时缓冲区。
     * 释放函数必须先判断空指针，避免对 NULL 做指针运算并访问 head[0]。
     */
    if (ptemp == NULL)
    {
        return 0;
    }
    
    head = (int *)ptemp;
    head -= 1;
    
    if((head[0] >> 24) != 0xcd)
    {
        return  0;
    }
    size = (head[0] >> 8) & 0xffff;
    
    for(i = 0;i <= size;i++)
    {
        head[i] = 0;
    } 
    return  1;    
}



/*****************************************************************************
 * * description : 切换管道时,清空同步的相关buf
 * * return       {*}
 * * Date        : 2022-11-02 13:58:31
 * * Other
 ******************************************************************************/
void Change_InsModeState_ClearBuf(void)
{
    Sonar_FristPing_flag = 0;
    Recv_WetMemsNum = 0;
    Recv_WetMemsSumNum = 0;
    Recv_ExtSensorNum = 0;
    Recv_ExtSensor_SumNum = 0;
    Mems_PPS_Num_Last = 0;
    ExtSensor_PPS_Num_Last = 0;
    Ins_Zda_Time = 0;
    Ext_Zda_Time = 0;
    Ext_flag_date = 0;
    Ins_flag_date = 0;
    Sonar_Sync_Time = 0;
    roll_value = 0;
    memset(Ext_zda_time_hour,0,sizeof(Ext_zda_time_hour));
    memset(Ext_zda_time_min,0,sizeof(Ext_zda_time_min));
    memset(Ext_zda_time_sec,0,sizeof(Ext_zda_time_sec));
    memset(Ins_zda_time_hour,0,sizeof(Ins_zda_time_hour));
    memset(Ins_zda_time_min,0,sizeof(Ins_zda_time_min));
    memset(Ins_zda_time_sec,0,sizeof(Ins_zda_time_sec));
    memset(Roll_Time_Diff,0,sizeof(Roll_Time_Diff));
    memset(Motion_Time_Sync,0,sizeof(Motion_Time_Sync));
    memset(Roll_Sequential_Value,0,sizeof(Roll_Sequential_Value));

    Sensor1_Num = 0;
    Sensor2_Num = 0;
    Sensor3_Num = 0;
    Sensor4_Num = 0;
}

  
 #if 0 
void Test_SYNC_Time_Roll_Func(void)
{
  
    fwrite(&(IQData_ParaHead_Last.PingCount), 4, 1, fb);
    printf("Fwrite_PingCnt=%d\n", IQData_ParaHead_Last.PingCount);
    fwrite(&Recv_WetMemsSumNum, 4, 1, fb);
    printf("Fwrite_Recv_WetMemsSumNum=%d\n", Recv_WetMemsSumNum);
    fwrite(&Motion_Time_Sync, 400, 4, fb);
    //printf("Motion_Time_Sync=%f\n", Motion_Time_Sync[0]);
    fwrite(&Sonar_Sync_Time, 4, 1, fb);
    printf("Fwrite_sonar_sync_time=%f\n", Sonar_Sync_Time);
    fwrite(&Roll_Sequential_Value, 400, 4, fb);
    //printf("wuroll=%f\n", Roll_Sequential_Value[0]);
    fwrite(&(ptr_fpga_register_data->fpga_registers_parameters.Mems_Num), 4, 1, fb);
    printf("Fwrite_Mems_Num=%d\n", ptr_fpga_register_data->fpga_registers_parameters.Mems_Num);
    fwrite(&Toa_Time_s, 512, 4, fb);
    //printf("Toa_Time_s=%f\n", Toa_Time_s[0]); 
}
#endif

/*****************************************************************************
 * * description : 1.设置FPGA波束形成时需要的最左侧波速开角即波速角的最小值
 *                 2.将扇形区域的角度均分为512份,每一份即为波速开角的递增系数
 *                 3.2^29 = 536870912 FPGA是29位小数
 *                 4.除以180的角度与弧度的转换，其中乘以pi在FPGA中实现
 * * return       {*}
 * * Date        : 2022-10-13 16:05:27
 * * Other       : [波束下放]
 ******************************************************************************/
static void Set_Beam_Angle_Value(void)
{
    pthread_mutex_lock(&mut2);
    Overall_Angle = (cmd_package.Finish_Angle - cmd_package.Start_Angle);//-65~65
    Beam_Angle = ((0.5 * Overall_Angle / 512) - (Overall_Angle / 2));//-64.873声纳图左边第一个波速角度
    ptr_fpga_register_data->fpga_registers_parameters.Beam_Angle = (int)(Beam_Angle / 180 * FPGA_2_29);//开角角度
    ptr_fpga_register_data->fpga_registers_parameters.Angle_k = (int)(Overall_Angle / 180 / 512 * FPGA_2_29);//512波束均分130度后的弧度值
    pthread_mutex_unlock(&mut2);
}

/*****************************************************************************
 * * description : 获取声速
 * * return       {*}
 * * Date        : 2022-10-13 14:08:30
 * * Other       : [波束下放]
 ******************************************************************************/
float svp_n = 0.0f;//20240109svp有时解析为0,加一个中间变量保存上次解析的svp
static void Set_SoundSpeed_Value(void)
{  
    int n = 0;
  //  pthread_mutex_lock(&mut2);
	if (cmd_package.Manual_SoundSpeed > 1) //手动声速,即显控下发的声速值
    {
        ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed = cmd_package.Manual_SoundSpeed;
	/*	DBG("Recv upper soundspeed\n");
		DBG("Svp_Mode = %f\t Svp_Value = %f\n", cmd_package.Manual_SoundSpeed ,ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed);
 */       
    }
    else if (cmd_package.Manual_SoundSpeed < 1)//表声上传的声速
	{
        if (Sensor4_Num > 0)
        {
	//	DBG("Sensor4_Num = %d\n",Sensor4_Num);
            for ( n = 0; n < 8; ++n)
            {
           //     Svp_Data[n] = *(unsigned char*)(IQData + IQDataSizeFromWetSend - 6 + SIZE_OF_SENSOR_FIRST + SIZE_OF_LONG * 4 + (Sensor1_Num + Sensor2_Num + Sensor3_Num) * SIZE_OF_ONE_PING_SENSOR + 16 + 2 + n * 2);
				Svp_Data[n] = *(unsigned char*)(IQData+SIZE_OF_WET_SONAR_FIRST+8+ptr_IQData_ParaHead->IQ_Data_Length+4+SIZE_OF_LONG*5+(Sensor1_Num+Sensor2_Num+Sensor3_Num)*SIZE_OF_ONE_PING_SENSOR+16+2+n*2);
			}
//			DBG("Svp_Data = %s\n",Svp_Data);
			if(strtof(Svp_Data,NULL)< 1250 || strtof(Svp_Data,NULL) > 1600 )
			{
				ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed = svp_n;
			} 
			else
			{
				ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed = strtof(Svp_Data,NULL);
				svp_n = strtof(Svp_Data,NULL);
			}
		}
		else 
		{
		//	ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed = 1500;
			ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed = svp_n;
		}
	}
    else
    {
        ;//空语句
    }
//	DBG("Svp_Mode = %f\t Svp_Value = %f\n", cmd_package.Manual_SoundSpeed ,ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed);
    // pthread_mutex_unlock(&mut2);
}

/*****************************************************************************
 * * description : 设备横摇补偿系数
 * * return       {*}
 * * Date        : 2022-10-13 17:07:20
 * * Other       : [波束下放]
 ******************************************************************************/
static void Set_Coe_Value(void)
{
    long long PwmFreq = 400;
    pthread_mutex_lock(&mut2);
    ptr_fpga_register_data->fpga_registers_parameters.Roll_Coe = PwmFreq * (-3.5) / (ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed) * FPGA_2_30;
    ptr_fpga_register_data->fpga_registers_parameters.Tao_Coe = 35 / (ptr_fpga_register_data->fpga_registers_parameters.Manual_SoundSpeed) * FPGA_2_30;
    pthread_mutex_unlock(&mut2);
}

/*****************************************************************************
 * * description : 设置波束形成时FPGA所需要的参数
 * * return       {*}
 * * Date        : 2022-10-13 17:28:21
 * * Other       : [波束下放]
 ******************************************************************************/
static void Set_BeamDown_Fpga_Para(void)
{
    Set_Beam_Angle_Value();
  //  Set_SoundSpeed_Value();
    Set_Coe_Value();
}

/*****************************************************************************
 * * description : 读取MEMS数据的参数头:DDDD开头
 * * param        {char} *pmems_data    要读出的MEMSdata包
 * * param        {int} offset          第几包的MEMSdata
 * * return       {*}                   正确返回0
 * * Date        : 2022-10-20 13:14:41
 * * Other
 ******************************************************************************/
static int Read_Mems_Data_Head(char *pmems_data,int offset)
{

 //   if (NULL != (ptr_InsSensor_Data_Para = (INS_SENSOR_DATA_HEAD*)malloc(INSSENSOR_DATA_PARA_LEN)))
    {
	//	DBG("malloc 3\n");
        memset(ptr_InsSensor_Data_Para, 0L, INSSENSOR_DATA_PARA_LEN);
        //memcpy(ptr_InsSensor_Data_Para, (pmems_data + offset * 1024),INSSENSOR_DATA_PARA_LEN);
        memcpy(ptr_InsSensor_Data_Para, (pmems_data + offset * 256),INSSENSOR_DATA_PARA_LEN);
       // ptr_InsSensor_Data_Para->PPS_ms = ptr_InsSensor_Data_Para->PPS_ms / 100000;
        return OK;
    }
 //   else  return FAIL;
}

/*****************************************************************************
 * * description : 读取一包MEMS data:$snc200开头
 * * param        {char} *pmems_data  要读取的MEMSdata
 * * param        {int} offset        要读取第几个MEMSdata
 * * return       {*}                 正确返回0
 * * Date        : 2022-10-20 13:16:06
 * * Other
 ******************************************************************************/
static int Read_Mems_OnePack_Data(char *pmems_data,int offset)
{
  //  if (NULL != (ptr_Ins_Sensor_Data = (INS_SENSOR_DATA*)malloc(INSSENSOR_DATA_LEN)))
    {
        memset(ptr_Ins_Sensor_Data, 0L, INSSENSOR_DATA_LEN);
        //memcpy(ptr_Ins_Sensor_Data, (pmems_data + (offset*1024) + INSSENSOR_DATA_PARA_LEN), INSSENSOR_DATA_LEN);
        memcpy(ptr_Ins_Sensor_Data, (pmems_data + (offset*256) + INSSENSOR_DATA_PARA_LEN), INSSENSOR_DATA_LEN);
        return OK;
    } 
  //  else  return FAIL;
}

/*****************************************************************************
 * * description : 得到MEMSdata里的那些参数
 * * param        {char} l_range    获得参数的范围
 * * param        {char} h_range    获得参数的范围
 * * return       {*}               正确返回0
 * * Date        : 2022-10-20 13:17:54
 * * Other
 ******************************************************************************/
static int Get_Data_for_Mems(char l_range,char h_range)
{
    int rd_inscnt = 0;
    unsigned char* pRdIns = NULL;
    unsigned char* ptr_Read_Ins = NULL;
    unsigned char* ptr_Read_Ins_base = NULL;
    int val_cnt = 0;
    int Ins_Len = 0;

    if (NULL == (ptr_Read_Ins = (unsigned char*)tmalloc(INSSENSOR_DATA_LEN)))
		return FAIL;

    /*
     * ptr_Read_Ins会在字段解析过程中不断向后移动，
     * 所以必须单独保存分配块首地址，释放时不能使用移动后的指针。
     */
    ptr_Read_Ins_base = ptr_Read_Ins;
//	DBG("malloc 2\n");
    memset(ptr_Read_Ins, 0L, INSSENSOR_DATA_LEN);
    pRdIns = (unsigned char*)ptr_Ins_Sensor_Data;
    while (!(*(unsigned char*)(pRdIns - 2) == 0X2A))//*
    {
//	DBG("pRdIns = %#x\n",(*(unsigned char*)(pRdIns-2)));
        rd_inscnt = 0;
        while (0 != strncmp((char*)pRdIns, ",", sizeof(char)))
        {
            if ((*(unsigned char*)pRdIns == 0x2A))  break;
            strncpy((char*)(ptr_Read_Ins + rd_inscnt), (char*)pRdIns, sizeof(char));
            pRdIns = pRdIns + 2;
            rd_inscnt = rd_inscnt + 1;
        }
        strncpy((char*)(ptr_Read_Ins + rd_inscnt), "\0", sizeof(char));
        Ins_Len = rd_inscnt + 1;
	//	DBG("val_cnt = %d\n",val_cnt);
        if ((val_cnt > l_range) && (val_cnt < h_range))//只取ZDA
        {
            switch (val_cnt)
            {
                case 0: memcpy(ptr_Ins_Sensor_Data->Ins_Head, ptr_Read_Ins, Ins_Len); break;
                case 1: memcpy(ptr_Ins_Sensor_Data->Ins_GnggaHead, ptr_Read_Ins, Ins_Len); break;
                case 2: memcpy(ptr_Ins_Sensor_Data->Ins_UTCtime, ptr_Read_Ins, Ins_Len); break;
                case 3: memcpy(ptr_Ins_Sensor_Data->Ins_Lat, ptr_Read_Ins, Ins_Len); break;
                case 4: memcpy(ptr_Ins_Sensor_Data->Ins_NorS, ptr_Read_Ins, Ins_Len); break;
                case 5: memcpy(ptr_Ins_Sensor_Data->Ins_Long, ptr_Read_Ins, Ins_Len); break;
                case 6: memcpy(ptr_Ins_Sensor_Data->Ins_WorE, ptr_Read_Ins, Ins_Len); break;
                case 7: memcpy(ptr_Ins_Sensor_Data->Ins_Sat_State, ptr_Read_Ins, Ins_Len); break;
                case 8: memcpy(ptr_Ins_Sensor_Data->Ins_Sat_Num, ptr_Read_Ins, Ins_Len); break;
                case 9: memcpy(ptr_Ins_Sensor_Data->Ins_HDOP, ptr_Read_Ins, Ins_Len); break;
                case 10:memcpy(ptr_Ins_Sensor_Data->Ins_Alt, ptr_Read_Ins, Ins_Len); break;
                case 11:memcpy(ptr_Ins_Sensor_Data->Ins_Alt_Unit, ptr_Read_Ins, Ins_Len); break;
                case 12:memcpy(ptr_Ins_Sensor_Data->Ins_Undulation, ptr_Read_Ins, Ins_Len); break;
                case 13:memcpy(ptr_Ins_Sensor_Data->Ins_Ins_Undulation_Unit, ptr_Read_Ins, Ins_Len); break;
                case 14:memcpy(ptr_Ins_Sensor_Data->Ins_Age, ptr_Read_Ins, Ins_Len); break;
                case 15:memcpy(ptr_Ins_Sensor_Data->Ins_Age_ID, ptr_Read_Ins, Ins_Len); break;
                case 16:memcpy(ptr_Ins_Sensor_Data->Ins_GpzdaHead, ptr_Read_Ins, Ins_Len); break;
                case 17:memcpy(ptr_Ins_Sensor_Data->Ins_Zdatime, ptr_Read_Ins, Ins_Len); break;
                case 18:memcpy(ptr_Ins_Sensor_Data->Ins_Day, ptr_Read_Ins, Ins_Len); break;
                case 19:memcpy(ptr_Ins_Sensor_Data->Ins_Mounth, ptr_Read_Ins, Ins_Len); break;
                case 20:memcpy(ptr_Ins_Sensor_Data->Ins_Year, ptr_Read_Ins, Ins_Len); break;
                case 21:memcpy(ptr_Ins_Sensor_Data->Ins_Hour, ptr_Read_Ins, Ins_Len); break;
                case 22:memcpy(ptr_Ins_Sensor_Data->Ins_Min, ptr_Read_Ins, Ins_Len); break;
                case 23:memcpy(ptr_Ins_Sensor_Data->Ins_HehdtHead, ptr_Read_Ins, Ins_Len); break;
                case 24:memcpy(ptr_Ins_Sensor_Data->Ins_Heading, ptr_Read_Ins, Ins_Len); break;
                case 25:memcpy(ptr_Ins_Sensor_Data->Ins_Tss1Head, ptr_Read_Ins, Ins_Len); break;
                case 26:memcpy(ptr_Ins_Sensor_Data->Ins_Hor_Acc, ptr_Read_Ins, Ins_Len); break;
                case 27:memcpy(ptr_Ins_Sensor_Data->Ins_Ver_Acc, ptr_Read_Ins, Ins_Len); break;
                case 28:memcpy(ptr_Ins_Sensor_Data->Ins_Space, ptr_Read_Ins, Ins_Len); break;
                case 29:memcpy(ptr_Ins_Sensor_Data->Ins_Heave, ptr_Read_Ins, Ins_Len); break;
                case 30:memcpy(ptr_Ins_Sensor_Data->Ins_horH, ptr_Read_Ins, Ins_Len); break;
                case 31:memcpy(ptr_Ins_Sensor_Data->Ins_Roll, ptr_Read_Ins, Ins_Len); break;
                case 32:memcpy(ptr_Ins_Sensor_Data->Ins_Space1, ptr_Read_Ins, Ins_Len); break;
                case 33:memcpy(ptr_Ins_Sensor_Data->Ins_Pitch, ptr_Read_Ins, Ins_Len); break;
                case 34:memcpy(ptr_Ins_Sensor_Data->Ins_Ant1_Num, ptr_Read_Ins, Ins_Len); break;
                case 35:memcpy(ptr_Ins_Sensor_Data->Ins_Ant2_Num, ptr_Read_Ins, Ins_Len); break;
                case 36:memcpy(ptr_Ins_Sensor_Data->Ins_Imu_State, ptr_Read_Ins, Ins_Len); break;
                case 37:memcpy(ptr_Ins_Sensor_Data->Ins_Sys_State, ptr_Read_Ins, Ins_Len); break;
                case 38:memcpy(ptr_Ins_Sensor_Data->Ins_GpVtgHead, ptr_Read_Ins, Ins_Len); break;
                case 39:memcpy(ptr_Ins_Sensor_Data->Ins_Spd_Speed, ptr_Read_Ins, Ins_Len); break;
                case 40:memcpy(ptr_Ins_Sensor_Data->Ins_Spd_Unit, ptr_Read_Ins, Ins_Len); break;
                default: break;
            }
        }
        pRdIns = pRdIns + 2;
        ptr_Read_Ins = ptr_Read_Ins + rd_inscnt;
        val_cnt++;
	//	DBG("val_cnt = %d\n",val_cnt);
    }
    rd_inscnt = 0;
    val_cnt = 0;
    Ins_Len = 0;

    /* 先释放真实的分配块首地址，再清空工作指针，避免释放 NULL。 */
	tfree(ptr_Read_Ins_base);
	ptr_Read_Ins = NULL;
	ptr_Read_Ins_base = NULL;
	pRdIns = NULL;
	top = -1;
    return OK;
}

/*****************************************************************************
 * * description :获得声纳同步时间
 * * param        {char} *ptr_mes     MEMSdata
 * * param        {unsigned int} cnt  1ping中共几包传感器数据
 * * return       {*}
 * * Date        : 2022-10-20 13:18:37
 * * Other       :通过上1ping的数据来计算当前ping的同步时间
 ******************************************************************************/
void Get_SonarSyncTime_Ins(char *ptr_mes,unsigned int cnt)
{
    unsigned int Mems_PPS_Num = 0;  
    
	if(NULL ==  (ptr_InsSensor_Data_Para = (INS_SENSOR_DATA_HEAD*)my_malloc(INSSENSOR_DATA_PARA_LEN)))
		return FAIL;
    if (NULL == (ptr_Ins_Sensor_Data = (INS_SENSOR_DATA*)my_malloc(INSSENSOR_DATA_LEN)))
    	return FAIL; 
    for (Recv_WetMemsNum = 0; Recv_WetMemsNum < cnt; ++Recv_WetMemsNum)
    {
	//	DBG("Mems_PPS_Num = %d\n",ptr_InsSensor_Data_Para->PPS_s);
        Read_Mems_Data_Head(ptr_mes,Recv_WetMemsNum);//ddddd
        Mems_PPS_Num = ptr_InsSensor_Data_Para->PPS_s;
        
        if (Mems_PPS_Num != Mems_PPS_Num_Last)//当PPS跳变时，取跳变时的ZDA时间；
        {
            DBG("****************Mems_PPS_Num Changed = [%d]********************\n",Mems_PPS_Num);
            Mems_PPS_Num_Last = Mems_PPS_Num; 
            Read_Mems_OnePack_Data(ptr_mes,Recv_WetMemsNum);//snc200
            Get_Data_for_Mems(16, 21);
            strncpy(Ins_zda_time_hour, (char*)ptr_Ins_Sensor_Data->Ins_Zdatime, 2);
            strncpy(Ins_zda_time_min, (char*)ptr_Ins_Sensor_Data->Ins_Zdatime + 2, 2);
            strncpy(Ins_zda_time_sec, (char*)ptr_Ins_Sensor_Data->Ins_Zdatime + 4, 6);
        }
        Ins_Zda_Time = atoi(Ins_zda_time_hour) * 60 * 60 + atoi(Ins_zda_time_min) * 60 + strtof(Ins_zda_time_sec, &pIns_zda_sec);
        Sonar_Sync_Time = Ins_Zda_Time + (float)IQData_ParaHead_Last.TimeStamp / 1000 + (float)IQData_ParaHead_Last.PPS_Number - (float)Mems_PPS_Num;//声学同步时间 ,单位是S，精度mS

        ptr_fpga_register_data->fpga_registers_parameters.Time_Sec = (Sonar_Sync_Time - (int)floor(Sonar_Sync_Time)) + (int)(floor(Sonar_Sync_Time)) % 60;
        ptr_fpga_register_data->fpga_registers_parameters.Time_Min = ((int)(floor(Sonar_Sync_Time)) % 3600 - ((int)(floor(Sonar_Sync_Time)) % 3600) % 60) / 60;
        ptr_fpga_register_data->fpga_registers_parameters.Time_Hours = ((int)(floor(Sonar_Sync_Time)) - (int)(floor(Sonar_Sync_Time)) % 3600) / 3600; 
        if (Ins_flag_date == 0)
        {
            Ins_flag_date = 1;
            ptr_fpga_register_data->fpga_registers_parameters.Time_Date = atoi((char*)ptr_Ins_Sensor_Data->Ins_Day);
            ptr_fpga_register_data->fpga_registers_parameters.Time_Date |= (atoi((char*)ptr_Ins_Sensor_Data->Ins_Mounth) << 8);
            ptr_fpga_register_data->fpga_registers_parameters.Time_Year = atoi((char*)ptr_Ins_Sensor_Data->Ins_Year);
        }  
	}
/*	Debug("PingCount = [%d] Mems_PPS_Num = [%d] Sonar_Sync_Time = [%f] \n",IQData_ParaHead_Last.PingCount,Mems_PPS_Num,Sonar_Sync_Time);
	Debug("Ins_Zda_Time = [%f]\n" ,Ins_Zda_Time);
	Debug("Sonar_Sync_Time = [%f]\n",Sonar_Sync_Time);
	Debug("sec = [%f]\n",ptr_fpga_register_data->fpga_registers_parameters.Time_Sec);
	Debug("min = [%d]\n", ptr_fpga_register_data->fpga_registers_parameters.Time_Min);
	Debug("hours = [%d]\n",ptr_fpga_register_data->fpga_registers_parameters.Time_Hours);
	Debug("data = [%d]\n",ptr_fpga_register_data->fpga_registers_parameters.Time_Date);
	Debug("year = [%d]\n",ptr_fpga_register_data->fpga_registers_parameters.Time_Year);*/
	ptr_InsSensor_Data_Para = NULL;
	ptr_Ins_Sensor_Data = NULL;
	my_free(ptr_InsSensor_Data_Para);
	my_free(ptr_Ins_Sensor_Data);
	top = -1;
}

/*****************************************************************************
 * * description : 获得当前ping的传感器时间及传感器的横摇值
 * * param        {char*} ptr_mes       MEMSdata
 * * param        {unsigned int} cnt    1ping中共几包传感器数据
 * * return       {*}
 * * Date        : 2022-10-20 13:20:09
 * * Other
 ******************************************************************************/
void Get_MotionTime_Roll_Ins(char* ptr_mes,unsigned int cnt)
{
    float Motion_Time;
    char motion_time_hour[2], motion_time_min[2],motion_time_sec[6];
    char* pIns_motiom_sec = motion_time_sec + 6;
    int  i = 0;

    if (NULL == (ptr_Ins_Sensor_Data = (INS_SENSOR_DATA*)my_malloc(INSSENSOR_DATA_LEN)))
    	return FAIL; 
	for (Recv_WetMemsNum = 0; Recv_WetMemsNum < cnt; ++Recv_WetMemsNum)
    {
        Read_Mems_OnePack_Data(ptr_mes,Recv_WetMemsNum); 
        Get_Data_for_Mems(16, 32);

        strncpy(motion_time_hour, (char*)ptr_Ins_Sensor_Data->Ins_Zdatime, 2);
        strncpy(motion_time_min, (char*)ptr_Ins_Sensor_Data->Ins_Zdatime + 2, 2);
        strncpy(motion_time_sec, (char*)ptr_Ins_Sensor_Data->Ins_Zdatime + 4, 6);
        Motion_Time = atoi(motion_time_hour) * 60 * 60 + atoi(motion_time_min) * 60 + strtof(motion_time_sec, &pIns_motiom_sec);
        
        Motion_Time_Sync[Recv_WetMemsSumNum] = Motion_Time;
        Roll_Sequential_Value[Recv_WetMemsSumNum] = (float)(atoi((char*)ptr_Ins_Sensor_Data->Ins_Roll)) / 100;//* 3.14 / 180;角度值
        Recv_WetMemsSumNum = Recv_WetMemsSumNum + 1;
    }
    for (i = 0; i < Recv_WetMemsSumNum; i++)
    {
        Roll_Time_Diff[i] = Motion_Time_Sync[i] - Sonar_Sync_Time;
        //printf("Roll_Time_Diff[%d] = %f   Roll_Sequential_Value[%d] = %f\n", i, Roll_Time_Diff[i] * 1000, i, Roll_Sequential_Value[i] * 3.14 / 180);
    }
    ptr_fpga_register_data->fpga_registers_parameters.Recv_SensorSumNum = Recv_WetMemsSumNum;
	//printf("fpga_registers_parameters.Recv_SensorSumNum = %d\n", Recv_SensorSumNum);
	ptr_Ins_Sensor_Data = NULL;
	my_free(ptr_Ins_Sensor_Data);
//	free(ptr_Ins_Sensor_Data);
	top = -1;

}

/*****************************************************************************
 * * description :设置FPGA波束形成需要的参数,并计算发送给DSP的长度
 * * return       {*}
 * * Date        : 2022-10-20 13:27:48
 * * Other
 ******************************************************************************/
void Set_Fpga_To_Dsp_Para(void)
{
    //湿端FPGA抽样后为40K带宽，%2标识FPGA需要偶数个数的采样点，如是奇数则抛掉
    ptr_fpga_register_data->fpga_registers_parameters.Ad_Sn = ((ptr_IQData_ParaHead_Last->IQ_Data_Length / 384 / SAMPLE_FACTOR) - (ptr_IQData_ParaHead_Last->IQ_Data_Length / 384 / SAMPLE_FACTOR) % 2) * SAMPLE_FACTOR;
    //由于湿端的40K带宽干端波束形成做不过来，所有又一次抽样，抽成了20K
    ptr_fpga_register_data->fpga_registers_parameters.Ad_Sn_After = (ptr_IQData_ParaHead_Last->IQ_Data_Length / 384 / SAMPLE_FACTOR) - (ptr_IQData_ParaHead_Last->IQ_Data_Length / 384 / SAMPLE_FACTOR) % 2;
    //1ping中每200个采样点取一个Mems_Num
    ptr_fpga_register_data->fpga_registers_parameters.Mems_Num = (unsigned int)((ptr_fpga_register_data->fpga_registers_parameters.Ad_Sn_After + 199) / 200);
    ptr_fpga_register_data->fpga_registers_parameters.Data_Type = ptr_IQData_ParaHead_Last->DataType;

    //波束形成后的数据长度：1ping的采样点数*（全阵位宽+子阵位宽）/ 8 * 波束个数 = Ad_Sn_After * 12 * 512
    BeamDataSize = ptr_fpga_register_data->fpga_registers_parameters.Ad_Sn_After * 12;
  //  BeamDataSize = ptr_fpga_register_data->fpga_registers_parameters.Ad_Sn_After * 12 * 512;
    BeamDataTotalSize = BeamDataSize + SIZE_OF_RAPIDIO_FIRST;//从帧头开始算起（含帧头）-> 波束数据结束
    All_Rapidio_SendToDSP_Size = 4 + BeamDataTotalSize + 4 + 4 + 4 + IQDataTotalSize_Last + 2;//除去帧头及帧长度后面的所有字节的长度,以$$结尾
    //将总长分为一半,保证每2k长度发送
    Rapidio1_Send_Size = All_Rapidio_SendToDSP_Size / 2 - (All_Rapidio_SendToDSP_Size / 2) % 2048;
    Rapidio2_Send_Size = (All_Rapidio_SendToDSP_Size / 2 + (All_Rapidio_SendToDSP_Size / 2) % 2048)  + 2048 - (All_Rapidio_SendToDSP_Size / 2 + (All_Rapidio_SendToDSP_Size / 2) % 2048) % 2048;
}


/*****************************************************************************
 * * description : 将长度向上对齐到指定边界。
 * * return      : 对齐后的长度。
 * * Other       : S231 DMA长度按256字节对齐，BD分段按8字节对齐。
 ******************************************************************************/
static unsigned int S231_AlignUp(unsigned int value, unsigned int align)
{
    return (value + align - 1U) & ~(align - 1U); /* align必须是2的幂，这里用于256/8这类硬件对齐值。 */
}

/*****************************************************************************
 * * description : 检查mmap出来的DDR/寄存器窗口是否有效且容量足够。
 * * return      : OK表示可用，FAIL表示不能访问。
 * * Other       : uio_init失败时mem_ptr可能是NULL或MAP_FAILED(-1)，这里统一拦截。
 ******************************************************************************/
static int S231_CheckMappedWindow(const char *name, const volatile void *ptr, int map_size, unsigned int need_size)
{
    if ((ptr == NULL) || (ptr == (const volatile void *)-1)) /* mmap失败时不能继续访问，否则会直接段错误。 */
    {
        DBG("%s mmap pointer invalid\n", name); /* 打印窗口名称，便于确认是哪一段设备树/UIO映射失败。 */
        return FAIL; /* 返回失败，调用者停止本帧下发。 */
    }

    if ((map_size <= 0) || ((unsigned int)map_size < need_size)) /* 映射长度必须覆盖本次要访问的寄存器或DDR空间。 */
    {
        DBG("%s mmap size too small: map=%d need=%u\n", name, map_size, need_size); /* 打印实际映射长度和需要长度。 */
        return FAIL; /* 长度不足时返回失败，避免越界写到未映射区域。 */
    }

    return OK; /* 指针和长度都满足要求，可以继续访问。 */
}

/*****************************************************************************
 * * description : 获取上一ping的纯IQ声学数据地址和长度。
 * *               wet_to_dry()接收的数据格式为：
 * *               128字节湿端参数头 + IQ声学数据 + 传感器数据。
 * *               S231新流程给PL的输入只允许是IQ声学数据，所以这里统一
 * *               从IQData_Last + SIZE_OF_WET_SONAR_FIRST取数据起点，
 * *               从IQ_Data_Length取数据长度，避免后续函数重复写偏移逻辑。
 * * return       : OK表示payload/payload_len有效，FAIL表示本ping不能下发。
 * * Other        : 这里只负责取地址和做边界检查，不执行DDR拷贝。
 ******************************************************************************/
static int Get_LastIqPayload(unsigned char **payload, unsigned int *payload_len)
{
    unsigned int iq_len; /* 上一ping中真正的IQ声学数据长度；IQ_Data_Length不包含128字节参数头，也不是传感器长度。 */

    if ((payload == NULL) || (payload_len == NULL)) /* 调用者必须提供两个输出参数，否则无法返回IQ地址和长度。 */
    {
        return FAIL; /* 输出参数无效时直接失败，避免后续解引用空指针。 */
    }

    iq_len = ptr_IQData_ParaHead_Last->IQ_Data_Length; /* IQ_Data_Length表示RECV_WET_SONAR_FIRST中的声学IQ数据长度。 */

    if (iq_len == 0U) /* 长度为0说明上一ping没有有效IQ数据。 */
    {
        DBG("IQ payload length is zero\n"); /* 打印异常，现场可以从湿端包长度继续追。 */
        return FAIL; /* 空IQ不能下发给PL处理。 */
    }

    if (iq_len > IQ_A_SIZE) /* IQA/IQB单个buffer只有32MiB，不能让湿端异常长度越界。 */
    {
        DBG("IQ payload too large: %u\n", iq_len); /* 打印异常IQ长度，便于核对湿端长度字段。 */
        return FAIL; /* 长度超过IQA/IQB容量时停止本帧。 */
    }

    if ((SIZE_OF_WET_SONAR_FIRST + iq_len) > (unsigned int)IQDataTotalSize_Last) /* 128字节头+IQ长度不能超过上一ping总包长。 */
    {
        DBG("IQ payload overflow: head=%u iq=%u total=%d\n", SIZE_OF_WET_SONAR_FIRST, iq_len, IQDataTotalSize_Last); /* 打印包内三个关键长度。 */
        return FAIL; /* 长度关系不成立时不能继续，避免把传感器区或越界内存当作IQ。 */
    }

    if ((iq_len % S231_IQ_BYTES_PER_POINT) != 0U) /* S231要求IQ长度必须是832字节的整数倍。 */
    {
        DBG("IQ payload length %u is not aligned to %u bytes per point\n", iq_len, S231_IQ_BYTES_PER_POINT); /* 打印未对齐长度。 */
        return FAIL; /* 未对齐时无法得到准确IQ点数，所以本帧不启动PL。 */
    }

    *payload = (unsigned char *)IQData_Last + SIZE_OF_WET_SONAR_FIRST; /* 跳过128字节湿端参数头，得到PL真正要读的IQ起始地址。 */
    *payload_len = iq_len; /* 把IQ有效长度回填给调用者，用于DDR拷贝和CBF INPUT_BYTES。 */
    return OK; /* 地址和长度均有效，调用者可以继续配置S231链路。 */
}

/*****************************************************************************
 * * description : 根据IQ点数计算S231输出帧长度和DMA搬移长度。
 * * return      : OK表示frame_bytes/dma_bytes/beam_bytes有效，FAIL表示点数或容量异常。
 * * Other       : 公式来自PL参考实现：beam=(points-10)*256*24，valid=3730+beam；
 * *               协议有效帧长度与DMA实际搬移长度必须分开计算。
 ******************************************************************************/
static int S231_CalcFrameBytes(unsigned int iq_points, unsigned int *frame_bytes, unsigned int *dma_bytes, unsigned int *beam_bytes)
{
    unsigned int records; /* 保存有效快照转换出的波束记录数量。 */
    unsigned int beam_len; /* 保存PL输出波束数据长度。 */
    unsigned int valid_len; /* 保存未按DMA对齐前的有效帧长度。 */
    unsigned int aligned_len; /* 保存按8字节对齐后的DMA传输长度。 */

    if ((frame_bytes == NULL) || (dma_bytes == NULL) || (beam_bytes == NULL)) /* 三个输出参数必须都有效。 */
    {
        return FAIL; /* 输出参数缺失时不能回填长度。 */
    }

    if ((iq_points < S231_FRAME_MIN_POINTS) || (iq_points > S231_FRAME_MAX_POINTS)) /* IQ点数必须落在PL支持范围内。 */
    {
        DBG("S231 iq_points invalid: %u\n", iq_points); /* 打印异常点数，方便从湿端IQ长度追。 */
        return FAIL; /* 点数非法时不启动PL。 */
    }

    records = (iq_points - 10U) * S231_BEAM_RECORDS_PER_SNAPSHOT; /* 前10点为历史/延迟，不产生有效输出记录。 */
    beam_len = records * S231_BEAM_RECORD_BYTES; /* 波束数据长度=记录数*每条记录24字节。 */
    valid_len = S231_FIXED_VALID_BYTES + beam_len; /* 有效帧长度=固定3730字节+波束数据。 */
    aligned_len = S231_AlignUp(valid_len, S231_DMA_ALIGN_BYTES); /* DMA实际搬移长度按8字节向上对齐。 */

    if (aligned_len > FRAME_SIZE) /* 输出帧不能超过0x24000000窗口预留的FRAME_SIZE。 */
    {
        DBG("S231 frame too large: valid=%u dma=%u frame_window=%u\n", valid_len, aligned_len, FRAME_SIZE); /* 打印长度和窗口容量。 */
        return FAIL; /* 输出帧超过预留DDR窗口时停止，避免覆盖后续NREAD/BD等区域。 */
    }

    *frame_bytes = valid_len; /* 回填有效帧长度，W2发送长度用这个值。 */
    *dma_bytes = aligned_len; /* 回填DMA对齐长度，AXI DMA SG BTT用这个值。 */
    *beam_bytes = beam_len; /* 回填波束数据长度，旧协议frame+3712也需要这个值。 */
    return OK; /* 三个长度均计算成功。 */
}

/*****************************************************************************
 * * description : 判断本帧NREAD需要回读多少字节。
 * * return      : OK表示read_bytes有效，FAIL表示参数异常。
 * * Other       : 参考程序规定21点测试模式回读固定4MiB，正常帧按frame_bytes回读。
 ******************************************************************************/
static int S231_GetNreadBytes(unsigned int iq_points, unsigned int frame_bytes, unsigned int *read_bytes)
{
    if (read_bytes == NULL) /* 调用者必须提供输出指针。 */
    {
        return FAIL; /* 输出指针为空时不能回填NREAD长度。 */
    }

    if (iq_points == 21U) /* PL参考程序把21点作为固定4MiB NREAD测试/短帧模式。 */
    {
        *read_bytes = S231_NREAD_RESULT_BYTES; /* 21点模式固定读取4MiB结果。 */
    }
    else
    {
        *read_bytes = S231_AlignUp(frame_bytes, S231_NREAD_ALIGN_BYTES); /* 正常业务模式按256字节向上对齐NREAD长度。 */
    }

    if (*read_bytes > NREAD_SIZE) /* NREAD目标DDR窗口必须能容纳回读结果。 */
    {
        DBG("S231 nread too large: read=%u nread_window=%u\n", *read_bytes, NREAD_SIZE); /* 打印NREAD长度和窗口容量。 */
        return FAIL; /* 回读长度超过NREAD窗口时停止，避免DSP写越界。 */
    }

    return OK; /* NREAD长度可用。 */
}

/*****************************************************************************
 * * description : 将上一ping的纯IQ声学数据拷贝到当前选中的IQA/IQB DDR。
 * * return      : OK表示DDR拷贝完成，FAIL表示映射/长度/源数据异常。
 * * Other       : 新流程只写纯IQ数据，不再拼旧的@@@@/参数头/IQ整包/$$。
 ******************************************************************************/
static int S231_CopyIqPayloadToDdr(UIO_CONFIG_PARAMETER struct_uio_ddr)
{
    unsigned char *iq_payload = NULL; /* 指向IQData_Last+128，即上一ping的IQ声学数据起始位置。 */
    unsigned int iq_payload_len = 0U; /* 保存上一ping的IQ声学数据长度，单位字节。 */

    if (Get_LastIqPayload(&iq_payload, &iq_payload_len) != OK) /* 先统一校验IQ来源，避免重复手写偏移。 */
    {
        return FAIL; /* IQ来源无效时不写DDR。 */
    }

    if (S231_CheckMappedWindow("IQA/IQB", struct_uio_ddr.mem_ptr, struct_uio_ddr.mem_size, iq_payload_len) != OK) /* 检查当前IQA/IQB映射是否足够容纳IQ。 */
    {
        return FAIL; /* 目标DDR映射不可用时不拷贝。 */
    }

    my_copy((unsigned char *)struct_uio_ddr.mem_ptr, iq_payload, iq_payload_len); /* 将纯IQ声学数据写入当前选中的IQA/IQB输入区。 */
    __sync_synchronize(); /* 确保CPU写DDR完成后，再让PL从该物理地址读取。 */
    return OK; /* IQ输入区已经准备完成。 */
}


/*
    构造frame寄存器
*/
static void fill_frame(FPGA_CONFIG_PARAMETERS* ptr_fpga_registers_parameters, frame_t* p_frame)
{
    unsigned int i; /* 用于填充参数区末尾6个0xEEEEEEEE标记。 */

    if ((ptr_fpga_registers_parameters == NULL) || (p_frame == NULL)) /* 参数源或目标frame为空时直接返回，避免空指针写崩溃。 */
    {
        return; /* 上层会因为frame内容无效而停止本帧下发。 */
    }

    memset(p_frame, 0, sizeof(*p_frame)); /* 先清零本地frame_t，保证保留字段、CH_error和Roll空洞不会残留上一帧内容。 */

    /*参数头*/
    p_frame->f_parameter.head = S231_FRAME_MAGIC_WORD;
    /*伪三位模式开关开关*/
    p_frame->f_parameter.PTD_MOD = ptr_fpga_registers_parameters->PTD_Mod & 0xff;
    /*声纳侧扫开关*/
    p_frame->f_parameter.SIDE_MOD = ptr_fpga_registers_parameters->Side_Mod & 0xff;
    /*声纳图显示开关*/
    p_frame->f_parameter.SONAR_SHOW = ptr_fpga_registers_parameters->Sonar_Show & 0xff;
    /*底检测开关*/
    p_frame->f_parameter.BASH_TEST = ptr_fpga_registers_parameters->Base_Test & 0xff;
    /*声纳图像对比度*/
    p_frame->f_parameter.ClolorRatio = ptr_fpga_registers_parameters->Image_Rotio;
    /*1024差值模式*/
    p_frame->f_parameter.INT_MOD = ptr_fpga_registers_parameters->INT_Mod & 0xff;
    /*水体检测控制*/
    p_frame->f_parameter.WATER_DETECTION = ptr_fpga_registers_parameters->Water_Detection & 0xff;
    /*等角等距模式选择*/
    p_frame->f_parameter.Beamtype = ptr_fpga_registers_parameters->Beam_Type & 0xff;
    /*进场聚焦*/
    p_frame->f_parameter.FOCUS = ptr_fpga_registers_parameters->Focus & 0xff;
    /*水柱图像模式*/
    p_frame->f_parameter.Water_Dro = ptr_fpga_registers_parameters->Water_Column & 0xff;
    /*显示模式*/
    p_frame->f_parameter.IMAGE_Work_Mod = ptr_fpga_registers_parameters->Image_Work_Mode & 0xff;
    /*惯导模式选择*/
    p_frame->f_parameter.INS_MOD = ptr_fpga_registers_parameters->Ins_Mod & 0xff;
    /*接收数据模式*/
    p_frame->f_parameter.DATA_TYPE = ptr_fpga_registers_parameters->Data_Type & 0xff;
    /*横摇补偿*/
    p_frame->f_parameter.Roll_Oe = ptr_fpga_registers_parameters->Roll_Oe & 0xff;
    /*艏摇控制*/
    p_frame->f_parameter.Ctrl_bow = ptr_fpga_registers_parameters->Ctrl_Bow & 0xff;
    /*升沉控制*/
    p_frame->f_parameter.Ctrl_heave = ptr_fpga_registers_parameters->Ctrl_Heave & 0xff;
    /*手动门限控制开关*/
    p_frame->f_parameter.THR_CON = ptr_fpga_registers_parameters->Thr_Con & 0xff;
    /*旁瓣因子*/
    p_frame->f_parameter.SIDELOBE_FACTOR = ptr_fpga_registers_parameters->Sidelobe_Factor;
    /*手动门限上限*/
    p_frame->f_parameter.UP_LIM = ptr_fpga_registers_parameters->Up_Limit;
    /*手动门限下限*/
    p_frame->f_parameter.DOWN_LIM = ptr_fpga_registers_parameters->Down_Limit;
    /*手动门限倾斜角度*/
    p_frame->f_parameter.ANGLE_LIM = ptr_fpga_registers_parameters->Angle_Limit;
    /*声纳图高度*/
    p_frame->f_parameter.IMAGE_H = ptr_fpga_registers_parameters->Image_H;
    /*声纳图宽度*/
    p_frame->f_parameter.IMAGE_W = ptr_fpga_registers_parameters->Image_W;
    /*
     * frame+60对应参数区word 12，PS先按完整frame协议写入SIDE_RATIO。
     * PL在后续组帧阶段会把这个位置覆盖为当前有效声速。
     */
    p_frame->f_parameter.SIDE_RATIO = ptr_fpga_registers_parameters->Side_Rotio;
    /*
     * frame+64对应参数区word 13，PS按协议写入SVP_TEST。
     * 按当前协议，DSP最终不能把frame+64当作有效声速读取。
     */
    p_frame->f_parameter.SVP_TEST = ptr_fpga_registers_parameters->Manual_SoundSpeed;
    /*波束个数*/
    p_frame->f_parameter.Beam_num = ptr_fpga_registers_parameters->Beam_Num;
    /*起始角度*/
    p_frame->f_parameter.BeamOA_st = ptr_fpga_registers_parameters->Start_Angle;
    /*终止角度*/
    p_frame->f_parameter.BeamOA_fi = ptr_fpga_registers_parameters->Finish_Angle;
    /*每帧数据下包含的mems个数*/
    p_frame->f_parameter.Mems_Num = ptr_fpga_registers_parameters->Mems_Num;
    /*横摇值*/
    p_frame->f_parameter.Roll = ptr_fpga_registers_parameters->Roll_Value;
    /*纵摇值*/
    p_frame->f_parameter.angle = ptr_fpga_registers_parameters->Pitch;
    /*波束显示的指定波束波束号*/
    p_frame->f_parameter.Ch_Beam = ptr_fpga_registers_parameters->Ch_Beam;
    /*同步经度*/
    p_frame->f_parameter.Sync_Longitude = ptr_fpga_registers_parameters->Sync_Longitude;
    /*同步纬度*/
    p_frame->f_parameter.Sync_Latitude = ptr_fpga_registers_parameters->Sync_Latitude;
    /*同步高程*/
    p_frame->f_parameter.Sync_Height = ptr_fpga_registers_parameters->Sync_Height;
    /*同步品质因子*/
    p_frame->f_parameter.Sync_Factor = ptr_fpga_registers_parameters->Sync_Factor;
    /*同步航速*/
    p_frame->f_parameter.Sync_Factorync_Speed = ptr_fpga_registers_parameters->Sync_Factorync_Speed;
    /*同步升沉*/
    p_frame->f_parameter.Sync_Heave = ptr_fpga_registers_parameters->Sync_Heave;
    /*Ping间距离*/
    p_frame->f_parameter.Ping_Dis = ptr_fpga_registers_parameters->Ping_Dis;
    /*航向*/
    p_frame->f_parameter.Heading = ptr_fpga_registers_parameters->Heading;
    /*日*/
    p_frame->f_parameter.Time_Date = ptr_fpga_registers_parameters->Time_Date & 0xffff;
    /*年*/
    p_frame->f_parameter.Time_Year = ptr_fpga_registers_parameters->Time_Year & 0xffff;
    /*秒*/
    p_frame->f_parameter.Time_Sec = ptr_fpga_registers_parameters->Time_Sec;
    /*分*/
    p_frame->f_parameter.Time_Min = ptr_fpga_registers_parameters->Time_Min & 0xffff;
    /*时*/
    p_frame->f_parameter.Time_Hours = ptr_fpga_registers_parameters->Time_Hours & 0xffff;
    /*通道自检*/
    p_frame->f_parameter.Ch_Test = ptr_fpga_registers_parameters->Ch_Test & 0xffff;
    /*异常通道*/
    p_frame->f_parameter.Ch_Error = ptr_fpga_registers_parameters->Ch_Error & 0xffff;
    /*固定增益*/
    p_frame->f_parameter.TVG_Gain = ptr_fpga_registers_parameters->TVG_Gain;
    /*吸收*/
    p_frame->f_parameter.TVG_Absorb = ptr_fpga_registers_parameters->TVG_Absorb;
    /*扩散*/
    p_frame->f_parameter.TVG_Spread = ptr_fpga_registers_parameters->TVG_Spread;
    /*横摇稳定*/
    p_frame->f_parameter.Roll_St = ptr_fpga_registers_parameters->Roll_St;
    /*纵摇稳定*/
    p_frame->f_parameter.Pitch_St = ptr_fpga_registers_parameters->Pitch_St;
    /*Beamform方法*/
    p_frame->f_parameter.Beamform_Type = ptr_fpga_registers_parameters->Beamform_Type;
    /*质量滤波*/
    p_frame->f_parameter.Quality_Filter = ptr_fpga_registers_parameters->Quality_Filter;
    /*设备型号*/
    p_frame->f_parameter.Device_Type = ptr_fpga_registers_parameters->Device_Type;
    /*安装倾角*/
    p_frame->f_parameter.Install_Angle = ptr_fpga_registers_parameters->Install_Angle;
    /*盲区比例*/
    p_frame->f_parameter.Blind = ptr_fpga_registers_parameters->Blind;
    /*纵摇补偿*/
    p_frame->f_parameter.Pitch_Oe = ptr_fpga_registers_parameters->Pitch_Oe;
    /*中值滤波*/
    p_frame->f_parameter.Median_Filter = ptr_fpga_registers_parameters->Median_Filter;
    /*抽样后采样次数*/
    p_frame->f_parameter.Ad_Sn_After = ptr_fpga_registers_parameters->Ad_Sn_After;
    /*接收的roll值数组现有个数*/
    p_frame->f_parameter.Recv_SensorSumNum = ptr_fpga_registers_parameters->Recv_SensorSumNum;
    /*是否上传IQ */
    p_frame->f_parameter.up_iq = ptr_fpga_registers_parameters->up_iq;
    /*检测模式*/
    p_frame->f_parameter.math_mode = ptr_fpga_registers_parameters->math_mode;
    /*IQ抽样因子*/
    p_frame->f_parameter.iq_num = ptr_fpga_registers_parameters->iq_num;

    p_frame->f_parameter.side_width = ptr_fpga_registers_parameters->side_width; /* 侧扫宽度参数，来自显控配置解析后的FPGA参数镜像。 */
    p_frame->f_parameter.Sgram_num = ptr_fpga_registers_parameters->Sgram_num; /* 瀑布图/声图相关数量参数，保持旧配置含义。 */
    p_frame->f_parameter.mid_angle = ptr_fpga_registers_parameters->mid_angle; /* 中心角参数，供PL/DSP按新frame协议读取。 */
    p_frame->f_parameter.detection_mode = ptr_fpga_registers_parameters->detection_mode; /* 检测模式参数，只写一次，避免重复赋值造成维护误判。 */
    p_frame->f_parameter.kernel_num = ptr_fpga_registers_parameters->kernel_num; /* 检测核数量参数，按显控下发值透传。 */
    p_frame->f_parameter.BeamOA = 0;
    p_frame->f_parameter.BeamOA_k = 0;
    p_frame->f_parameter.Tao_Coe = ptr_fpga_registers_parameters->Tao_Coe; 

    /*结尾*/
    for (i = 0U; i < 6U; i++) /* memset不能按32bit填0xEEEEEEEE，所以这里逐word写入尾标记。 */
    {
        p_frame->f_parameter.end_tail[i] = S231_PARAM_END_MARK; /* 参数区末尾6个word固定为0xEEEEEEEE，供PL/DSP做边界检查。 */
    }

    p_frame->f_parameter.rsvd = 0U; /* 参数区最后1个word为保留字段，协议要求保持0。 */

    if (ptr_recv_upper_package != NULL) /* 显控配置包存在时，复制240字节异常通道表。 */
    {
        memcpy(p_frame->f_ch_err.ch_err, ptr_recv_upper_package->CH_error, sizeof(p_frame->f_ch_err.ch_err)); /* frame+272..511放CH_error。 */
    }

    memcpy(p_frame->f_roll_time_diff.roll_time_diff, Roll_Time_Diff, sizeof(p_frame->f_roll_time_diff.roll_time_diff)); /* frame+512..2111放400个Roll时间差。 */
    memcpy(p_frame->f_Roll_Sequential_Value.Roll_Sequential_Value, Roll_Sequential_Value, sizeof(p_frame->f_Roll_Sequential_Value.Roll_Sequential_Value)); /* frame+2112..3711放400个Roll序列值。 */
}


/*****************************************************************************
 * * description : 在FRAME_BASE(0x24000000)构造S231给DSP/PL使用的PS头。
 * * return      : OK表示PS写入区构造完成，FAIL表示frame映射或参数指针异常。
 * * Other       : 
 * *               1. frame+0..+11由PL生成，PS不能清零或写入；
 * *               2. frame+12..+3711由PS准备；
 * *               3. frame+3712及之后由PL/DMA生成，PS不能清零或写入。
 ******************************************************************************/
static int S231_BuildFrameHeader(void)
{
    unsigned char *frame = (unsigned char *)uio_frame.mem_ptr; /* 0x24000000映射后的ARM虚拟地址。 */
    frame_t frame_data; /* 本地临时frame_t，只描述PS需要写入frame+12..+3711的内容。 */

    if (S231_CheckMappedWindow("FRAME", uio_frame.mem_ptr, uio_frame.mem_size, S231_FRAME_PREFIX_BYTES) != OK) /* 检查0x24000000映射是否覆盖PS头区域。 */
    {
        return FAIL; /* frame映射无效时不能构造帧头。 */
    }

    if ((ptr_fpga_register_data == NULL) || (ptr_fpga_register_data == (FPGA_REGISTERS *)-1)) /* 参数区需要旧流程已解析好的FPGA参数。 */
    {
        DBG("FPGA register data pointer invalid\n"); /* 打印参数寄存器指针异常。 */
        return FAIL; /* 没有参数就不能构造S231参数区。 */
    }

    fill_frame(&ptr_fpga_register_data->fpga_registers_parameters, &frame_data); /* 先按frame_t结构统一构造PS侧参数、CH_error和Roll数据。 */

    /*
     * 一次写入完整的PS帧头区：
     * frame+12..+3711，共3700字节。
     * frame+60先按frame_t中的SIDE_RATIO正常写入，随后由PL在组帧时覆盖为active sound speed；
     * frame+64按frame_t中的SVP_TEST正常写入，PS不需要为frame+60做特殊分段处理。
     */
    memcpy(frame + S231_FRAME_PARAM_OFFSET, &frame_data, sizeof(frame_data));
    __sync_synchronize(); /* 确保frame头已经写入DDR后，再启动DMA/CBF读取。 */
    return OK; /* PS负责的frame+12..+3711区域构造完成。 */
}

/*****************************************************************************
 * * description : 清理NREAD回读DDR窗口。
 * * return      : OK表示清理完成，FAIL表示NREAD映射异常。
 * * Other       : 用0xA5填充便于调试时区分“DSP未写”和“DSP写了有效数据”。
 ******************************************************************************/
static int S231_ClearNreadBuffer(unsigned int read_bytes)
{
    if (S231_CheckMappedWindow("NREAD", uio_nread.mem_ptr, uio_nread.mem_size, read_bytes) != OK) /* 检查0x30000000映射是否覆盖本次回读长度。 */
    {
        return FAIL; /* NREAD映射无效时不能启动回读，避免后续显控读脏数据。 */
    }

    memset((unsigned char *)uio_nread.mem_ptr, 0xA5, read_bytes); /* 将本次可能被DSP回写的NREAD区域预填0xA5。 */
    __sync_synchronize(); /* 确保清理动作先于W2/NREAD启动生效。 */
    return OK; /* NREAD区域已准备完成。 */
}

/*****************************************************************************
 * * description : 检查S231关键硬件状态。
 * * return      : OK表示可以继续启动，FAIL表示当前硬件状态不适合启动本帧。
 * * Other       : BUILD_ID只告警不硬拦截，避免0x123/0x124调试阶段被软件锁死。
 ******************************************************************************/
static int S231_CheckHwReady(void)
{
    volatile ddr_cbf_t *cbf = ptr_ddr_cbf; /* CBF是MMIO寄存器窗口，使用volatile确保每次都真实读硬件。 */

    if ((cbf == NULL) || (cbf == (volatile ddr_cbf_t *)-1)) /* CBF寄存器必须先映射成功。 */
    {
        DBG("CBF register pointer invalid\n"); /* 打印CBF映射异常。 */
        return FAIL; /* CBF不可访问时不能启动PL。 */
    }

    if (cbf->MAGIC != S231_CBF_MAGIC_VALUE) /* MAGIC不对通常说明映射错地址或PL版本不匹配。 */
    {
        DBG("CBF MAGIC mismatch: 0x%08X\n", cbf->MAGIC); /* 打印硬件实际MAGIC值。 */
        return FAIL; /* MAGIC不匹配时硬启动风险太高，直接停止。 */
    }

    if (cbf->BUILD_ID != S231_CBF_BUILD_ID_VALUE) /* BUILD_ID不一致可能是0x123/0x124参考设计切换。 */
    {
        DBG("CBF BUILD_ID warning: 0x%08X expected 0x%08X\n", cbf->BUILD_ID, S231_CBF_BUILD_ID_VALUE); /* 只提示，不阻断本帧。 */
    }

    if (cbf->STATUS & S231_CBF_STATUS_BUSY) /* CBF仍忙时不能再次写START。 */
    {
        DBG("CBF is busy: status=0x%08X\n", cbf->STATUS); /* 打印状态，便于判断上一帧是否卡住。 */
        return FAIL; /* 忙状态下启动新帧会覆盖旧帧状态。 */
    }

    if (cbf->STATUS & S231_CBF_STATUS_ERROR) /* CBF处于错误状态时先尝试清错。 */
    {
        cbf->CONTROL = S231_CBF_CTRL_CLEAR_ERRORS; /* 写CLEAR_ERRORS，让硬件清除错误锁存位。 */
        usleep(1000); /* 等待1ms，让PL有时间完成清错动作。 */
    }

    if ((cbf->SR_LINK_STATUS & S231_SR_LINK_READY_MASK) != S231_SR_LINK_READY_MASK) /* 两路SRIO链路都ready后，W2/DSP交互才可靠。 */
    {
        DBG("SRIO link not ready: 0x%08X\n", cbf->SR_LINK_STATUS); /* 打印SRIO链路状态。 */
        return FAIL; /* SRIO未就绪时启动会导致DSP/NREAD失败。 */
    }

    return OK; /* CBF和SRIO状态满足启动条件。 */
}

/*****************************************************************************
 * * description : 配置W2 RapidIO/NREAD寄存器。
 * * return      : OK表示配置完成，FAIL表示W2映射异常。
 * * Other       : 这里写的是物理地址，不能写mmap后的虚拟地址。
 ******************************************************************************/
static int S231_ProgramW2(unsigned int frame_bytes, unsigned int read_bytes)
{
    volatile ddr_w2_t *w2 = ptr_ddr_w2; /* W2是MMIO寄存器窗口，使用volatile确保寄存器写不被优化。 */

    if (S231_CheckMappedWindow("W2", (void *)w2, uio_w2.mem_size, W2_SIZE) != OK) /* 检查W2寄存器映射是否有效。 */
    {
        return FAIL; /* W2不可访问时不能启动RapidIO/NREAD。 */
    }

    w2->SEND_BYTE_CNT_0 = frame_bytes; /* lane0发送长度写有效帧长度。 */
    w2->SENDR_BASE_ADDR_0 = DDR_FRAME_ADDR; /* lane0发送源地址为FRAME_BASE物理地址。 */
    w2->RECEIVE_BASE_ADDR_0 = DDR_NREAD_ADDR; /* lane0 NREAD回写地址为NREAD_BASE物理地址。 */
    w2->SEND_START_0 = 0U; /* 新S231流程由DDR_PR/CBF触发，这里保持旧SEND_START为0。 */
    w2->SEND_TARGET_ADDR_0 = S231_DSP_BANK0; /* lane0发送到DSP bank0。 */
    w2->SEND_BYTE_CNT_1 = frame_bytes; /* lane1发送长度同样写有效帧长度。 */
    w2->SENDR_BASE_ADDR_1 = DDR_FRAME_ADDR; /* lane1发送源地址同样为FRAME_BASE物理地址。 */
    w2->RECEIVE_BASE_ADDR_1 = DDR_NREAD_ADDR; /* lane1 NREAD回写地址同样为NREAD_BASE物理地址。 */
    w2->SEND_START_1 = 0U; /* 保持旧SEND_START为0，避免旧触发路径和新CBF路径冲突。 */
    w2->SEND_TARGET_ADDR_1 = S231_DSP_BANK1; /* lane1发送到DSP bank1。 */
    w2->READ_TARGET_ADDR_0 = S231_DSP_BANK0; /* lane0 NREAD从DSP bank0读取。 */
    w2->READ_TARGET_ADDR_1 = S231_DSP_BANK1; /* lane1 NREAD从DSP bank1读取。 */
    w2->READ_BYTE_CNT_0 = read_bytes; /* NREAD读取长度按21点/正常帧模式选择。 */
    w2->DDR_PR = 0U; /* 先拉低DDR_PR，形成一个干净的地址更新脉冲。 */
    w2->DDR_PR = 1U; /* 拉高DDR_PR，通知PL采样W2地址和长度配置。 */
    w2->DDR_PR = 0U; /* 再拉低DDR_PR，完成一次脉冲。 */
    __sync_synchronize(); /* 确保W2寄存器写入顺序在CBF START之前完成。 */
    return OK; /* W2配置完成。 */
}

/*****************************************************************************
 * * description : 配置CBF输入寄存器。
 * * return      : OK表示配置完成，FAIL表示CBF映射异常。
 * * Other       : INPUT_BASE写IQA/IQB物理地址，INPUT_BYTES写纯IQ声学数据长度。
 ******************************************************************************/
static int S231_ProgramCbf(unsigned int iq_phys, unsigned int iq_payload_len, unsigned int iq_points)
{
    volatile ddr_cbf_t *cbf = ptr_ddr_cbf; /* CBF是MMIO寄存器窗口，使用volatile确保写寄存器真实落到硬件。 */

    if (S231_CheckMappedWindow("CBF", (void *)cbf, uio_cfb.mem_size, CFB_SIZE) != OK) /* 检查CBF寄存器映射是否有效。 */
    {
        return FAIL; /* CBF不可访问时不能启动PL处理。 */
    }

    cbf->IQ_POINTS = iq_points; /* 写IQ点数，PL用它决定处理多少个832字节快照。 */
    cbf->ROLL_INTERVAL = iq_points + 1U; /* 参考程序使用points+1作为roll更新间隔。 */
    cbf->INPUT_BASE = iq_phys; /* 写IQA/IQB物理地址，PL从这里读取纯IQ输入。 */
    cbf->INPUT_BYTES = iq_payload_len; /* 写纯IQ输入长度，不包含128字节参数头和传感器数据。 */
    cbf->FRAME_ID = ptr_IQData_ParaHead_Last->PingCount; /* 写湿端上一ping帧号，便于PL日志和显控数据对齐。 */
    cbf->INPUT_FORMAT = S231_INPUT_FORMAT_WORD; /* 写S231约定的IQ输入格式。 */
    cbf->ROLL_SIGN = 0U; /* 当前先按参考程序默认方向写0，后续如PL要求再接入正负方向配置。 */
    __sync_synchronize(); /* 确保CBF配置寄存器都写完，再启动DMA和START。 */
    return OK; /* CBF输入配置完成。 */
}

/*****************************************************************************
 * 函数名称：S231_RadiansToQ29
 * 功能说明：将弧度浮点数转换为PL要求的有符号Q3.29格式。
 *
 * Q3.29的含义是：
 *   - 低29位表示小数；
 *   - 数值乘以2^29后，以32位有符号整数形式写入寄存器；
 *   - 寄存器本身按照unsigned int访问，但其位模式代表signed Q3.29。
 *
 * 该函数只负责数值转换，不直接访问硬件寄存器，便于后续单元测试和排查
 * 横摇单位问题。当前工程写入CFG前统一使用弧度作为输入单位。
 ******************************************************************************/
static int S231_RadiansToQ29(float radians, uint32_t *q29_value)
{
    double scaled_value; /* 保存弧度乘以2^29后的临时浮点结果，避免float中间计算精度不足。 */
    int32_t signed_value; /* Q3.29寄存器的有符号32位表示。 */

    if (q29_value == NULL) /* 输出地址为空时不能写入转换结果。 */
    {
        return FAIL; /* 返回失败，调用方不能继续写CFG寄存器。 */
    }

    if (!isfinite((double)radians)) /* NaN或无穷大都不能转换成有效的硬件定点数。 */
    {
        return FAIL; /* 拒绝非法传感器值，避免产生不可解释的Q29数据。 */
    }

    scaled_value = (double)radians * 536870912.0; /* 536870912等于2^29，完成Q3.29缩放。 */

    if (scaled_value > 2147483647.0) /* 超出有符号32位最大值时进行饱和，避免转换溢出。 */
    {
        signed_value = INT32_MAX; /* 将过大的正值限制到Q3.29可表示的最大值。 */
    }
    else if (scaled_value < -2147483648.0) /* 超出有符号32位最小值时同样进行饱和。 */
    {
        signed_value = INT32_MIN; /* 将过小的负值限制到Q3.29可表示的最小值。 */
    }
    else
    {
        signed_value = (int32_t)scaled_value; /* 在合法范围内截断小数部分，得到Q3.29整数位模式。 */
    }

    *q29_value = (uint32_t)signed_value; /* 保留signed Q3.29的32位补码位模式写入寄存器。 */
    return OK; /* 转换完成。 */
}

/*****************************************************************************
 * 函数名称：S231_ProgramCfgFrameParameters
 * 功能说明：逐帧写入当前版本PL已明确的CFG动态参数。
 *
 * 参考代码s231_cbf_smoke.c在每次启动前写入：
 *   CFG + 0x44：动态横摇Q3.29值；
 *   CFG + 0x48：横摇数据来源选择；
 *   CFG + 0x4C：动态横摇提交标志；
 *   CFG + 0x50：频率编码。
 *
 * 当前工程可以确定Roll_Value的业务来源，但显控参数中的PWMFreq是实际频率
 * 数值，而PL要求的frequency_code是20..40范围的编码，两者不能直接赋值。
 * 因此本函数暂不写frequency_code，避免把Hz数值误当编码下发。
 *
 * Sinc、Full Apod、Sub Apod表以及256点角度表也没有出现在当前湿端/显控数据
 * 结构中，不能用Beam_Num或起止角度随意伪造。它们暂时由PL已有配置保持，后续
 * 获得正式数据源后，必须在本函数中按参考代码的“数据、地址、WE”顺序逐帧写入。
 ******************************************************************************/
static int S231_ProgramCfgFrameParameters(void)
{
    volatile ddr_cfg_t *cfg = (volatile ddr_cfg_t *)ptr_ddr_cfg; /* CFG是AXI-Lite寄存器，必须使用volatile访问。 */
    uint32_t roll_q29 = 0U; /* 保存本帧动态横摇的Q3.29位模式。 */
    float roll_radians = 0.0f; /* 保存当前配置镜像中的弧度横摇值。 */

    if ((cfg == NULL) || (cfg == (volatile ddr_cfg_t *)-1)) /* 检查CFG映射是否有效。 */
    {
        DBG("CFG register pointer invalid\n"); /* 记录映射错误，防止空指针访问。 */
        return FAIL; /* CFG不可访问时不能启动本帧。 */
    }

    if (ptr_fpga_register_data != NULL) /* 参数镜像已经初始化时读取当前帧横摇。 */
    {
        pthread_mutex_lock(&mut2); /* 与显控参数更新线程共用同一把锁，避免读到更新一半的浮点值。 */
        if ((ptr_fpga_register_data->fpga_registers_parameters.Ins_Mod == 0U) &&
            (Recv_WetMemsSumNum > 0U) &&
            (Recv_WetMemsSumNum <= 400U)) /* 内置惯导路径只更新Roll_Sequential_Value[]，因此优先从数组取最新值并限制数组下标。 */
        {
            roll_radians = Roll_Sequential_Value[Recv_WetMemsSumNum - 1U] *
                           0.017453292519943295f; /* 内置惯导数组保存角度值，乘pi/180转换为弧度。 */
        }
        else
        {
            roll_radians = ptr_fpga_register_data->fpga_registers_parameters.Roll_Value; /* 外置惯导路径已经把平均横摇保存为弧度。 */
        }
        pthread_mutex_unlock(&mut2); /* 只保护软件参数快照，不把锁带入后续MMIO写入。 */
    }

    if (S231_RadiansToQ29(roll_radians, &roll_q29) != OK) /* 将弧度转换为PL要求的Q3.29。 */
    {
        DBG("invalid roll value: %f\n", roll_radians); /* 打印异常横摇值，便于定位传感器输入问题。 */
        return FAIL; /* 不把非法值写入CFG。 */
    }

    cfg->roll_update_q29 = roll_q29; /* 写入本帧动态横摇值，对应CFG偏移0x44。 */
    cfg->roll_update_source = 0U; /* 按参考代码选择默认横摇来源0，对应CFG偏移0x48。 */
    cfg->roll_update_push = 1U; /* 提交本帧动态横摇值，对应CFG偏移0x4C。 */
    __sync_synchronize(); /* 保证横摇数据、来源和提交标志按顺序到达PL。 */

    /*
     * 下面几类参数暂不写入：
     * 1. cfg->frequency_code：当前只有PWMFreq实际Hz，没有Hz到PL编码的映射表；
     * 2. CFG中的Sinc/Full Apod/Sub Apod：当前湿端协议没有携带这些完整表；
     * 3. CBF 0x100/0x104/0x108角度表：当前没有256点Q3.29角度表数据源。
     *
     * 这里故意不写0，也不把旧参数直接转换成新表，避免覆盖PL预加载的有效表。
     * 后续补齐数据源后，应在本函数或同一逐帧配置阶段中增加对应写入。
     */
    return OK; /* 当前已确认的CFG动态横摇参数配置完成。 */
}

/*****************************************************************************
 * 函数名称：S231_ReserveW3W4FrameParameters
 * 功能说明：为W3/W4逐帧配置预留统一入口，并检查两个映射窗口有效。
 *
 * 参考工程没有写W3/W4，当前PL文档也没有给出足够的W3/W4写入时序和字段
 * 所有权。当前结构体中的W3_EXT_STATUS、W4中的镜像/状态字段不能仅凭名称
 * 判定为PS可写寄存器。因此本函数暂时只做映射检查，不向W3/W4写猜测值。
 *
 * 等PL明确以下内容后，再在此函数中按每帧顺序补充：
 *   - W3哪些字段由PS写入；
 *   - W4哪些字段由PS写入；
 *   - 是否需要独立WE/COMMIT脉冲；
 *   - W3/W4数据是否在CBF START时锁存。
 ******************************************************************************/
static int S231_ReserveW3W4FrameParameters(void)
{
    if (S231_CheckMappedWindow("W3", (void *)ptr_ddr_w3, uio_w3.mem_size, W3_SIZE) != OK) /* 检查W3映射范围。 */
    {
        return FAIL; /* W3映射异常时禁止继续启动本帧。 */
    }

    if (S231_CheckMappedWindow("W4", (void *)ptr_ddr_w4, uio_w4.mem_size, W4_SIZE) != OK) /* 检查W4映射范围。 */
    {
        return FAIL; /* W4映射异常时禁止继续启动本帧。 */
    }

    /* 当前只保留接口，不写W3/W4，避免把状态寄存器或镜像寄存器误当作配置寄存器。 */
    return OK; /* W3/W4映射已确认，等待PL协议补充具体写入内容。 */
}

/*****************************************************************************
 * 函数名称：S231_ProgramPlFrameParameters
 * 功能说明：统一执行本帧启动前的PL参数配置阶段。
 *
 * 该函数必须位于IQ/Frame准备完成之后、CBF CONTROL.START之前。这样后续补充
 * Sinc、Apod、角度表和W3/W4写入时，不需要重新打散Set_Dsp_TransBuf()主流程。
 ******************************************************************************/
static int S231_ProgramPlFrameParameters(void)
{
    if (S231_ProgramCfgFrameParameters() != OK) /* 先写入当前已经明确的CFG动态参数。 */
    {
        return FAIL; /* CFG配置失败时不启动本帧。 */
    }

    if (S231_ReserveW3W4FrameParameters() != OK) /* 再确认预留的W3/W4窗口有效。 */
    {
        return FAIL; /* 映射窗口异常时不启动本帧。 */
    }

    __sync_synchronize(); /* 确保本阶段所有已写入的PL参数先于DMA和CBF启动生效。 */
    return OK; /* 本帧PL参数阶段完成。 */
}

/*****************************************************************************
 * * description : 等待AXI DMA S2MM复位位清零。
 * * return      : OK表示复位完成，FAIL表示超时。
 * * Other       : 只等待1秒，避免DMA异常时业务线程永久卡死。
 ******************************************************************************/
static int S231_WaitDmaResetClear(volatile ddr_dma_t *dma)
{
    unsigned int waited_us = 0U; /* 已经等待的时间，单位微秒。 */

    while (waited_us < S231_DMA_RESET_TIMEOUT_US) /* 按1秒上限循环等待DMA复位完成。 */
    {
        if ((dma->S2MM_DMACR & S231_DMA_S2MM_RESET) == 0U) /* RESET位由硬件清0后表示复位完成。 */
        {
            return OK; /* DMA复位完成。 */
        }

        usleep(1000); /* 每次等待1ms，避免忙等占满CPU。 */
        waited_us += 1000U; /* 累加已等待时间。 */
    }

    DBG("DMA reset timeout, DMACR=0x%08X DMASR=0x%08X\n", dma->S2MM_DMACR, dma->S2MM_DMASR); /* 打印DMA控制/状态寄存器。 */
    return FAIL; /* 超过1秒仍未清零，认为DMA异常。 */
}

/*****************************************************************************
 * * description : 配置AXI DMA S2MM SG描述符并启动DMA。
 * * return      : OK表示DMA已经arm，FAIL表示BD/DMA状态异常。
 * * Other       : DMA写入目标为FRAME_BASE，长度使用256对齐后的dma_bytes。
 ******************************************************************************/
static int S231_ArmDmaS2mm(unsigned int dma_bytes)
{
    ddr_bd_t *bd = ptr_ddr_bd; /* BD区域是普通DDR，AXI DMA会从这里读取SG描述符。 */
    volatile ddr_dma_t *dma = ptr_ddr_dma; /* DMA控制器是MMIO寄存器窗口，使用volatile访问。 */
    unsigned int count = 0U; /* 本次需要的BD描述符个数。 */
    unsigned int i = 0U; /* 循环变量，用于逐个填写BD。 */
    unsigned int offset = 0U; /* 当前BD对应FRAME_BASE内的目标偏移。 */
    unsigned int remaining = dma_bytes; /* 尚未分配到BD的DMA总长度。 */
    unsigned int dma_status = 0U; /* 保存DMA状态寄存器读值。 */
    unsigned int tail_phys = 0U; /* 最后一个BD的物理地址，用于写TAILDESC。 */

    if (S231_CheckMappedWindow("BD", (void *)bd, uio_bd.mem_size, S231_DMA_BD_BYTES) != OK) /* 检查BD窗口至少能放一个描述符。 */
    {
        return FAIL; /* BD不可访问时不能启动DMA。 */
    }

    if (S231_CheckMappedWindow("DMA", (void *)dma, uio_dma.mem_size, DMA_SIZE) != OK) /* 检查DMA寄存器窗口是否有效。 */
    {
        return FAIL; /* DMA寄存器不可访问时不能启动DMA。 */
    }

    if ((dma_bytes == 0U) || ((dma_bytes & 7U) != 0U)) /* AXI DMA SG BTT必须非0且按8字节对齐。 */
    {
        DBG("DMA bytes invalid: %u\n", dma_bytes); /* 打印非法DMA长度。 */
        return FAIL; /* DMA长度非法时不启动。 */
    }

    count = (dma_bytes + S231_DMA_MAX_SEGMENT - 1U) / S231_DMA_MAX_SEGMENT; /* 按单BD最大长度计算需要几个BD。 */
    if ((count == 0U) || (count * S231_DMA_BD_BYTES > (unsigned int)uio_bd.mem_size)) /* BD数量必须能放进0x3FF00000描述符窗口。 */
    {
        DBG("DMA BD count invalid: count=%u map=%d\n", count, uio_bd.mem_size); /* 打印BD数量和映射大小。 */
        return FAIL; /* BD窗口不足时不启动DMA。 */
    }

    memset(bd, 0, count * S231_DMA_BD_BYTES); /* 清空本次会用到的BD，避免上一帧状态位残留。 */

    for (i = 0U; i < count; i++) /* 逐个填写DMA SG描述符。 */
    {
        unsigned int segment = remaining > S231_DMA_MAX_SEGMENT ? S231_DMA_MAX_SEGMENT : remaining; /* 当前BD最多搬一个S231_DMA_MAX_SEGMENT分段。 */
        if (remaining > segment) /* 如果后面还有BD，当前BD长度必须保持8字节对齐。 */
        {
            segment &= ~7U; /* 非最后一段按8字节向下对齐，满足AXI DMA要求。 */
        }
        if (segment == 0U) /* 分段长度不能为0。 */
        {
            DBG("DMA segment is zero at bd=%u\n", i); /* 打印出错的BD序号。 */
            return FAIL; /* 分段异常时停止启动DMA。 */
        }
        bd[i].next_desc_low = DDR_BD_ADDR + (((i + 1U) % count) * S231_DMA_BD_BYTES); /* next指向下一个BD，最后一个回环到第一个BD。 */
        bd[i].next_desc_high = 0U; /* 当前系统物理地址在32bit范围内，高32bit写0。 */
        bd[i].buffer_addr_low = DDR_FRAME_ADDR + offset; /* 当前BD的S2MM写入目标地址为FRAME_BASE+offset。 */
        bd[i].buffer_addr_high = 0U; /* 当前系统物理地址在32bit范围内，高32bit写0。 */
        bd[i].control_btt = segment & S231_DMA_BD_BTT_MASK; /* control低23bit写本BD搬移长度。 */
        bd[i].status = 0U; /* status清0，DMA完成后会由硬件回写完成/错误状态。 */
        offset += segment; /* 累加已经分配的目标偏移。 */
        remaining -= segment; /* 扣掉本BD负责的传输长度。 */
    }

    if ((remaining != 0U) || (offset != dma_bytes)) /* 所有BD分段总和必须精确等于dma_bytes。 */
    {
        DBG("DMA split mismatch: remaining=%u offset=%u total=%u\n", remaining, offset, dma_bytes); /* 打印分段校验信息。 */
        return FAIL; /* 分段长度不一致时不启动DMA。 */
    }

    __sync_synchronize(); /* 确保BD内容写入DDR后，再让DMA控制器读取BD。 */
    dma->S2MM_DMACR = S231_DMA_S2MM_RESET; /* 复位S2MM通道，清掉上一帧状态。 */
    if (S231_WaitDmaResetClear(dma) != OK) /* 等待复位位清零。 */
    {
        return FAIL; /* DMA复位失败时不能继续启动。 */
    }

    dma_status = dma->S2MM_DMASR; /* 先保存清除前的原始状态，避免W1C操作把错误现场清掉。 */
    if (dma_status & S231_DMA_DMASR_ERROR_MASK) /* 先根据原始状态判断DMA当前是否已经存在错误。 */
    {
        DBG("DMA status error before start: 0x%08X\n", dma_status); /* 打印DMA错误状态。 */
        dma->S2MM_DMASR = S231_DMA_S2MM_IRQ_W1C; /* 错误现场已经保存后，再清除本帧残留的DMA中断/错误状态。 */
        return FAIL; /* 当前帧发现启动前DMA错误，放弃本帧启动，不影响后续帧重新尝试。 */
    }
    dma->S2MM_DMASR = S231_DMA_S2MM_IRQ_W1C; /* 原始状态确认无错误后，清除上一帧残留的IOC/ERR/DLY中断状态。 */

    dma->S2MM_CURDESC_L = DDR_BD_ADDR; /* 写当前BD链表首地址低32bit。 */
    dma->S2MM_CURDESC_H = 0U; /* 写当前BD链表首地址高32bit，当前平台为0。 */
    dma->S2MM_DMACR = S231_DMA_S2MM_RUNSTOP | S231_DMA_S2MM_IOC_IRQ_EN; /* 置RS启动DMA，同时打开完成中断。 */
    tail_phys = DDR_BD_ADDR + ((count - 1U) * S231_DMA_BD_BYTES); /* 计算最后一个BD物理地址。 */
    dma->S2MM_TAILDESC_L = tail_phys; /* 写TAILDESC低32bit，DMA开始拉取BD链。 */
    dma->S2MM_TAILDESC_H = 0U; /* 写TAILDESC高32bit，当前平台为0。 */
    __sync_synchronize(); /* 确保DMA启动寄存器写完后再返回。 */
    return OK; /* DMA已经arm，等待CBF START产生S2MM数据流。 */
}

/*****************************************************************************
 * * description : 准备CBF/NREAD中断。
 * * return      : OK表示中断已清除并重新使能，FAIL表示寄存器映射异常。
 * * Other       : 
 ******************************************************************************/
/*
 * 等待当前帧的AXI DMA S2MM传输真正完成。
 *
 * S231_ArmDmaS2mm()只负责填写BD并让DMA进入运行状态；CBF还没有START
 * 之前不会产生有效的S2MM数据，所以本函数必须在写入CBF CONTROL.START
 * 之后调用。这里等待的是CBF生成的Frame已经完整搬移到FRAME DDR，
 * 不等同于CBF、DSP和NREAD整个业务链路都已经完成。
 */
static int S231_WaitDmaS2mm(unsigned int dma_bytes)
{
    volatile ddr_bd_t *bd = ptr_ddr_bd; /* BD由DMA硬件回写，必须按volatile读取状态。 */
    volatile ddr_dma_t *dma = ptr_ddr_dma; /* DMA状态寄存器是MMIO，必须按volatile访问。 */
    unsigned int count = 0U; /* 本帧使用的BD数量。 */
    unsigned int i = 0U; /* BD遍历下标。 */
    unsigned int waited_us = 0U; /* 已等待的时间，单位为微秒。 */

    if (dma_bytes == 0U) /* DMA长度为0时没有合法的传输可以等待。 */
    {
        DBG("DMA wait length is zero\n"); /* 打印非法长度，便于定位上游IQ长度计算问题。 */
        return FAIL; /* 拒绝继续访问BD。 */
    }

    if (S231_CheckMappedWindow("BD", (void *)bd, uio_bd.mem_size, S231_DMA_BD_BYTES) != OK) /* 检查BD映射至少可访问一个描述符。 */
    {
        return FAIL; /* BD映射异常时不能读取DMA完成状态。 */
    }

    if (S231_CheckMappedWindow("DMA", (void *)dma, uio_dma.mem_size, DMA_SIZE) != OK) /* 检查DMA寄存器窗口，避免空指针或越界访问。 */
    {
        return FAIL; /* DMA寄存器不可访问时直接返回。 */
    }

    count = (dma_bytes + S231_DMA_MAX_SEGMENT - 1U) / S231_DMA_MAX_SEGMENT; /* 使用和启动阶段相同的公式重新计算BD数量。 */
    if ((count == 0U) || (count * S231_DMA_BD_BYTES > (unsigned int)uio_bd.mem_size)) /* 确认本次BD数量没有超过映射窗口。 */
    {
        DBG("DMA wait BD count invalid: count=%u map=%d\n", count, uio_bd.mem_size); /* 打印BD数量和映射大小。 */
        return FAIL; /* 数量异常时不读取越界地址。 */
    }

    while (waited_us < S231_DMA_WAIT_TIMEOUT_US) /* 在限定时间内轮询DMA和BD状态。 */
    {
        unsigned int dma_status = dma->S2MM_DMASR; /* 先读取原始DMASR，不能先写W1C清掉错误信息。 */
        unsigned int complete = 1U; /* 只有所有BD完成时才保持为1。 */
        unsigned int transferred = 0U; /* 累加每个BD状态中的实际传输字节数。 */

        if ((dma_status & S231_DMA_DMASR_ERROR_MASK) != 0U) /* DMA状态出现错误位时立即终止等待。 */
        {
            DBG("DMA transfer error: DMASR=0x%08X\n", dma_status); /* 输出原始错误状态，便于区分DMA故障类型。 */
            return FAIL; /* 错误状态下不能把当前帧标记为可复用。 */
        }

        for (i = 0U; i < count; i++) /* 遍历本帧所有BD，检查硬件回写的状态。 */
        {
            unsigned int status = bd[i].status; /* 读取BD status，bit31表示完成，低位保存实际传输量。 */

            if ((status & S231_DMA_BD_STATUS_ERROR) != 0U) /* BD报告DMA内部、从机或解码错误。 */
            {
                DBG("DMA BD error: bd=%u status=0x%08X\n", i, status); /* 打印出错BD序号和状态字。 */
                return FAIL; /* 当前帧数据不完整，不能继续复用相关缓冲区。 */
            }

            if ((status & S231_DMA_BD_COMPLETE) == 0U) /* 只要有一个BD未完成，整帧就仍处于进行中。 */
            {
                complete = 0U; /* 标记本轮轮询尚未完成。 */
            }

            transferred += status & S231_DMA_BD_BTT_MASK; /* 累加BD状态中记录的实际传输字节数。 */
        }

        if ((complete != 0U) && (transferred + 255U >= dma_bytes)) /* 允许最后BD存在协议要求的256字节对齐填充。 */
        {
            DBG("DMA transfer complete: bd_count=%u bytes=%u dmasr=0x%08X\n", count, transferred, dma_status); /* 记录本帧DMA完成信息。 */
            return OK; /* DMA和全部BD均已完成。 */
        }

        usleep(2000); /* 每2ms轮询一次，避免忙等占满负责接收湿端数据的CPU线程。 */
        waited_us += 2000U; /* 累加等待时间，最终受1秒上限约束。 */
    }

    DBG("DMA transfer timeout: bd_count=%u dma_bytes=%u DMASR=0x%08X\n", count, dma_bytes, dma->S2MM_DMASR); /* 打印超时现场寄存器。 */
    return FAIL; /* 超时表示PL没有在规定时间产生完整S2MM数据。 */
}

static int S231_PrepareIrq(void)
{
    volatile ddr_cbf_t *cbf = ptr_ddr_cbf; /* CBF IRQ寄存器位于CBF寄存器窗口内。 */
    unsigned int one = 1U; /* UIO重新使能中断时需要向fd写入1。 */

    if (S231_CheckMappedWindow("CBF", (void *)cbf, uio_cfb.mem_size, CFB_SIZE) != OK) /* 检查CBF映射是否有效。 */
    {
        return FAIL; /* CBF不可访问时不能清中断。 */
    }

    cbf->IRQ_STATUS = 0xFFFFFFFFU; /* 写1清除所有已锁存的CBF/NREAD中断状态。 */
    cbf->IRQ_MASK = 1U; /* 打开PL约定的NREAD/CBF中断mask。 */

    /* CBF节点同时承担寄存器映射和中断，所以中断操作必须使用uio_cfb.fd。 */
    if (uio_cfb.fd < 0) /* 检查CBF中断文件描述符是否有效。 */
    {
        DBG("CBF UIO fd is invalid\n"); /* 无法重新使能中断时，不启动本帧。 */
        return FAIL; /* 将初始化错误返回给调用者。 */
    }
    if (write(uio_cfb.fd, &one, sizeof(one)) != (int)sizeof(one)) /* uio_pdrv_genirq要求写1重新打开中断。 */
    {
        DBG("UIO irq re-enable failed\n"); /* 记录中断重新使能失败。 */
        return FAIL; /* 中断未准备好时不能启动CBF。 */
    }

    __sync_synchronize(); /* 确保IRQ寄存器清除动作在CBF START之前完成。 */
    return OK; /* 中断状态已准备好。 */
}

/*****************************************************************************
 * * description : S231新流程的IQ输入、frame头、W2、CBF、DMA启动函数。
 * *               1. 在IQA(0x20000000)和IQB(0x22000000)之间乒乓切换；
 * *               2. 将IQData_Last + 128处的纯IQ数据拷贝到选中的DDR；
 * *               3. 在FRAME_BASE(0x24000000)写PS头；
 * *               4. 在CBF START之前逐帧配置CFG动态参数，并预留W3/W4参数阶段；
 * *               5. 配置W2/NREAD、CBF输入、AXI DMA SG；
 * *               6. 最后写CBF CONTROL.START启动PL处理。
 * * return       : 无。异常时直接return，本帧不触发PL。
 * * Other        : dry_to_upper()仍按旧线程逻辑处理，本函数只负责把输入送到PL并启动新链路。
 ******************************************************************************/
void Set_Dsp_TransBuf(void)
{
    UIO_CONFIG_PARAMETER *iq_uio = NULL; /* 指向本次要使用的IQA或IQB映射结构体。 */
    unsigned int iq_phys = 0U; /* 本次IQA/IQB的物理地址，写给CBF INPUT_BASE。 */
    unsigned int next_flag = 1U; /* 本帧成功启动后写回Bd_Date_TransBuf_Flag，用于下次乒乓切换。 */
    unsigned int iq_payload_len = 0U; /* 本次IQ有效载荷长度，来自RECV_WET_SONAR_FIRST.IQ_Data_Length。 */
    unsigned char *iq_payload = NULL; /* 本次IQ有效载荷的ARM虚拟地址，只用于统一校验来源。 */
    unsigned int iq_points = 0U; /* IQ点数=IQ有效字节数/832。 */
    unsigned int frame_bytes = 0U; /* S231有效帧长度，用于W2发送长度。 */
    unsigned int dma_bytes = 0U; /* DMA按256字节对齐后的搬移长度，用于BD BTT。 */
    unsigned int beam_bytes = 0U; /* PL输出波束数据长度，用于frame+3712和旧全局变量。 */
    unsigned int read_bytes = 0U; /* DSP NREAD回读长度，用于W2 READ_BYTE_CNT。 */
    volatile ddr_cbf_t *cbf = ptr_ddr_cbf; /* CBF寄存器窗口，最后写CONTROL.START启动本帧。 */

    if (Bd_Date_TransBuf_Flag != 2) /* 当前不是“上次已用IQA”的状态时，本帧选择IQA。 */
    {
        iq_uio = &uio_iq_a; /* 本帧使用IQA映射窗口。 */
        iq_phys = DDR_IQ_A_ADDR; /* 本帧使用IQA物理地址0x20000000。 */
        next_flag = 2U; /* 本帧成功后，下次切到IQB。 */
    }
    else /* 当前上次使用了IQA时，本帧选择IQB。 */
    {
        iq_uio = &uio_iq_b; /* 本帧使用IQB映射窗口。 */
        iq_phys = DDR_IQ_B_ADDR; /* 本帧使用IQB物理地址0x22000000。 */
        next_flag = 1U; /* 本帧成功后，下次切回IQA。 */
    }

    if (Get_LastIqPayload(&iq_payload, &iq_payload_len) != OK) /* 统一取上一ping纯IQ地址和长度。 */
    {
        return; /* IQ来源异常时，本帧不下发。 */
    }

    iq_points = iq_payload_len / S231_IQ_BYTES_PER_POINT; /* 根据832字节/点换算IQ点数。 */
    if (S231_CalcFrameBytes(iq_points, &frame_bytes, &dma_bytes, &beam_bytes) != OK) /* 根据IQ点数计算frame/dma/beam长度。 */
    {
        return; /* 长度计算失败时，本帧不启动PL。 */
    }

    if (S231_GetNreadBytes(iq_points, frame_bytes, &read_bytes) != OK) /* 计算本帧DSP回读长度。 */
    {
        return; /* NREAD长度异常时，本帧不启动PL。 */
    }

    if (S231_CheckHwReady() != OK) /* 检查CBF MAGIC、忙状态、SRIO链路等关键硬件状态。 */
    {
        return; /* 硬件未准备好时，不写启动寄存器。 */
    }

    if (S231_CopyIqPayloadToDdr(*iq_uio) != OK) /* 将纯IQ声学数据写入当前IQA/IQB DDR输入区。 */
    {
        return; /* IQ拷贝失败时不能启动PL。 */
    }

    if (S231_BuildFrameHeader() != OK) /* 只在0x24000000构造PS负责的frame+12..+3711区域。 */
    {
        return; /* frame头构造失败时不能启动PL/DMA。 */
    }

    if (S231_ClearNreadBuffer(read_bytes) != OK) /* 清理本帧DSP NREAD可能写入的DDR区域。 */
    {
        return; /* NREAD区域不可用时不能启动PL。 */
    }

    if (S231_ProgramPlFrameParameters() != OK) /* 在W2/DMA/CBF START前完成本帧PL参数配置。 */
    {
        return; /* PL参数配置失败时禁止启动本帧，避免使用半套参数。 */
    }

    if (S231_ProgramW2(frame_bytes, read_bytes) != OK) /* 配置W2发送源、目标、NREAD目标和读取长度。 */
    {
        return; /* W2配置失败时不能启动PL。 */
    }

    if (S231_ProgramCbf(iq_phys, iq_payload_len, iq_points) != OK) /* 配置CBF输入地址、长度、点数、帧号和格式。 */
    {
        return; /* CBF配置失败时不能启动PL。 */
    }

    if (S231_ArmDmaS2mm(dma_bytes) != OK) /* 配置并启动AXI DMA S2MM SG链表。 */
    {
        return; /* DMA未arm成功时不能启动CBF，否则输出流没人接。 */
    }

    usleep(20000); /* 参考程序保留20ms，让DMA有时间完成BD预取。 */
    if (S231_PrepareIrq() != OK) /* 清除并重新使能CBF/NREAD中断。 */
    {
        return; /* 中断寄存器不可访问时，不启动本帧。 */
    }

    cbf->CONTROL = S231_CBF_CTRL_START; /* 最后写START，正式触发PL读取IQ、生成帧、启动DSP/NREAD流程。 */

    if (S231_WaitDmaS2mm(dma_bytes) != OK) /* CBF启动后等待本帧Frame通过S2MM DMA完整写入DDR。 */
    {
        DBG("S231 DMA completion wait failed, keep IQ bank=%d\n", Bd_Date_TransBuf_Flag); /* DMA失败时保留原乒乓状态，避免误认为当前缓冲区已经安全释放。 */
        return; /* 当前帧失败，暂不继续执行本帧完成和乒乓切换逻辑。 */
    }

    Bd_Date_TransBuf_Flag = (int)next_flag; /* DMA确认完成后才切换IQA/IQB，避免下一帧覆盖仍在搬运的输入区。 */
    DBG("S231 started: iq_points=%u iq_bytes=%u frame=%u dma=%u read=%u iq_phys=0x%08X\n", iq_points, iq_payload_len, frame_bytes, dma_bytes, read_bytes, iq_phys); /* 打印本帧关键长度和IQ物理地址。 */
}

#if 0
void Copy_AllDataToFpga(UIO_CONFIG_PARAMETER struct_uio_ddr)
{
    int total_len_iqallsize = 0;
    char rapidio_head[4] = { '@','@','@','@' };
    char rapidio_tail[2] = { '$','$' };
    char SendUpperHead[4] = { '<','<','S','T' };
    int pingnum = 0;
  
    memcpy((unsigned char*)(struct_uio_ddr.mem_ptr), rapidio_head, 4);//@@@@
	memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 4), &All_Rapidio_SendToDSP_Size, 4);//帧长
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 8), &BeamDataTotalSize, 4);//波束数据总长度（含参数头）
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA ),ptr_recv_upper_package->CH_error,240);
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240), Roll_Time_Diff, 1600);
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240 + 1600), Roll_Sequential_Value, 1600);
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240 + 1600 + 1600), &BeamDataSize, 4);
//	DBG("BeamDataSize = %d\n",BeamDataSize);
    total_len_iqallsize = IQDataTotalSize_Last + 8;//包含了<<ST + 长度 + IQData + ED>>
//	DBG("total_len_iqallsize = %d\n",total_len_iqallsize);
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240 + 1600 + 1600 + 4 + BeamDataSize), &total_len_iqallsize, 4);
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240 + 1600 + 1600 + 4 + BeamDataSize + 4), SendUpperHead, 4);
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240 + 1600 + 1600 + 4 + BeamDataSize + 4 + 4), &IQDataTotalSize_Last, 4);
    my_copy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240 + 1600 + 1600 + 4 + BeamDataSize + 4 + 4 + 4), IQData_Last, IQDataTotalSize_Last);
    memcpy((unsigned char*)((unsigned char*)(struct_uio_ddr.mem_ptr) + 12 + SIZE_OF_RAPIDIO_FIRST_FPGA + 240 + 1600 + 1600 + 4 + BeamDataSize + 4 + 4 + 4 + IQDataTotalSize_Last), rapidio_tail, 2); 

}

/*****************************************************************************
 * * description : 配置读写rapidio地址,目前是3个buff轮发,每个buf最大204M大小
 * * return       {*}
 * * Date        : 2022-10-20 11:11:20
 * * Other        目前硬件湿2X SRIO,其中LAN1作为写数据总线,LAN0作为读数据总线
 ******************************************************************************/
void Set_Dsp_TransBuf(void)
{
    if (Bd_Date_TransBuf_Flag == 1)
    {
        //配置波束地址
        ptr_fpga_register_addr->BeamPackageHead_Addr = 0x19000000 + 12;//帧头+帧长度+波束数据长度
        ptr_fpga_register_addr->Beam_Addr1 = 0x19000000 + 12 + SIZE_OF_RAPIDIO_FIRST;//波束数据参数
        ptr_fpga_register_addr->Beam_Addr2 = 0x19000000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize / 3;
        ptr_fpga_register_addr->Beam_Addr3 = 0x19000000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize / 3 * 2;
        ptr_fpga_register_addr->IQ_Base_Addr = 0x19000000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize + 12 + SIZE_OF_WET_SONAR_FIRST;
        //配置rapidio 0 目前没有使用
        ptr_fpga_register_addr->Send_Byte_Cnt_0 = Rapidio1_Send_Size;//需要发送的数据长度
        ptr_fpga_register_addr->send_Base_Addr_0 = 0x19000000;
        ptr_fpga_register_addr->Send_Target_Addr_0 = 0x80000000;
        //配置rapidio 1
        ptr_fpga_register_addr->Send_Byte_Cnt_1 = Rapidio1_Send_Size + Rapidio2_Send_Size;
        ptr_fpga_register_addr->send_Base_Addr_1 = 0x19000000;
      //  ptr_fpga_register_addr->Send_Target_Addr_1 = 0x80000000;//dsp_srio高速总线写缓冲地址

        ptr_fpga_register_addr->Read_TargetAddr_0_Buffer1 = 0x0c000000;//dsp_srio高速总线读缓冲地址
        ptr_fpga_register_addr->Read_TargetAddr_0_Buffer2 = 0x0c000000;
        ptr_fpga_register_addr->Read_Byte_Cnt_0 = 0x40000;//3M数据
       // ptr_fpga_register_addr->Read_Byte_Cnt_0 = 0x100000;//3M数据
       //ptr_fpga_register_addr->Read_Byte_Cnt_0 = 0x300000;//3M数据

        //ptr_fpga_register_addr->Read_TargetAddr_0_Buffer1 = 0x0c020000;
        //ptr_fpga_register_addr->Read_TargetAddr_0_Buffer2 = 0x0c020000;
       

        //拷贝数据到ddr
        //printf("*******1*************\n");
        Copy_AllDataToFpga(uio_ddr_mem_IQ_1);            
        usleep(1000);
        Bd_Date_TransBuf_Flag = Bd_Date_TransBuf_Flag + 1;
        ptr_fpga_register_addr->Addr_irq = 0;
        usleep(1000);
        ptr_fpga_register_addr->Addr_irq = 1;
    }
    else if (Bd_Date_TransBuf_Flag == 2)
    {
        //配置波束地址
        ptr_fpga_register_addr->BeamPackageHead_Addr = 0x25C00000 + 12;
        ptr_fpga_register_addr->Beam_Addr1 = 0x25C00000 + 12 + SIZE_OF_RAPIDIO_FIRST;
        ptr_fpga_register_addr->Beam_Addr2 = 0x25C00000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize / 3;
        ptr_fpga_register_addr->Beam_Addr3 = 0x25C00000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize / 3 * 2;
        ptr_fpga_register_addr->IQ_Base_Addr = 0x25C00000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize + 12 + SIZE_OF_WET_SONAR_FIRST;
        //配置rapidio 0
        ptr_fpga_register_addr->Send_Byte_Cnt_0 = Rapidio1_Send_Size;
        ptr_fpga_register_addr->send_Base_Addr_0 = 0x25C00000;  
        ptr_fpga_register_addr->Send_Target_Addr_0 = 0x80000000;
        //配置rapidio 1
        ptr_fpga_register_addr->Send_Byte_Cnt_1 = Rapidio1_Send_Size + Rapidio2_Send_Size;
        ptr_fpga_register_addr->send_Base_Addr_1 = 0x25C00000;
     //   ptr_fpga_register_addr->Send_Target_Addr_1 = 0x80000000;

        //printf("*******2*************\n");
        Copy_AllDataToFpga(uio_ddr_mem_IQ_2);
        usleep(1000);
        Bd_Date_TransBuf_Flag = Bd_Date_TransBuf_Flag + 1;
        ptr_fpga_register_addr->Addr_irq = 0;
        usleep(1000);
        ptr_fpga_register_addr->Addr_irq = 1;
    }
    else if (Bd_Date_TransBuf_Flag == 3)
    {
        //配置波束地址
        ptr_fpga_register_addr->BeamPackageHead_Addr = 0x32800000 + 12;
        ptr_fpga_register_addr->Beam_Addr1 = 0x32800000 + 12 + SIZE_OF_RAPIDIO_FIRST;
        ptr_fpga_register_addr->Beam_Addr2 = 0x32800000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize / 3;
        ptr_fpga_register_addr->Beam_Addr3 = 0x32800000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize / 3 * 2;
        ptr_fpga_register_addr->IQ_Base_Addr = 0x32800000 + 12 + SIZE_OF_RAPIDIO_FIRST + BeamDataSize + 12 + SIZE_OF_WET_SONAR_FIRST;
        //配置rapidio 0
        ptr_fpga_register_addr->Send_Byte_Cnt_0 = Rapidio1_Send_Size;
        ptr_fpga_register_addr->send_Base_Addr_0 = 0x32800000;
        ptr_fpga_register_addr->Send_Target_Addr_0 = 0x80000000;
        //配置rapidio 1
        ptr_fpga_register_addr->Send_Byte_Cnt_1 = Rapidio1_Send_Size + Rapidio2_Send_Size;
        ptr_fpga_register_addr->send_Base_Addr_1 = 0x32800000;
     //   ptr_fpga_register_addr->Send_Target_Addr_1 = 0x80000000;

        //printf("*******3*************\n");                            
        Copy_AllDataToFpga(uio_ddr_mem_IQ_3);
        usleep(1000);
        Bd_Date_TransBuf_Flag = 1;
        ptr_fpga_register_addr->Addr_irq = 0;
        usleep(1000);
        ptr_fpga_register_addr->Addr_irq = 1;
    }
    else
    {
        ;//空语句
    }
}
#endif

/*****************************************************************************
 * * description : 波束下放内置传感器处理程序
 * * return       {*}
 * * Date        : 2022-10-20 13:10:35
 * * Other       : 当前IQdata中存放的是上1ping的MEMSdata.所以计算上1ping同步时间时,需要用当前ping的数据
 ******************************************************************************/
static void Beam_Down_Ins_Mode(void)
{

    if ((Recv_WetMemsSumNum + Recv_Wet_MemsSensorHead.GGA_ZDA_NUM) <= 400)
    {
		Get_SonarSyncTime_Ins(Mems_Data,Recv_Wet_MemsSensorHead.GGA_ZDA_NUM);
        Get_MotionTime_Roll_Ins(Mems_Data,Recv_Wet_MemsSensorHead.GGA_ZDA_NUM);
    }
    else if ((Recv_WetMemsSumNum + Recv_Wet_MemsSensorHead.GGA_ZDA_NUM) > 400)
    {
        float Temp_Save[400];
        if (Recv_Wet_MemsSensorHead.GGA_ZDA_NUM < 400)
        {
            memcpy(Temp_Save, ((unsigned int *)(Roll_Time_Diff )+ Recv_WetMemsSumNum + Recv_Wet_MemsSensorHead.GGA_ZDA_NUM - 400), (400 - Recv_Wet_MemsSensorHead.GGA_ZDA_NUM)*4); 
            memcpy(Roll_Time_Diff, Temp_Save, (400 - Recv_Wet_MemsSensorHead.GGA_ZDA_NUM)*4);

            memcpy(Temp_Save, ((unsigned int*)(Motion_Time_Sync)+Recv_WetMemsSumNum + Recv_Wet_MemsSensorHead.GGA_ZDA_NUM - 400), (400 - Recv_Wet_MemsSensorHead.GGA_ZDA_NUM) * 4);  
            memcpy(Motion_Time_Sync, Temp_Save, (400 - Recv_Wet_MemsSensorHead.GGA_ZDA_NUM) * 4);  

            memcpy(Temp_Save,((unsigned int*) (Roll_Sequential_Value) + Recv_WetMemsSumNum + Recv_Wet_MemsSensorHead.GGA_ZDA_NUM - 400), (400 - Recv_Wet_MemsSensorHead.GGA_ZDA_NUM) * 4);
            memcpy(Roll_Sequential_Value, Temp_Save, (400 - Recv_Wet_MemsSensorHead.GGA_ZDA_NUM)*4);

            Recv_WetMemsSumNum = 400 - Recv_Wet_MemsSensorHead.GGA_ZDA_NUM;//目前已有Roll值的个数
        }
        else
        {
            Recv_WetMemsSumNum = 0;
            Recv_WetMemsNum = Recv_Wet_MemsSensorHead.GGA_ZDA_NUM - 400;
        }
        Get_SonarSyncTime_Ins(Mems_Data,Recv_Wet_MemsSensorHead.GGA_ZDA_NUM);
        Get_MotionTime_Roll_Ins(Mems_Data,Recv_Wet_MemsSensorHead.GGA_ZDA_NUM);
    } 
    else
    {
        ;//空语句
    }
}


/*****************************************************************************
 * * description : 读取一包外置传感器GNGGA数据
 * * param        {char} *ptr_ext
 * * param        {int} offset
 * * return       {*}
 * * Date        : 2022-11-02 14:00:04
 * * Other
 ******************************************************************************/
static int Read_Ext_Gngga_OnePack_Data(char *ptr_ext, int offset)
{
 //   if (NULL != (ptr_Ext_Gpzda_Data = (EXT_GPZDA_DATA*)malloc(SIZE_OF_ONE_PING_SENSOR)))
    {
        memset(ptr_Ext_Gpzda_Data, 0L, SIZE_OF_ONE_PING_SENSOR);
		memcpy(ptr_Ext_Gpzda_Data, (char*)(ptr_ext + (offset * 256) + EXTSENSOR_DATA_PARA_LEN), SIZE_OF_ONE_PING_SENSOR);
        return OK;
    }
 //   else  return FAIL;
}


/*****************************************************************************
 * * description : 得到外置传感器ZPZDA的值
 * * return       {*}
 * * Date        : 2022-11-02 14:01:58
 * * Other
 ******************************************************************************/
char Get_Zpzda_Data_for_Ext_Sensor(void)
{
    int rd_extcnt = 0;
    unsigned char* pRdExt = NULL;
    unsigned char* ptr_Read_Ext = NULL;
    int val_cnt = 0;
    int Ins_Len = 0;
    char deal_head[20] = { 0 };
    int head_flag = 0;

    if (NULL == (ptr_Read_Ext = (unsigned char*)tmalloc(SIZE_OF_ONE_PING_SENSOR))){
		DBG("malloc zda ptr failed\r\n");
        return FAIL;
    }
    memset(ptr_Read_Ext, 0L, SIZE_OF_ONE_PING_SENSOR);
    pRdExt = (unsigned char*)ptr_Ext_Gpzda_Data;
    if(pRdExt == NULL){
        DBG("pRdExt is NULL\r\n");
        tfree(ptr_Read_Ext);
        return FAIL;
    }

/*   
    20251211:
        解决提取ZDA数据
*/
    //提取协议头，仅处理zda数据
    int head_idx = 0;
    unsigned char* p_head = pRdExt;
    unsigned char* pRdExt_end = pRdExt + SIZE_OF_ONE_PING_SENSOR;   //传感器数据256字节边界
    
    //读取前40字节（覆盖20有效字符）
    while(head_idx < 19 && p_head < pRdExt + 40){
        if( *p_head == '$' ){
            deal_head[head_idx++] = *p_head;
            p_head += 2;
            //继续读取直到 ， * ，提取完整协议头
            while(head_idx < 19 && *p_head != ',' && *p_head != 0x2A){
                deal_head[head_idx++] = *p_head;
                p_head += 2;
            }
            break;
        }
        p_head += 2;
    }
    deal_head[head_idx] = '\0';
    // printf("识别到ZDA头[%s],开始下一步解算\r\n",deal_head);
    if(strstr(deal_head,"ZDA") == NULL){
        // DBG("非 ZDA协议头[%s] 跳过解算\r\n",deal_head);
        tfree(ptr_Read_Ext);
        return FAIL;
    }

    pRdExt = p_head;
    

    while(pRdExt < pRdExt_end && val_cnt < 7){
        
        rd_extcnt = 0;
        //读取到 ， * 为止
        while(pRdExt < pRdExt_end && *pRdExt != ',' && *pRdExt != '*'){
            //防止溢出保护
            if(rd_extcnt >= (SIZE_OF_ONE_PING_SENSOR-1)){
                // DBG("ZDA 字段长度溢出\r\n");
                tfree(ptr_Read_Ext);
                return FAIL;
            }

            //字符复制
            *(char*)(ptr_Read_Ext + rd_extcnt) = *pRdExt;
            pRdExt += 2;
            rd_extcnt++;            
        }

        //字段字符串加 '\0'
        *(char*)(ptr_Read_Ext + rd_extcnt) = '\0';
        Ins_Len = rd_extcnt + 1;

        //按照ZDA协议填充字段
        switch (val_cnt)
        {
            case 0: 
                memcpy(ptr_Ext_Gpzda_Data->Ext_GpzdaHead, deal_head, head_idx+1); 
                // printf("填充Ext_GpzdaHead：%s\r\n",ptr_Ext_Gpzda_Data->Ext_GpzdaHead);
                break;
            case 1: 
                memcpy(ptr_Ext_Gpzda_Data->Ext_Zdatime, ptr_Read_Ext, Ins_Len); 
                // printf("填充Ext_Zdatime：%s\r\n",ptr_Ext_Gpzda_Data->Ext_Zdatime);
                break;
            case 2: 
                memcpy(ptr_Ext_Gpzda_Data->Ext_Day, ptr_Read_Ext, Ins_Len); 
                // printf("填充Ext_Day：%s\r\n",ptr_Ext_Gpzda_Data->Ext_Day);
                break;
            case 3: 
                memcpy(ptr_Ext_Gpzda_Data->Ext_Mounth, ptr_Read_Ext, Ins_Len); 
                // printf("填充Ext_Mounth：%s\r\n",ptr_Ext_Gpzda_Data->Ext_Mounth);
                break;
            case 4: 
                memcpy(ptr_Ext_Gpzda_Data->Ext_Year, ptr_Read_Ext, Ins_Len); 
                // printf("填充Ext_Year：%s\r\n",ptr_Ext_Gpzda_Data->Ext_Year);
                break;
            case 5: 
                memcpy(ptr_Ext_Gpzda_Data->Ext_Hour, ptr_Read_Ext, Ins_Len); 
                // printf("填充Ext_Hour：%s\r\n",ptr_Ext_Gpzda_Data->Ext_Hour);
                break;
            case 6: 
                memcpy(ptr_Ext_Gpzda_Data->Ext_Min, ptr_Read_Ext, Ins_Len); 
                // printf("填充Ext_Min：%s\r\n",ptr_Ext_Gpzda_Data->Ext_Min);
                break;
            default: 
                break;
        }

        //指针偏移 跳过当前逗号
        if(*pRdExt == ',') {
            pRdExt += 2;
        } 
        
        val_cnt++;

        //找到终止符 * 退出循环
        if(*pRdExt == 0x2A){
            // printf("找到ZDA终止符 结算结束\r\n");
            break;
        }

    }

    rd_extcnt = 0;
    val_cnt = 0;
    Ins_Len = 0;
    /* ptr_Read_Ext没有在解析过程中改变，当前值就是临时缓冲区首地址。 */
    tfree(ptr_Read_Ext);
	ptr_Read_Ext = NULL;
	pRdExt = NULL;
	top = -1;
    return OK;
}


/*****************************************************************************
 * * description : 获取外置传感器的同步时间
 * * param        {char*} ptr_ext
 * * param        {int} cnt
 * * return       {*}
 * * Date        : 2022-11-02 14:02:30
 * * Other
 ******************************************************************************/
static void Get_SonarSyncTime_Ext(char* ptr_ext,int cnt)
{
    static float  Last_Zda_Time = 0;
    unsigned int ExtSensor_PPS_Num = 0;

    if (NULL == (ptr_Ext_Gpzda_Data = (EXT_GPZDA_DATA*)my_malloc(SIZE_OF_ONE_PING_SENSOR)))
    	return FAIL;
    // printf("===============================================================================================================\r\n");
    // printf("ZDA数据总条数：(IQdate->帧计数-1 = SensorDate->帧计数)%d \r\n",cnt);
    // printf("IQdate->帧计数 = [%d]，帧计数-1=[%d]\n",IQData_ParaHead_Last.PingCount,IQData_ParaHead_Last.PingCount-1);
    // printf("---------------------------------------------------------------------------------------------------------------------1\r\n");
    for (Recv_ExtSensorNum = 0; Recv_ExtSensorNum < cnt; ++Recv_ExtSensorNum)
    {
        //传感器PPS秒计数
        ExtSensor_PPS_Num =   *(int*)((char*)ptr_ext + 8 + Recv_ExtSensorNum * 256);
        // printf("上一条ZDA传感器数据->PPS秒计数 ExtSensor_PPS_Num_Last = %d\r\n",ExtSensor_PPS_Num_Last);
        // printf("当前ZDA传感器数据->PPS秒计数 ExtSensor_PPS_Num = %d\r\n",ExtSensor_PPS_Num);
        
        if (ExtSensor_PPS_Num != ExtSensor_PPS_Num_Last)//当PPS跳变时，取跳变时的ZDA时间；50hz
        {
            // printf("PPS跳变 与上一次PPS值不同 重新获取ZDA时间信息\r\n");
            Read_Ext_Gngga_OnePack_Data(ptr_ext, Recv_ExtSensorNum);
            Get_Zpzda_Data_for_Ext_Sensor();
            if (strncmp((char*)ptr_Ext_Gpzda_Data->Ext_GpzdaHead, "$GPZDA", 7) == 0 | strncmp((char*)ptr_Ext_Gpzda_Data->Ext_GpzdaHead, "$GNZDA", 7) == 0 | strncmp((char*)ptr_Ext_Gpzda_Data->Ext_GpzdaHead, "$BDZDA", 7) == 0)
            {                
                strncpy(Ext_zda_time_hour, (char*)ptr_Ext_Gpzda_Data->Ext_Zdatime, 2);
                strncpy(Ext_zda_time_min, (char*)ptr_Ext_Gpzda_Data->Ext_Zdatime + 2, 2);
                strncpy(Ext_zda_time_sec, (char*)ptr_Ext_Gpzda_Data->Ext_Zdatime + 4, 6);
                Ext_Zda_Time = (float)(atoi(Ext_zda_time_hour) * 60 * 60 + atoi(Ext_zda_time_min) * 60) + strtof(Ext_zda_time_sec, &pExt_zda_sec);  
                
                ptr_fpga_register_data->fpga_registers_parameters.Time_Date = atoi((char*)ptr_Ext_Gpzda_Data->Ext_Day);
                ptr_fpga_register_data->fpga_registers_parameters.Time_Date |= (atoi((char*)ptr_Ext_Gpzda_Data->Ext_Mounth) << 8);
                ptr_fpga_register_data->fpga_registers_parameters.Time_Year = atoi((char*)ptr_Ext_Gpzda_Data->Ext_Year);
                // printf("重新计算zda时间Ext_Zda_Time%f=%d*60*60+%d*60+%f\r\n",Ext_Zda_Time,atoi(Ext_zda_time_hour),atoi(Ext_zda_time_min),strtof(Ext_zda_time_sec, &pExt_zda_sec));
                
                if (ExtSensor_PPS_Num_Last != 0)
                {
                    if (Ext_Zda_Time - Last_Zda_Time > 0.9 && Ext_Zda_Time - Last_Zda_Time < 1.1)
                    {
                        PPS_status = 1;
                    }
                    else
                    {
                        PPS_status = 0;
                    }
                }
                Last_Zda_Time = Ext_Zda_Time;
                ExtSensor_PPS_Num_Last = ExtSensor_PPS_Num;
            }else {
                // printf("没有找到正确的ZDA数据头:%s 使用上一次的ZDA时间:%f\r\n",(char*)ptr_Ext_Gpzda_Data->Ext_GpzdaHead,Ext_Zda_Time);
                
                Ext_Zda_Time = Ext_Zda_Time;
            }


        }
        //计算传感器同步时间
        Sonar_Sync_Time = Ext_Zda_Time + (float)IQData_ParaHead_Last.TimeStamp / 1000.0 + (float)IQData_ParaHead_Last.PPS_Number - (float)ExtSensor_PPS_Num;//声学同步时间 ,单位是S，精度mS
        // printf("声学同步时间：Sonar_Sync_Time:[%f] = Ext_Zda_Time:[%f] + (float)IQData_ParaHead_Last.TimeStamp / 1000.0:[%f] + (float)IQData_ParaHead_Last.PPS_Number:%f - (float)ExtSensor_PPS_Num:%f\r\n",Sonar_Sync_Time,Ext_Zda_Time,(float)IQData_ParaHead_Last.TimeStamp / 1000.0,(float)IQData_ParaHead_Last.PPS_Number,(float)ExtSensor_PPS_Num )
        sonar_stime = (float)IQData_ParaHead_Last.TimeStamp / 1000.0 + (float)IQData_ParaHead_Last.PPS_Number;
        // printf("sonar_stime:[%f] = (float)IQData_ParaHead_Last.TimeStamp / 1000.0:[%f] + (float)IQData_ParaHead_Last.PPS_Number:[%f]\r\n",sonar_stime,(float)IQData_ParaHead_Last.TimeStamp / 1000.0,(float)IQData_ParaHead_Last.PPS_Number);
        
        ptr_fpga_register_data->fpga_registers_parameters.Time_Sec = (Sonar_Sync_Time - (int)floor(Sonar_Sync_Time)) + (int)(floor(Sonar_Sync_Time)) % 60;
        ptr_fpga_register_data->fpga_registers_parameters.Time_Min = ((int)(floor(Sonar_Sync_Time)) % 3600 - ((int)(floor(Sonar_Sync_Time)) % 3600) % 60) / 60;
        ptr_fpga_register_data->fpga_registers_parameters.Time_Hours = ((int)(floor(Sonar_Sync_Time)) - (int)(floor(Sonar_Sync_Time)) % 3600) / 3600; 
       
        
    }

	ptr_Ext_Gpzda_Data = NULL;
	my_free(ptr_Ext_Gpzda_Data);
	top = -1;
}

int roll_flag = 0;
/*****************************************************************************
 * * description : 获取外部MOTION TIME 与 ROll值
 * * param        {char*} ptr_mes
 * * param        {unsigned int} cnt
 * * return       {*}
 * * Date        : 2022-11-02 14:02:50
 * * Other
 ******************************************************************************/
void Get_MotionTime_Roll_Ext(char* ptr_mes,unsigned int cnt)
{
    float Motion_Time;
    char ext_roll[6];
    int roll_pps_cnt = 0;
    int roll_pps_ms_cnt = 0;
    static last_pps_cnt = 0;
    int i;
    unsigned int j;
	float roll;
	unsigned short utemp;
	short stemp;
	memset(ext_roll,0,sizeof(ext_roll));

#if 0
/*
    打印完整HAEDINIG or 姿态
*/
	unsigned char c;
    printf("\n==============TSS1 256byte:============================\r\n");
    unsigned char* date = (unsigned char*)Ext_Tss1_Data;
    for(i=0;i<SIZE_OF_ONE_PING_SENSOR;i++){
        c = date[i];
        printf("%02x ",c);
    }
    printf("\n==========================================\n\n");
#endif
#if 0
    printf("\n==============HEADING 256byte=========================\r\n");
    date = (unsigned char*)Ext_Handing_Data;
    for(i=0;i<SIZE_OF_ONE_PING_SENSOR;i++){
        c = date[i];
        printf("%02x ",c);
    }
    printf("\n==========================================\n\n");
#endif

    for (Recv_ExtSensorNum = 0; Recv_ExtSensorNum < cnt; ++Recv_ExtSensorNum)
    {
        if (*(ptr_mes + 16 + Recv_ExtSensorNum * 256) == 'q')
        {
            memset(ext_roll,0,sizeof(ext_roll));
            ext_roll[0] = *(ptr_mes + 16 + Recv_ExtSensorNum * 256 + 26*2);
            ext_roll[1] = *(ptr_mes + 16 + Recv_ExtSensorNum * 256 + 27*2);
            memcpy(&utemp, ext_roll, sizeof(unsigned short));
            stemp = (short)(MYSWAP16(utemp));
            roll = (float)stemp*180.0*3.051758e-05;//1/2^15
            roll_pps_cnt = *((int*)(ptr_mes + Recv_ExtSensorNum * 256) + 2);
            roll_pps_ms_cnt = *((int*)(ptr_mes + Recv_ExtSensorNum * 256) + 3) / 100000;
            last_pps_cnt = roll_pps_cnt;
            Motion_Time = (float)roll_pps_ms_cnt/1000.0 + (float)roll_pps_cnt;
            Motion_Time_Sync[Recv_ExtSensor_SumNum] = Motion_Time;
            Roll_Sequential_Value[Recv_ExtSensor_SumNum] = roll;
            Recv_ExtSensor_SumNum = Recv_ExtSensor_SumNum + 1;
        }
        else if(*(ptr_mes + 16 + Recv_ExtSensorNum * 256) == ':')
        {
            memset(ext_roll,0,sizeof(ext_roll));
            for (i = 0; i < 5; ++i)
            {
                ext_roll[i] = *(ptr_mes + 16 + Recv_ExtSensorNum * 256 + 2 * 14 + i * 2 );
            }
            if (ext_roll[0] == 32)
            {
                ext_roll[0] = 43;//如果找到的是空格就给个“+”
            }
            roll = atof(ext_roll);
            roll_pps_cnt = *((int*)(ptr_mes + Recv_ExtSensorNum * 256) + 2);
            roll_pps_ms_cnt = *((int*)(ptr_mes + Recv_ExtSensorNum * 256) + 3) / 100000;
            last_pps_cnt = roll_pps_cnt;
            Motion_Time = (float)roll_pps_ms_cnt/1000.0 + (float)roll_pps_cnt;
            Motion_Time_Sync[Recv_ExtSensor_SumNum] = Motion_Time;
            Roll_Sequential_Value[Recv_ExtSensor_SumNum] = roll / 100.0;//* 3.14 / 180;
            Recv_ExtSensor_SumNum = Recv_ExtSensor_SumNum + 1;
        }
        else
            continue;
    }
    for (j = 1; j < Recv_ExtSensor_SumNum; j++)
    {
        Roll_Time_Diff[j] = Motion_Time_Sync[j] - sonar_stime;
		if(Roll_Time_Diff[j] > 0)
		{
			ptr_fpga_register_data->fpga_registers_parameters.Roll_Value = ((Roll_Sequential_Value[j-1]+Roll_Sequential_Value[j])/2)*3.14/180;  
		} 
    }
    ptr_fpga_register_data->fpga_registers_parameters.Recv_SensorSumNum = Recv_ExtSensor_SumNum;
}

/*****************************************************************************
 * * description : 波束下放外置传感器处理程序
 * * return       {*}
 * * Date        : 2022-10-31 11:27:45
 * * Other
 ******************************************************************************/
static void Beam_Down_Ext_Mode(void)
{
    Get_SonarSyncTime_Ext(Ext_Gpzda_Data,Sensor1_Num);
    if ((Recv_ExtSensor_SumNum + Sensor3_Num) < 400)
    {
        Get_MotionTime_Roll_Ext(Ext_Tss1_Data,Sensor3_Num); 
    }
	if ((Recv_ExtSensor_SumNum + Sensor3_Num) >= 400)
    {
        float Temp_Save[400];
        if (Sensor3_Num < 400)
        {
            memcpy(Temp_Save, ((unsigned int *)(Roll_Time_Diff )+ Recv_ExtSensor_SumNum + Sensor3_Num - 400), (400 - Sensor3_Num)*4); 
			memcpy(Roll_Time_Diff, Temp_Save, (400 - Sensor3_Num)*4);

            memcpy(Temp_Save, ((unsigned int*)(Motion_Time_Sync)+Recv_ExtSensor_SumNum + Sensor3_Num - 400), (400 - Sensor3_Num) * 4);  
            memcpy(Motion_Time_Sync, Temp_Save, (400 - Sensor3_Num) * 4);  

            memcpy(Temp_Save,((unsigned int*) (Roll_Sequential_Value) + Recv_ExtSensor_SumNum + Sensor3_Num - 400), (400 - Sensor3_Num) * 4);
            memcpy(Roll_Sequential_Value, Temp_Save, (400 - Sensor3_Num)*4);

            Recv_ExtSensor_SumNum = 400 - Sensor3_Num;//目前已有Roll值的个数
        }
        else
        {
            Recv_ExtSensor_SumNum = 0;
            Recv_ExtSensor_SumNum = Sensor3_Num - 400;
        }
        Get_MotionTime_Roll_Ext(Ext_Tss1_Data,Sensor3_Num);  

    } 
}

void Copy_IQ_Extsensordata(void)
{
	memcpy(&Sensor1_Num,(char *)IQData+SIZE_OF_WET_SONAR_FIRST+8+ptr_IQData_ParaHead->IQ_Data_Length+4,4);
    if (Sensor1_Num > 100 || Sensor1_Num < 0)
        Sensor1_Num = 0;
	memcpy(&Sensor2_Num,(char *)IQData+SIZE_OF_WET_SONAR_FIRST+8+ptr_IQData_ParaHead->IQ_Data_Length+4+SIZE_OF_LONG*2+Sensor1_Num*SIZE_OF_ONE_PING_SENSOR,4);//2400 4字节时间位置长度原来在一起，后来分开，所以有4字节空 该sensor2为航向
    if (Sensor2_Num > 200 || Sensor2_Num < 0)
        Sensor2_Num = 0;
    memcpy(&Sensor3_Num,(char *)IQData+SIZE_OF_WET_SONAR_FIRST+8+ptr_IQData_ParaHead->IQ_Data_Length+4+SIZE_OF_LONG*3+(Sensor1_Num+Sensor2_Num)*SIZE_OF_ONE_PING_SENSOR,4);
    if (Sensor3_Num > 200 || Sensor3_Num < 0)
        Sensor3_Num = 0;
    memcpy(&Sensor4_Num,(char *)IQData+SIZE_OF_WET_SONAR_FIRST+8+ptr_IQData_ParaHead->IQ_Data_Length+4+SIZE_OF_LONG*4+(Sensor1_Num+Sensor2_Num+Sensor3_Num)*SIZE_OF_ONE_PING_SENSOR,4);
    if (Sensor4_Num > 200 || Sensor4_Num < 0)
        Sensor4_Num = 0;
    {
        my_copy(Ext_Gpzda_Data, IQData + SIZE_OF_WET_SONAR_FIRST+8 +ptr_IQData_ParaHead->IQ_Data_Length+4+4, Sensor1_Num * SIZE_OF_ONE_PING_SENSOR);   
        usleep(10);
        my_copy(Ext_Handing_Data, IQData + SIZE_OF_WET_SONAR_FIRST+8 +ptr_IQData_ParaHead->IQ_Data_Length+4+4+(Sensor1_Num * SIZE_OF_ONE_PING_SENSOR)+4+4, Sensor2_Num * SIZE_OF_ONE_PING_SENSOR);   
        usleep(10);
        my_copy(Ext_Tss1_Data, IQData + SIZE_OF_WET_SONAR_FIRST+8 + ptr_IQData_ParaHead->IQ_Data_Length+4+4+4+((Sensor1_Num + Sensor2_Num) * SIZE_OF_ONE_PING_SENSOR) + 4+4,Sensor3_Num * SIZE_OF_ONE_PING_SENSOR);    
    }
} 

/*****************************************************************************
 * * description :波束下放数据处理函数
 * * return       {*}
 * * Date        : 2022-10-20 13:09:32
 * * Other
 ******************************************************************************/
void Beam_Down_Data_Manage(void)
{
	char SendUpperTail[6] = { '0','0','E','D','>','>' };

	gettimeofday (&tv2, NULL);
	if (Sonar_FristPing_flag == 0) //第一ping`
	{
		Sonar_FristPing_flag = 1;
		Set_BeamDown_Fpga_Para();
		Set_SoundSpeed_Value();
		// 该长度不包括<<ST及本身大小
		// IQDataTotalSize = IQDataSizeFromWetSend + 4 + ptr_send_to_upper_sensor->ExtSensorTotalSize + 4 + SIZE_OF_ONE_PING_SENSOR * Sensor4_Num;
		IQDataTotalSize = IQDataSizeFromWetSend;
		IQDataTotalSize_Last = IQDataTotalSize;
		memcpy(IQData + IQDataTotalSize - 6, SendUpperTail, 6);
		my_copy(IQData_Last, IQData, IQDataTotalSize_Last);//上1ping,此时已经压了2ping,湿端程序压了1ping
		Sensor1_Num = 0;
		Sensor2_Num = 0;
		Sensor3_Num = 0;
		Sensor4_Num = 0;
	}
	else
	{
        memcpy(&IQData_ParaHead_Last, IQData_Last, SIZE_OF_WET_SONAR_FIRST);//获取上1ping的PPS_CNT及PPS_NS_CNT
        memcpy(&IQData_ParaHead, IQData, SIZE_OF_WET_SONAR_FIRST);//获取当前ping的Ins_Mod
		if(ptr_recv_upper_package->INS_mod != Ins_Mod_State)//切惯导后清空同步buf
		{
			Ins_Mod_State = ptr_recv_upper_package->INS_mod;
			Change_InsModeState_ClearBuf();
		}
		Set_BeamDown_Fpga_Para();//coe svp angel
		memset(Ext_Gpzda_Data,0L,sizeof(Ext_Gpzda_Data));  
		memset(Ext_Tss1_Data,0L,sizeof(Ext_Tss1_Data));
		Copy_IQ_Extsensordata(); 
        if (Sensor1_Num > 0)
            GGAZDA_status =1;
        if (Sensor2_Num > 0)
            Heading_status = 1;
        if (Sensor3_Num > 0)
            TSS1_status = 1;
        if (Sensor4_Num > 0)
            SV_status = 1;                    
		Set_SoundSpeed_Value();
		Beam_Down_Ext_Mode();
        IQDataTotalSize = IQDataSizeFromWetSend;
        memcpy(IQData + IQDataTotalSize - 6, SendUpperTail, 6);//当前ping
		Set_Fpga_To_Dsp_Para();
        Set_Dsp_TransBuf();
        /*
         * S231新链路已经在Set_Dsp_TransBuf()内部通过CBF CONTROL.START启动PL处理。
         * 旧版IQDataUpdata会触发旧寄存器链路读取旧DDR地址，和S231的IQA/IQB+DMA流程不是同一套协议。
         * 这里保留旧代码但禁止编译，方便后续回退对比，同时避免现场新旧触发同时发生。
         */
#if 0
        ptr_fpga_register_data->IQDataUpdata = 0;
        usleep(1000);
        ptr_fpga_register_data->IQDataUpdata = 1;
#endif
	//	sem_post(&sem_UPPER);
		Sensor1_Num = 0;
        Sensor2_Num = 0;
        Sensor3_Num = 0;
        Sensor4_Num = 0;
       // Get_Sonar_Sync_Time_Flag = 0;
        IQDataTotalSize_Last = IQDataTotalSize;
        my_copy(IQData_Last, IQData, IQDataTotalSize_Last);
    }
}
