/*****************************************************************************
 * * Copyright (C) 2022 Beijing StarTest High-Tech Co., Ltd.
 * * 
 * * All rights reserved by Star Test, Inc.
 * * 
 * * FilePath     : armtofpga.c
 * * 
 * * Author       : WuCheng <wucheng@startest.net>
 * * 
 * * Date         : 2022-09-30 17:44:39
 * * 
 * * LastEditTime : 2022-10-20 16:48:41
 ******************************************************************************/
#include "stdio.h"
#include "main.h"
#include "armtofpga.h"
#include "armtodsp.h"
#include "uppertodry.h"

FPGA_REGISTERS fpga_register_data;//下发给FPGA的参数配置寄存器
FPGA_REGISTERS* ptr_fpga_register_data = NULL;


WET_FPGA_REGISTERS wet_fpga_register_data;//发射上移后给FPGA的参数配置寄存器
WET_FPGA_REGISTERS* ptr_wet_fpga_register_data = NULL;
char *ptr_mark_registers = NULL;

/********************************************************************************
 * 名称：         parsing_instructions_200k
 * 功能：         接收到的上位机数据转化为FPGA寄存器值
 * 入口参数：      *ptr_recv_upper_package：接收到的上位机的结构体值 *ptr_fpga_config_para：转化为FPGA的值
 * 出口参数：      正确为0，失败是-1
 *********************************************************************************/
int parsing_instructions_200k(const WET_RECV_UPPER_CONFIG_PARAMETERS *ptr_recv_upper_package, WET_FPGA_CONFIG_PARAMETERS *ptr_fpga_config_para)
{
    ptr_fpga_config_para->wsm_registers_value.wsm_mod = (ptr_recv_upper_package->WorkMode) | (ptr_recv_upper_package->LFMMode << 1) | (ptr_recv_upper_package->DataType << 2); 
    if (ptr_recv_upper_package->PingRate!=0)
    {
        ptr_fpga_config_para->wsm_registers_value.wsm_ct = (unsigned int)(FPGA_CLK_FREQUENCY / ptr_recv_upper_package->PingRate);
    }
    else
    {
        printf("revc upper PingRate is 0!\n");
        return -1;
    }
    ptr_fpga_config_para->adc_registers_value.adc_sct = (unsigned int)((FPGA_CLK_FREQUENCY / (ptr_recv_upper_package->SamplingRate*1000)) - 1);  
    ptr_fpga_config_para->adc_registers_value.adc_sn = (unsigned int)((unsigned int)(ptr_recv_upper_package->Range * 2 * ptr_recv_upper_package->SamplingRate*1000/ V_SOUND)/ SAMPLE_FACTOR_8 * SAMPLE_FACTOR_8-1);  
    //ptr_fpga_config_para->dac_registers_value.dac_sct = (unsigned int)(ptr_recv_upper_package->UpperConfigParameter.Range * 2 * FPGA_CLK_FREQUENCY / 1500 / ptr_recv_upper_package->TvgDataSize); 
    ptr_fpga_config_para->dac_registers_value.dac_sct = 99999; //1ms
    ptr_fpga_config_para->dac_registers_value.dac_sn  = (unsigned int)(ptr_recv_upper_package->Range * 2 * 1000 / V_SOUND);
    ptr_fpga_config_para->pwm_registers_value.pwm_pulse = (unsigned int)(ptr_recv_upper_package->PulseWidth * (FPGA_CLK_FREQUENCY / 1000000.0f));
    ptr_fpga_config_para->pwm_registers_value.TransGear = ptr_recv_upper_package->TransGear;	
    
	ptr_fpga_config_para->pwm_registers_value.svp_select = ptr_recv_upper_package->Svp_Select;
    //同步输入输出及传感器通道赋值，2025年5月21日16:31:01添加
	ptr_fpga_config_para->pwm_registers_value.gnss_in = ptr_recv_upper_package->gnss_in;
	ptr_fpga_config_para->pwm_registers_value.gnss_level = ptr_recv_upper_package->gnss_level;
    ptr_fpga_config_para->pwm_registers_value.heading_in = ptr_recv_upper_package->heading_in;
    ptr_fpga_config_para->pwm_registers_value.heading_level = ptr_recv_upper_package->heading_level;
    ptr_fpga_config_para->pwm_registers_value.motion_in = ptr_recv_upper_package->motion_in;
    ptr_fpga_config_para->pwm_registers_value.motion_level = ptr_recv_upper_package->motion_level;
    ptr_fpga_config_para->pwm_registers_value.pps_in = ptr_recv_upper_package->pps_in;
    ptr_fpga_config_para->pwm_registers_value.pps_level = ptr_recv_upper_package->pps_level;
    ptr_fpga_config_para->pwm_registers_value.svp_in = ptr_recv_upper_package->svp_in;
    ptr_fpga_config_para->pwm_registers_value.svp_level = ptr_recv_upper_package->svp_level;
    ptr_fpga_config_para->pwm_registers_value.fan_mode = ptr_recv_upper_package->fan_mode;
    ptr_fpga_config_para->pwm_registers_value.fan_speed = ptr_recv_upper_package->fan_speed;
    ptr_fpga_config_para->pwm_registers_value.sy_no = ptr_recv_upper_package->sy_no;
    ptr_fpga_config_para->pwm_registers_value.syncin_delay = ptr_recv_upper_package->syncin_delay;
    ptr_fpga_config_para->pwm_registers_value.syncin_mode = ptr_recv_upper_package->syncin_mode;
    ptr_fpga_config_para->pwm_registers_value.syncin_period = ptr_recv_upper_package->syncin_period;
    ptr_fpga_config_para->pwm_registers_value.syncin_pulse = ptr_recv_upper_package->syncin_pulse;
    ptr_fpga_config_para->pwm_registers_value.syncin_trig = ptr_recv_upper_package->syncin_trig;
    ptr_fpga_config_para->pwm_registers_value.syncout_after = ptr_recv_upper_package->syncout_after;
    ptr_fpga_config_para->pwm_registers_value.syncout_before = ptr_recv_upper_package->syncout_before;
    ptr_fpga_config_para->pwm_registers_value.syncout_mode = ptr_recv_upper_package->syncout_mode;
    ptr_fpga_config_para->pwm_registers_value.syncout_moment = ptr_recv_upper_package->syncout_moment;
    ptr_fpga_config_para->pwm_registers_value.syncout_pulse = ptr_recv_upper_package->syncout_pulse;
    ptr_fpga_config_para->pwm_registers_value.syncout_trig = ptr_recv_upper_package->syncout_trig;
    //功率系数赋值或者经过计算后赋值，公式待定
    ptr_fpga_config_para->pwm_registers_value.power_factor = ((unsigned int)(ptr_recv_upper_package->power_factor*10))<<16|(unsigned int)(ptr_recv_upper_package->power_factor *1.23/48/3.3*4096);

    //pwm 频率和采样率和量程，赋值，fpga回传使用
    ptr_fpga_config_para->pwm_registers_value.pwm_frequency = ptr_recv_upper_package->PWMFreq;
    ptr_fpga_config_para->pwm_registers_value.sample_rate = ptr_recv_upper_package->SamplingRate;
    ptr_fpga_config_para->pwm_registers_value.range = ptr_recv_upper_package->Range; 
    ptr_fpga_config_para->pwm_registers_value.bandwidth = ptr_recv_upper_package->PWMBandWidth; 
    //pwm开关
    ptr_fpga_config_para->pwm_registers_value.pwm_start = ptr_recv_upper_package->pwm_start;
    //抽样后的ad个数，留着fpga使用
    ptr_fpga_config_para->pwm_registers_value.AD_sn_after = (ptr_fpga_config_para->adc_registers_value.adc_sn + 1) / SAMPLE_FACTOR;
 //   ptr_fpga_config_para->pwm_registers_value.AD_sn_after = ((ptr_fpga_config_para->adc_registers_value.adc_sn + 1) / SAMPLE_FACTOR)/2;
    //    if ((ptr_recv_upper_package->PWMFreq == 200)||(ptr_recv_upper_package->PWMFreq == 400))
    //    {
    //        ptr_fpga_config_para->pwm_registers_value.AD_sn_after = (ptr_fpga_config_para->adc_registers_value.adc_sn + 1) / SAMPLE_FACTOR_8;
    //    }
    //    if ((ptr_recv_upper_package->PWMFreq > 200)&&(ptr_recv_upper_package->PWMFreq < 400)) 
    //    {
    //        ptr_fpga_config_para->pwm_registers_value.AD_sn_after = (ptr_fpga_config_para->adc_registers_value.adc_sn + 1) / SAMPLE_FACTOR;
    //    }
    //ad通道组数选择
    ptr_fpga_config_para->pwm_registers_value.adc_num_fpga = ptr_recv_upper_package->AD_NUM;
    //计算pitch使用的系数
    ptr_fpga_config_para->pwm_registers_value.phcoe =(int)((307200)*(ptr_recv_upper_package->PWMFreq));//-*256/24*2^8*100=-273066
    ptr_fpga_config_para->pwm_registers_value.final_coe = (int)(100000/ptr_recv_upper_package->PWMFreq);//3276800000
    //初始相位开关
    ptr_fpga_config_para->pwm_registers_value.pitch_oe = ptr_recv_upper_package->pitch_oe;

    if (ptr_recv_upper_package->WorkMode == 0) 
    {
        ptr_fpga_config_para->pwm_registers_value.pwm_bf = (unsigned int)((double)(ptr_recv_upper_package->PWMFreq*1000*N2_32) / FPGA_CLK_FREQUENCY); 
        ptr_fpga_config_para->pwm_registers_value.pwm_lfm=0;
    }
    if (ptr_recv_upper_package->WorkMode == 1) 
    {
        ptr_fpga_config_para->pwm_registers_value.pwm_lfm = (unsigned int)((PWM_LFM_FACTOR / ptr_recv_upper_package->PulseWidth)* ptr_recv_upper_package->PWMBandWidth);
        if (ptr_recv_upper_package->LFMMode == 0) 
        {
            ptr_fpga_config_para->pwm_registers_value.pwm_bf = (unsigned int)(N2_32 / (FPGA_CLK_FREQUENCY/1000) * (ptr_recv_upper_package->PWMFreq - ptr_recv_upper_package->PWMBandWidth / 2) - ptr_fpga_config_para->pwm_registers_value.pwm_lfm / 2/N2_23); 
        }
        else
        {
            ptr_fpga_config_para->pwm_registers_value.pwm_bf = (unsigned int)(N2_32 / (FPGA_CLK_FREQUENCY/1000) * (ptr_recv_upper_package->PWMFreq + ptr_recv_upper_package->PWMBandWidth /2)- ptr_fpga_config_para->pwm_registers_value.pwm_lfm / 2/ N2_23);
        }
    } 
	if(ptr_recv_upper_package->sensor_fh_AT == 'q'|| ptr_recv_upper_package->sensor_fh_HEADING == 'q')
	{
		//DBG("111111111111111111111111111111111\n");
		ptr_fpga_config_para->pwm_registers_value.sync_state = (16 | ptr_fpga_config_para->pwm_registers_value.sync_state);
	}
	else
	{
		//DBG("2222222222222222222222222222222222\n");
		ptr_fpga_config_para->pwm_registers_value.sync_state = (~16 & ptr_fpga_config_para->pwm_registers_value.sync_state);
	}
    return 0;
}

void config_sensor_baud_and_framehead(const WET_RECV_UPPER_CONFIG_PARAMETERS *ptr_recv_upper_package, WET_FPGA_REGISTERS* ptr_fpga_register_data)
{
    //Debug_pritf_sensor_baud_and_framehead(ptr_recv_upper_package);
    
	ptr_fpga_register_data->fpga_sensor.GGA_ZDA_BAUD = FPGA_CLK_FREQUENCY/ptr_recv_upper_package->sensor_boud_GGA_ZDA; //波特率38400
    
	ptr_fpga_register_data->fpga_sensor.HEADING_BAUD =  FPGA_CLK_FREQUENCY/ptr_recv_upper_package->sensor_boud_HEADING; 
	ptr_fpga_register_data->fpga_sensor.AT_BAUD= FPGA_CLK_FREQUENCY/ptr_recv_upper_package->sensor_boud_AT;
	ptr_fpga_register_data->fpga_sensor.SVT_BAUD= FPGA_CLK_FREQUENCY/ptr_recv_upper_package->sensor_boud_SVT;
	ptr_fpga_register_data->fpga_sensor.GGA_ZDA_FH =(unsigned int)ptr_recv_upper_package->sensor_fh_GGA_ZDA;//4724;//$G改成$
	if(ptr_recv_upper_package->sensor_fh_AT == 'q'||ptr_recv_upper_package->sensor_fh_HEADING == 'q')
	{
		ptr_fpga_register_data->fpga_registers_200k.pwm_registers_value.sync_state = (16 | ptr_fpga_register_data->fpga_registers_200k.pwm_registers_value.sync_state);
		ptr_fpga_register_data->fpga_sensor.HEADING_FH =(unsigned int) ptr_recv_upper_package->sensor_fh_HEADING; //0x4824;//$H改成$
		ptr_fpga_register_data->fpga_sensor.AT_FH=(unsigned int)ptr_recv_upper_package->sensor_fh_AT;//0x303A;//:0改成:
	}
	else
	{
		ptr_fpga_register_data->fpga_registers_200k.pwm_registers_value.sync_state = (~16 & ptr_fpga_register_data->fpga_registers_200k.pwm_registers_value.sync_state);
		ptr_fpga_register_data->fpga_sensor.HEADING_FH =(unsigned int) ptr_recv_upper_package->sensor_fh_HEADING; //0x4824;//$H改成$
		ptr_fpga_register_data->fpga_sensor.AT_FH=(unsigned int)ptr_recv_upper_package->sensor_fh_AT;//0x303A;//:0改成:
	}
	ptr_fpga_register_data->fpga_sensor.SVT_FH=(unsigned int)ptr_recv_upper_package->sensor_fh_SVT;//0x3120; //空格1 改成空格
	ptr_fpga_register_data->fpga_sensor.GGA_ZDA_FT =(unsigned int)(ptr_recv_upper_package->sensor_ft_GGA_ZDA[1]*0x100+ptr_recv_upper_package->sensor_ft_GGA_ZDA[0]);//4724;//$G改成$
	ptr_fpga_register_data->fpga_sensor.HEADING_FT = (unsigned int)(ptr_recv_upper_package->sensor_ft_HEADING[1]*0x100+ptr_recv_upper_package->sensor_ft_HEADING[0]); //0x4824;//$H改成$
	ptr_fpga_register_data->fpga_sensor.AT_FT=(unsigned int)(ptr_recv_upper_package->sensor_ft_AT[1]*0x100+ptr_recv_upper_package->sensor_ft_AT[0]);//0x303A;//:0改成:
	ptr_fpga_register_data->fpga_sensor.SVT_FT=(unsigned int)(ptr_recv_upper_package->sensor_ft_SVT[1]*0x100+ptr_recv_upper_package->sensor_ft_SVT[0]);//0x3120; //空格1 改成空格
    
    
}

void Debug_pritf_sensor_baud_and_framehead(WET_FPGA_CONFIG_SENSER_PARAMETERS *fpga_sensor_printf)
{
	Debug("GGA_ZDA_BAUD: %d\n", fpga_sensor_printf->GGA_ZDA_BAUD); 
	Debug("HEADING_BAUD: %d\n", fpga_sensor_printf->HEADING_BAUD); 
	Debug("GGA_ZDA_BAUD: %d\n", fpga_sensor_printf->AT_BAUD); 
	Debug("GGA_ZDA_BAUD: %d\n", fpga_sensor_printf->SVT_BAUD); 

	Debug("GGA_ZDA_FH: %c\n", fpga_sensor_printf->GGA_ZDA_FH); 
	Debug("HEADING_FH: %c\n", fpga_sensor_printf->HEADING_FH); 
	Debug("AT_FH     : %c\n", fpga_sensor_printf->AT_FH);
	Debug("SVT_FH    : %c\n", fpga_sensor_printf->SVT_FH);

	Debug("GGA_ZDA_FT: %x\n", fpga_sensor_printf->GGA_ZDA_FT); 
	Debug("HEADING_FT: %x\n", fpga_sensor_printf->HEADING_FT); 
	Debug("AT_FT     : %x\n", fpga_sensor_printf->AT_FT);
	Debug("SVT_FT    : %x\n", fpga_sensor_printf->SVT_FT);
}
/*****************************************************************************
 * * description : 接收到的上位机数据转化为FPGA寄存器值
 * * param        {RECV_UPPER_CONFIG_PARAMETERS*} ptr_recv_upper_package 接收到的上位机的结构体值
 * * param        {FPGA_REGISTERS*} ptr_fpga_config_para 转化为FPGA的值
 * * return       {*}
 * * Date        : 2022-10-09 14:57:54
 * * Other
 ******************************************************************************/
void parsing_upperpara_to_fpga(const RECV_UPPER_CONFIG_PARAMETERS* ptr_recv_upper_package, FPGA_REGISTERS* ptr_fpga_config_para)
{
    if (ptr_recv_upper_package->INS_mod == 0)//内置惯导
    {
        ptr_fpga_config_para->fpga_registers_parameters.Ins_Mod = (unsigned int)(ptr_fpga_config_para->fpga_registers_parameters.Ins_Mod & 0x00000000);
    }
    if (ptr_recv_upper_package->INS_mod == 1)
    {
        ptr_fpga_config_para->fpga_registers_parameters.Ins_Mod = (unsigned int)(ptr_fpga_config_para->fpga_registers_parameters.Ins_Mod & 0x00000000 | 0x00000001);
    }

    ptr_fpga_config_para->fpga_registers_parameters.Image_Work_Mode = (unsigned int)ptr_recv_upper_package->Image_show_mode;
  	//DBG("Image_Work_Mode = %d\n",(unsigned int)ptr_recv_upper_package->Image_show_mode);
  	//DBG("Image_Work_Mode = %d\n",ptr_fpga_config_para->fpga_registers_parameters.Image_Work_Mode);
    ptr_fpga_config_para->fpga_registers_parameters.Beam_Num = (unsigned int)ptr_recv_upper_package->Beam_Num;
    ptr_fpga_config_para->fpga_registers_parameters.Roll_Oe = (unsigned int)ptr_recv_upper_package->roll_com;
    ptr_fpga_config_para->fpga_registers_parameters.Focus = (unsigned int)ptr_recv_upper_package->Focus;
    ptr_fpga_config_para->fpga_registers_parameters.Base_Test = (unsigned int)ptr_recv_upper_package->Base_Test;
    ptr_fpga_config_para->fpga_registers_parameters.Sonar_Show = (unsigned int)ptr_recv_upper_package->Sonar_image;
    ptr_fpga_config_para->fpga_registers_parameters.Side_Mod = (unsigned int)ptr_recv_upper_package->Side_scan;
    ptr_fpga_config_para->fpga_registers_parameters.PTD_Mod = (unsigned int)ptr_recv_upper_package->TD_mode;
    ptr_fpga_config_para->fpga_registers_parameters.Thr_Con = (unsigned int)ptr_recv_upper_package->Threshold_control;
    ptr_fpga_config_para->fpga_registers_parameters.Water_Column = (unsigned int)ptr_recv_upper_package->Water_Column;
    ptr_fpga_config_para->fpga_registers_parameters.INT_Mod = (unsigned int)ptr_recv_upper_package->interpolation;
    ptr_fpga_config_para->fpga_registers_parameters.Water_Detection = (unsigned int)ptr_recv_upper_package->Water_Detection;
    ptr_fpga_config_para->fpga_registers_parameters.Beam_Type = (unsigned int)ptr_recv_upper_package->Beam_Type;
    ptr_fpga_config_para->fpga_registers_parameters.Up_Limit = (float)ptr_recv_upper_package->Treshold_upper;
    ptr_fpga_config_para->fpga_registers_parameters.Down_Limit = (float)ptr_recv_upper_package->Treshold_lower;
    ptr_fpga_config_para->fpga_registers_parameters.Angle_Limit = (float)ptr_recv_upper_package->Treshold_angle;
    ptr_fpga_config_para->fpga_registers_parameters.Sidelobe_Factor = (float)ptr_recv_upper_package->Sidelobe_Factor;
    ptr_fpga_config_para->fpga_registers_parameters.Image_W = (unsigned int)ptr_recv_upper_package->Image_width;
    ptr_fpga_config_para->fpga_registers_parameters.Image_H = (unsigned int)ptr_recv_upper_package->Image_heith;
    ptr_fpga_config_para->fpga_registers_parameters.Image_Rotio = (float)ptr_recv_upper_package->Image_ratio;
    ptr_fpga_config_para->fpga_registers_parameters.Side_Rotio = (float)ptr_recv_upper_package->Side_ratio;
    ptr_fpga_config_para->fpga_registers_parameters.Manual_SoundSpeed = (unsigned int)ptr_recv_upper_package->Manual_SoundSpeed;
    ptr_fpga_config_para->fpga_registers_parameters.Ctrl_Bow = (unsigned int)ptr_recv_upper_package->Bow_control;
    ptr_fpga_config_para->fpga_registers_parameters.Ctrl_Heave = (unsigned int)ptr_recv_upper_package->Heave_control;
    ptr_fpga_config_para->fpga_registers_parameters.Start_Angle = (float)ptr_recv_upper_package->Start_Angle;
    ptr_fpga_config_para->fpga_registers_parameters.Finish_Angle = (float)ptr_recv_upper_package->Finish_Angle;
    ptr_fpga_config_para->fpga_registers_parameters.Ch_Beam = (unsigned int)ptr_recv_upper_package->CH_beam;
    ptr_fpga_config_para->fpga_registers_parameters.Ch_Test = (unsigned int)ptr_recv_upper_package->CH_test;
 //   ptr_fpga_config_para->fpga_registers_parameters.Ch_Error = (unsigned int)ptr_recv_upper_package->CH_error;
    ptr_fpga_config_para->fpga_registers_parameters.TVG_Gain = (unsigned int)ptr_recv_upper_package->ManualGain;
    ptr_fpga_config_para->fpga_registers_parameters.TVG_Absorb = (unsigned int)ptr_recv_upper_package->AbsorbGainCoef;
    ptr_fpga_config_para->fpga_registers_parameters.TVG_Spread = (unsigned int)ptr_recv_upper_package->SpreadGainCoef;
    ptr_fpga_config_para->fpga_registers_parameters.Roll_St = (unsigned int)ptr_recv_upper_package->Roll_stability;
    ptr_fpga_config_para->fpga_registers_parameters.Pitch_St = (unsigned int)ptr_recv_upper_package->Pitch_stability;
    ptr_fpga_config_para->fpga_registers_parameters.Beamform_Type = (unsigned int)ptr_recv_upper_package->Beamform;
    ptr_fpga_config_para->fpga_registers_parameters.Quality_Filter = (unsigned int)ptr_recv_upper_package->quality_filter;
    ptr_fpga_config_para->fpga_registers_parameters.Device_Type = (unsigned int)ptr_recv_upper_package->device_type;
  //  ptr_fpga_config_para->fpga_registers_parameters.Device_Type = 1004;
    ptr_fpga_config_para->fpga_registers_parameters.Install_Angle = (float)ptr_recv_upper_package->ins_angle;
    ptr_fpga_config_para->fpga_registers_parameters.Blind = (float)ptr_recv_upper_package->Blind_area;
    ptr_fpga_config_para->fpga_registers_parameters.Pitch_Oe = (unsigned int)ptr_recv_upper_package->pitch_com;
    ptr_fpga_config_para->fpga_registers_parameters.Median_Filter = (unsigned int)ptr_recv_upper_package->Median_Filter;
    ptr_fpga_config_para->fpga_registers_parameters.Water_Column = (unsigned int)ptr_recv_upper_package->Water_Column;
 //   ptr_fpga_config_para->fpga_registers_parameters.Image_Work_Mode = (unsigned int)ptr_recv_upper_package->Image_show_mode;
   
	ptr_fpga_config_para->fpga_registers_parameters.up_iq = (unsigned int)ptr_recv_upper_package->up_iq;
    ptr_fpga_config_para->fpga_registers_parameters.math_mode = (unsigned int)ptr_recv_upper_package->math_mode;
    ptr_fpga_config_para->fpga_registers_parameters.iq_num = (unsigned int)ptr_recv_upper_package->iq_num;
    ptr_fpga_config_para->fpga_registers_parameters.side_width = (unsigned int)ptr_recv_upper_package->side_width;
    ptr_fpga_config_para->fpga_registers_parameters.Sgram_num = (unsigned int)ptr_recv_upper_package->Sgram_num;
    ptr_fpga_config_para->fpga_registers_parameters.mid_angle = (float)ptr_recv_upper_package->mid_angle;
    ptr_fpga_config_para->fpga_registers_parameters.detection_mode = (unsigned int)ptr_recv_upper_package->detection_mode;
    ptr_fpga_config_para->fpga_registers_parameters.kernel_num = (unsigned int)ptr_recv_upper_package->kernel_num;
 
    if (ptr_recv_upper_package->GPSBaud == 0)
        ptr_fpga_config_para->fpga_sensor.GGA_ZDA_BAUD = 0;
    else ptr_fpga_config_para->fpga_sensor.GGA_ZDA_BAUD = (unsigned int)(FPGA_CLK_FREQUENCY / ptr_recv_upper_package->GPSBaud);
    if (ptr_recv_upper_package->HEDBaud == 0)
        ptr_fpga_config_para->fpga_sensor.HEADING_BAUD = 0;
    else ptr_fpga_config_para->fpga_sensor.HEADING_BAUD = (unsigned int)(FPGA_CLK_FREQUENCY / ptr_recv_upper_package->HEDBaud);
    if (ptr_recv_upper_package->POSBaud == 0)
        ptr_fpga_config_para->fpga_sensor.AT_BAUD = 0;
    else ptr_fpga_config_para->fpga_sensor.AT_BAUD = (unsigned int)(FPGA_CLK_FREQUENCY / ptr_recv_upper_package->POSBaud);
    if (ptr_recv_upper_package->SVPBaud == 0)
        ptr_fpga_config_para->fpga_sensor.SVT_BAUD = 0;
    else ptr_fpga_config_para->fpga_sensor.SVT_BAUD = (unsigned int)(FPGA_CLK_FREQUENCY / ptr_recv_upper_package->SVPBaud);

    ptr_fpga_config_para->fpga_sensor.GNSS_protocol = (unsigned int)ptr_recv_upper_package->GNSS_protocol;
    ptr_fpga_config_para->fpga_sensor.HEADING_protocol = (unsigned int)ptr_recv_upper_package->HEADING_protocol;
    ptr_fpga_config_para->fpga_sensor.MOTION_protocol = (unsigned int)ptr_recv_upper_package->MOTION_protocol;
    ptr_fpga_config_para->fpga_sensor.SVP_protocol = (unsigned int)ptr_recv_upper_package->SVP_protocol;
    //mark_baudrate = ptr_recv_upper_package->mark_baud;
    /*波束形成参数及 读写rapidio总线开关参数配置,该配置只执行一次*/
    unsigned int  pulse_dely_l = 150;
    unsigned int  pulse_dely_h = 96;
    ptr_fpga_config_para->rsv_data.repat_neam = 32;
    ptr_fpga_config_para->rsv_data.beam_num = 16;
    ptr_fpga_config_para->rsv_data.pulse_dely = (pulse_dely_l & 0x0000ffff) | (((pulse_dely_h & 0xffff) << 16) & 0xffff0000);
    ptr_fpga_config_para->rsv_data.read_num = 31;
    /*
     * S231新链路不能在显控参数解析阶段直接启动旧RapidIO发送。
     * 真正启动时机必须放在beam_down.c:Set_Dsp_TransBuf()里面：
     * 等IQA/IQB输入、FRAME头、W2参数、CBF参数、DMA SG全部准备好后，
     * 再通过CBF CONTROL.START统一触发PL处理。
     */
    ptr_fpga_register_addr->Send_Start_0 = 0; /* 关闭lane0旧启动位，避免参数下发时误触发旧RapidIO发送。 */
    ptr_fpga_register_addr->Send_Start_1 = 0; /* 关闭lane1旧启动位，避免旧触发路径和S231的DDR_PR/CBF START路径冲突。 */
}

/********************************************************************************
 * 名称：                    Debug_pritf_convert_wet_fpga_parameter
 * 功能：                    打印发射上移后的发送给FPGA的发射寄存器的内容
 * 入口参数：            	 *ptr_fpga_config_para ARM下发给FPGA的值
 * 出口参数：            	 无
 *********************************************************************************/
void Debug_pritf_convert_wet_fpga_parameter(WET_FPGA_CONFIG_PARAMETERS *ptr_fpga_config_para )
{
    Debug("*********************wet para to dry fpga start******************************\n");
    Debug("wsm_mod: %d\n", ptr_fpga_config_para->wsm_registers_value.wsm_mod); 
    Debug("wsm_ct: %d\n", ptr_fpga_config_para->wsm_registers_value.wsm_ct); 
    Debug("adc_sct: %d\n", ptr_fpga_config_para->adc_registers_value.adc_sct); 
    Debug("adc_sn: %d\n", ptr_fpga_config_para->adc_registers_value.adc_sn);
    Debug("dac_sct: %d\n", ptr_fpga_config_para->dac_registers_value.dac_sct); 
    Debug("dac_sn: %d\n", ptr_fpga_config_para->dac_registers_value.dac_sn);  
    Debug("pwm_bf: %d\n", ptr_fpga_config_para->pwm_registers_value.pwm_bf); 
    Debug("pwm_lfm: %d\n", ptr_fpga_config_para->pwm_registers_value.pwm_lfm); 
//    Debug("pwm_ip[1]: %d\n", ptr_fpga_config_para->pwm_registers_value.pwm_ip[1]);
    Debug("pwm_pulse: %d\n", ptr_fpga_config_para->pwm_registers_value.pwm_pulse);
    Debug("pwm_start %d\n", ptr_fpga_config_para->pwm_registers_value.pwm_start); 
    Debug("sync_state %d\n", ptr_fpga_config_para->pwm_registers_value.sync_state); 
    Debug("sync_plustype %d\n", ptr_fpga_config_para->pwm_registers_value.sync_selec); 
    Debug("sync_delaytime %d\n", ptr_fpga_config_para->pwm_registers_value.sync_delaytime); 

    Debug("power_factor %d\n", ptr_fpga_config_para->pwm_registers_value.power_factor);
    Debug("pwm_frequency %d\n", ptr_fpga_config_para->pwm_registers_value.pwm_frequency);
    Debug("sample_rate %d\n", ptr_fpga_config_para->pwm_registers_value.sample_rate);
    Debug("range %d\n", ptr_fpga_config_para->pwm_registers_value.range);
    Debug("bandwidth %d\n", ptr_fpga_config_para->pwm_registers_value.bandwidth);
    Debug("AD_sn_after %d\n", ptr_fpga_config_para->pwm_registers_value.AD_sn_after);
    Debug("pitch_oe %d\n", ptr_fpga_config_para->pwm_registers_value.pitch_oe);
    Debug("phcoe %d\n", ptr_fpga_config_para->pwm_registers_value.phcoe);
    Debug("final_coe %d\n", ptr_fpga_config_para->pwm_registers_value.final_coe);

    Debug("sensor_config %d\n", ptr_fpga_config_para->pwm_registers_value.sensor_config);
    Debug("gnss_in %d\n", ptr_fpga_config_para->pwm_registers_value.gnss_in);
    Debug("svp_in %d\n", ptr_fpga_config_para->pwm_registers_value.svp_in);
    Debug("heading_in %d\n", ptr_fpga_config_para->pwm_registers_value.heading_in);
    Debug("motion_in %d\n", ptr_fpga_config_para->pwm_registers_value.motion_in);
    Debug("pps_in %d\n", ptr_fpga_config_para->pwm_registers_value.pps_in);
    Debug("gnss_level %d\n", ptr_fpga_config_para->pwm_registers_value.gnss_level);
    Debug("svp_level %d\n", ptr_fpga_config_para->pwm_registers_value.svp_level);
    Debug("heading_level %d\n", ptr_fpga_config_para->pwm_registers_value.heading_level);
    Debug("motion_level %d\n", ptr_fpga_config_para->pwm_registers_value.motion_level);
    Debug("pps_level %d\n", ptr_fpga_config_para->pwm_registers_value.pps_level);
    Debug("syncin_trig %d\n", ptr_fpga_config_para->pwm_registers_value.syncin_trig);
    Debug("syncin_mode %d\n", ptr_fpga_config_para->pwm_registers_value.syncin_mode);
    Debug("syncin_pulse %d\n", ptr_fpga_config_para->pwm_registers_value.syncin_pulse);
    Debug("syncin_period %d\n", ptr_fpga_config_para->pwm_registers_value.syncin_period);
    Debug("syncin_delay %d\n", ptr_fpga_config_para->pwm_registers_value.syncin_delay);
    Debug("syncout_trig %d\n", ptr_fpga_config_para->pwm_registers_value.syncout_trig);
    Debug("syncout_mode %d\n", ptr_fpga_config_para->pwm_registers_value.syncout_mode);
    Debug("syncout_pulse %d\n", ptr_fpga_config_para->pwm_registers_value.syncout_pulse);
    Debug("syncout_moment %d\n", ptr_fpga_config_para->pwm_registers_value.syncout_moment);
    Debug("syncout_before %d\n", ptr_fpga_config_para->pwm_registers_value.syncout_before);
    Debug("syncout_after %d\n", ptr_fpga_config_para->pwm_registers_value.syncout_after);
    Debug("sy_no %d\n", ptr_fpga_config_para->pwm_registers_value.sy_no);
    Debug("fan_mode %d\n", ptr_fpga_config_para->pwm_registers_value.fan_mode);
    Debug("fan_speed %d\n", ptr_fpga_config_para->pwm_registers_value.fan_speed);
}



/*****************************************************************************
 * * description : 打印FPGA的寄存器的内容
 * * param        {FPGA_REGISTERS*} ptr_fpga_config_para ARM下发给FPGA的值
 * * return       {*}
 * * Date        : 2022-10-09 14:58:40
 * * Other
 ******************************************************************************/
void Debug_pritf_convert_fpga_parameter(FPGA_REGISTERS* ptr_fpga_config_para)
{
    Debug("====================== dry para to dry fpga start=====================\n");
    Debug("ptr_fpga_register_data->Ins_Mod: %x\n", ptr_fpga_config_para->fpga_registers_parameters.Ins_Mod);
    Debug("ptr_fpga_register_data->Image_Work_Mode: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Image_Work_Mode);
    Debug("ptr_fpga_register_data->Beam_Num: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Beam_Num);
    //Debug("ptr_fpga_register_data->Beam_angle: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Beam_angle);
    Debug("ptr_fpga_register_data->Roll_Oe: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Roll_Oe);
    Debug("ptr_fpga_register_data->Focus: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Focus);
    Debug("ptr_fpga_register_data->Base_Test: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Base_Test);
    Debug("ptr_fpga_register_data->Sonar_Show: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Sonar_Show);
    Debug("ptr_fpga_register_data->Side_Mod: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Side_Mod);
    Debug("ptr_fpga_register_data->PTD_Mod: %d\n", ptr_fpga_config_para->fpga_registers_parameters.PTD_Mod);
    Debug("ptr_fpga_register_data->Thr_Con %d\n", ptr_fpga_config_para->fpga_registers_parameters.Thr_Con);
    Debug("ptr_fpga_register_data->Water_Column %d\n", ptr_fpga_config_para->fpga_registers_parameters.Water_Column);
    Debug("ptr_fpga_register_data->INT_Mod: %d\n", ptr_fpga_config_para->fpga_registers_parameters.INT_Mod);
    Debug("ptr_fpga_register_data->Water_Detection: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Water_Detection);
    Debug("ptr_fpga_register_data->Beam_Type: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Beam_Type);
    Debug("ptr_fpga_register_data->Up_Limit %f\n", ptr_fpga_config_para->fpga_registers_parameters.Up_Limit);
    Debug("ptr_fpga_register_data->Down_Limit %f\n", ptr_fpga_config_para->fpga_registers_parameters.Down_Limit);
    Debug("ptr_fpga_register_data->Angle_Limit %f\n", ptr_fpga_config_para->fpga_registers_parameters.Angle_Limit);
    Debug("ptr_fpga_register_data->Sidelobe_Factor %f\n", ptr_fpga_config_para->fpga_registers_parameters.Sidelobe_Factor);
    Debug("ptr_fpga_register_data->Image_W: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Image_W);
    Debug("ptr_fpga_register_data->Image_H: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Image_H);
    Debug("ptr_fpga_register_data->Image_Rotio: %f\n", ptr_fpga_config_para->fpga_registers_parameters.Image_Rotio);
    Debug("ptr_fpga_register_data->Side_Rotio: %f\n", ptr_fpga_config_para->fpga_registers_parameters.Side_Rotio);
    Debug("ptr_fpga_register_data->Manual_SoundSpeed: %f\n", ptr_fpga_config_para->fpga_registers_parameters.Manual_SoundSpeed);
  //  Debug("ptr_fpga_register_data->Beam_Angle: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Beam_Angle);
 //   Debug("ptr_fpga_register_data->Angle_k: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Angle_k);
    Debug("ptr_fpga_register_data->Data_Type: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Data_Type);
    Debug("ptr_fpga_register_data->Ctrl_Bow: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Ctrl_Bow);
    Debug("ptr_fpga_register_data->Ctrl_Heave: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Ctrl_Heave);
    Debug("ptr_fpga_register_data->Start_Angle: %f\n", ptr_fpga_config_para->fpga_registers_parameters.Start_Angle);
    Debug("ptr_fpga_register_data->Finish_Angle: %f\n", ptr_fpga_config_para->fpga_registers_parameters.Finish_Angle);
    Debug("ptr_fpga_register_data->Ch_Beam: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Ch_Beam);
    Debug("ptr_fpga_register_data->Ch_Test: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Ch_Test);
    Debug("ptr_fpga_register_data->Ch_Error: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Ch_Error);
    Debug("ptr_fpga_register_data->TVG_Gain: %d\n", ptr_fpga_config_para->fpga_registers_parameters.TVG_Gain);
    Debug("ptr_fpga_register_data->TVG_Absorb: %d\n", ptr_fpga_config_para->fpga_registers_parameters.TVG_Absorb);
    Debug("ptr_fpga_register_data->TVG_Spread: %d\n", ptr_fpga_config_para->fpga_registers_parameters.TVG_Spread);
    Debug("ptr_fpga_register_data->Roll_St: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Roll_St);
    Debug("ptr_fpga_register_data->Pitch_St: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Pitch_St);
    Debug("ptr_fpga_register_data->Beamform_Type: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Beamform_Type);
    Debug("ptr_fpga_register_data->Quality_Filter: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Quality_Filter);
    Debug("ptr_fpga_register_data->Device_Type: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Device_Type);
    Debug("ptr_fpga_register_data->Install_Angle: %f\n", ptr_fpga_config_para->fpga_registers_parameters.Install_Angle);
    Debug("ptr_fpga_register_data->Blind: %f\n", ptr_fpga_config_para->fpga_registers_parameters.Blind);
    Debug("ptr_fpga_register_data->Pitch_Oe: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Pitch_Oe);
    Debug("ptr_fpga_register_data->Median_Filter: %d\n", ptr_fpga_config_para->fpga_registers_parameters.Median_Filter);
    Debug("ptr_fpga_register_data->GGA_ZDA_BAUD: %d\n", ptr_fpga_config_para->fpga_sensor.GGA_ZDA_BAUD);
    Debug("ptr_fpga_register_data->HEADING_BAUD: %d\n", ptr_fpga_config_para->fpga_sensor.HEADING_BAUD);
    Debug("ptr_fpga_register_data->AT_BAUD: %d\n", ptr_fpga_config_para->fpga_sensor.AT_BAUD);
    Debug("ptr_fpga_register_data->SVT_BAUD: %d\n", ptr_fpga_config_para->fpga_sensor.SVT_BAUD);
    Debug("ptr_fpga_register_data->GNSS_protocol: %d\n", ptr_fpga_config_para->fpga_sensor.GNSS_protocol);
    Debug("ptr_fpga_register_data->HEADING_protocol: %d\n", ptr_fpga_config_para->fpga_sensor.HEADING_protocol);
    Debug("ptr_fpga_register_data->MOTION_protocol: %d\n", ptr_fpga_config_para->fpga_sensor.MOTION_protocol);
    Debug("ptr_fpga_register_data->SVP_protocol: %d\n", ptr_fpga_config_para->fpga_sensor.SVP_protocol);
    Debug("ptr_fpga_register_data->status: %d\n", ptr_fpga_config_para->fpga_sta.status);
    Debug("ptr_fpga_register_data->date: %d\n", ptr_fpga_config_para->fpga_sta.date);
    Debug("ptr_fpga_register_data->SYSTEM_STATUS: %d\n", ptr_fpga_config_para->fpga_sta.SYSTEM_STATUS);
    Debug("--------------------  dry para to dry fpga end -------------------- \n");
}
