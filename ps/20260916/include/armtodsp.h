#ifndef __ARMTODSP_H
#define __ARMTODSP_H
/****************************FPGA rapidio寄存器2 共128字节**********************/
typedef struct
{       
	unsigned int    Addr_irq;//地址更新中断
	unsigned int	ExtSensor_irq;//外置传感器中断
	unsigned int    Send_Byte_Cnt_0;//待发送的数据长度
    unsigned int    send_Base_Addr_0;//待发送数据的DDR首地址
    unsigned int    recive_Base_Addr_0;//接收到的数据的DDR存储地址
    unsigned int    Send_Start_0;//发送启动标志,置1发送
    unsigned int    Send_Target_Addr_0;//从机端数据存放地址
    unsigned int    Send_Byte_Cnt_1;//待发送的数据长度
    unsigned int    send_Base_Addr_1;//待发送数据的DDR首地址
    unsigned int    recive_Base_Addr_1;//接收到的数据的DDR
    unsigned int    Send_Start_1;//发送启动标志
    unsigned int    Send_Target_Addr_1;//从机端数据存放地址
    unsigned int    IQ_Base_Addr;//IQ数据地址
    unsigned int    Beam_Addr1;//波束数据1地址
    unsigned int    Beam_Addr2;//波束数据2地址
    unsigned int    Beam_Addr3;//波束数据3地址
    unsigned int    BeamPackageHead_Addr;//波束数据包头地址
    unsigned int    GGA_Buffer_Addr;//GGA_buffer地址
    unsigned int    Heading_Buffer_Addr;//Heading _buffer地址
    unsigned int    At_Buffer_Addr;//at_buffer基地址
    unsigned int    Svp_Buffer_Addr;//svp _buffer基地址
    unsigned int	Read_TargetAddr_0_Buffer1;
	unsigned int	Read_TargetAddr_0_Buffer2;
	unsigned int	Read_Byte_Cnt_0;
	unsigned int	Read_TargetAddr_1_Buffer1;
	unsigned int	Read_TargetAddr_1_Buffer2;
	unsigned int	Read_Byte_Cnt_1;
	unsigned int	ddr_read_flag;
	unsigned int    Res[5];
}FPGA_rapidio_REGISTERS;

extern FPGA_rapidio_REGISTERS fpga_register_addr;
extern FPGA_rapidio_REGISTERS* ptr_fpga_register_addr;

#endif
