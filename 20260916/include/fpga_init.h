#ifndef __FPGA_INIT_H
#define __FPGA_INIT_H

#include <stdint.h>

extern int fd;

#define   DDR_IQ_A_ADDR              0X20000000//DDR基地址IQ_buffera
#define   DDR_IQ_B_ADDR              0X22000000//DDR基地址IQ_bufferb
#define   DDR_FRAME_ADDR             0X24000000//DDR frame基地址 PL 输出
#define   DDR_NREAD_ADDR             0X30000000//DDR nread基地址 dsp数据处理结果寄存器
#define   DDR_BD_ADDR                0X3FF00000//DDR bd基地址    AXI DMA 描述符
#define   DDR_TVG_ADDR               0X40000000//DDR tvg地址
#define   DDR_DMA_ADDR               0X40400000//DDR dma基地址   DMA 控制寄存器
#define   DDR_FPGA_REGISTER_ADDR     0X43C00000//DDR fpga_register基地址
#define   DDR_CFB_ADDR               0X43C00400//DDR cfb基地址
#define   DDR_W2_ADDR                0x43C10000//DDR w2基地址
#define   DDR_CFG_ADDR               0x43C10400//DDR w1/cfg基地址
#define   DDR_WET_FPGA_REGISTER_ADDR 0X43C20000//DDR 湿端fpga寄存器地址
#define   DDR_W3_ADDR                0x43C20400//DDR w3基地址
#define   DDR_MAKE_FPGA_REGISTER_ADDR 0X43C30000U//DDR MAKE FPGA寄存器地址，设备树uio_make_fpga_register节点对应的基地址
/* 保留旧拼写，避免其他旧代码引用时产生编译错误；新代码应使用上面的规范宏。 */
#define   DDR_MAKE_FPGA_REFIGSTER     DDR_MAKE_FPGA_REGISTER_ADDR
#define   DDR_W4_ADDR                0x43C30400//DDR w4基地址

#define   IQ_A_SIZE                  (32U * 1024U * 1024U)//IQ_buffera大小 
#define   IQ_B_SIZE                  (32U * 1024U * 1024U)//IQ_bufferb大小
#define   FRAME_SIZE                 0x0BC00000U//FRAME大小
#define   NREAD_SIZE                 (32U * 1024U * 1024U)//NREAD大小
#define   S231_NREAD_RESULT_MAX_BYTES (4U * 1024U * 1024U)//DSP NREAD结果和Linux正式上传的固定上限为4MiB
#define   S231_NREAD_UPLOAD_CHUNK_BYTES (1U * 1024U * 1024U)//Linux向显控分块发送NREAD时使用1MiB分块
#define   BD_SIZE                    (64U * 1024U)//BD大小
#define   TVG_SIZE                   0x00004000U//TVG映射大小
#define   DMA_SIZE                   0x60//DMA大小
#define   FPGA_REGISTER_ADDR         0x400 //fpga ctrl 寄存器·
#define   CFB_SIZE                   0x00000400//CFB大小
#define   W2_SIZE                    0x00000400//W2大小
#define   CFG_SIZE                   0x00000400//CFG大小
#define   W3_SIZE                    0x00000400//W3大小
#define   W4_SIZE                    0x00000400//W4大小

/*
 * 同一4KB页内的逻辑寄存器窗口不能分别作为UIO节点匹配。
 * 下面定义每个物理页的整体映射长度，以及页内逻辑子窗口偏移。
 */
#define   FPGA_CFB_WINDOW_SIZE       0x00001000U
#define   W2_CFG_WINDOW_SIZE         0x00001000U
#define   WET_W3_WINDOW_SIZE         0x00001000U
#define   MAKE_W4_WINDOW_SIZE        0x00001000U
#define   CFB_WINDOW_OFFSET          (DDR_CFB_ADDR - DDR_FPGA_REGISTER_ADDR)
#define   CFG_WINDOW_OFFSET          (DDR_CFG_ADDR - DDR_W2_ADDR)
#define   W3_WINDOW_OFFSET           (DDR_W3_ADDR - DDR_WET_FPGA_REGISTER_ADDR)
#define   W4_WINDOW_OFFSET           (DDR_W4_ADDR - DDR_MAKE_FPGA_REGISTER_ADDR)

//MIO运行状态灯
#define SYSFS_GPIO_EXPORT                "/sys/class/gpio/export" 
#define SYSFS_GPIO_RUN_LIGHT             "962"                         
#define SYSFS_GPIO_RUN_LIGHT_DIR         "/sys/class/gpio/gpio962/direction"
#define SYSFS_GPIO_RUN_LIGHT_VAL         "/sys/class/gpio/gpio962/value"
//输入输出设置
#define SYSFS_GPIO_OUT                   "out" 
#define SYSFS_GPIO_IN                    "in"
//高低电平设置
#define SYSFS_GPIO_VAL_H                 "1"
#define SYSFS_GPIO_VAL_L                 "0"


/*
    BD 寄存器结构  0x3FF00000  AXI DMA 描述符 用来拆分传输dma
*/
typedef struct{
    unsigned int next_desc_low;
    unsigned int next_desc_high;
    unsigned int buffer_addr_low;
    unsigned int buffer_addr_high;
    unsigned int res[2];
    unsigned int control_btt;
    unsigned int status;
    unsigned int res2[8];
}ddr_bd_t;
extern ddr_bd_t *ptr_ddr_bd;


/*
    CBF 寄存器结构  0x43C00400
*/
typedef struct {
    /* ---- 0x00 ~ 0x08 识别/版本 ---- */
    unsigned int MAGIC;                    /* 0x00  读: 必须 0x43424633 */
    unsigned int VERSION;                  /* 0x04  读: 硬件版本 */
    unsigned int BUILD_ID;                 /* 0x08  读: 写操作前门禁, 0x124 */

    /* ---- 0x0C 控制 ---- */
    unsigned int CONTROL;                  /* 0x0C  写: bit0 START, bit1 SOFT_RESET, bit2 CLEAR_ERRORS */

    /* ---- 0x10 状态 ---- */
    unsigned int STATUS;                   /* 0x10  读: BUSY/DONE/ERROR */

    /* ---- 0x14 ~ 0x1C 帧参数 ---- */
    unsigned int IQ_POINTS;                /* 0x14  写: 输入快照数 */
    unsigned int ROLL_INTERVAL;            /* 0x18  写: roll 更新间隔 */
    unsigned int HISTORY_DELAY;            /* 0x1C  读: 当前为 10 */

    /* ---- 0x20 ~ 0x2C 输入配置 ---- */
    unsigned int INPUT_BASE;               /* 0x20  写: IQ DDR 起始地址 */
    unsigned int INPUT_BYTES;              /* 0x24  写: IQ 长度 */
    unsigned int FRAME_ID;                 /* 0x28  写: 帧号 */
    unsigned int INPUT_FORMAT;             /* 0x2C  写: 0x00011010 */

    /* ---- 0x30 ~ 0x34 输出统计 ---- */
    unsigned int OUTPUT_VALID_SNAPSHOTS;   /* 0x30  读: 有效输出快照数 */
    unsigned int OUTPUT_VALID_BYTES;       /* 0x34  读: 有效输出字节数 */

    /* ---- 0x38 错误 ---- */
    unsigned int ERROR_STATUS;             /* 0x38  读/W1C: 错误原因 */

    /* ---- 0x3C ~ 0x44 运行状态 ---- */
    unsigned int CURRENT_SNAPSHOT;         /* 0x3C  读: 运行进度 */
    unsigned int CURRENT_ROLL_INDEX;       /* 0x40  读: 当前 roll 组 */
    unsigned int ACTIVE_ROLL_Q29;          /* 0x44  读: 当前 roll 值 */

    /* ---- 0x48 调试/兼容 ---- */
    unsigned int SRIO_TARGET_ADDR;         /* 0x48  写: CBF 兼容/调试字段 */

    /* ---- 0x4C ~ 0x54 ---- */
    unsigned int SRIO_TRANSFER_BYTES;      /* 0x4C  读: 实际传输长度 */
    unsigned int DEBUG_CAPTURE_BASE;       /* 0x50  写: 调试捕获起点 */
    unsigned int DEBUG_CAPTURE_LIMIT;      /* 0x54  写: 调试捕获上限 */

    /* ---- 0x58 ~ 0x68 计数 ---- */
    unsigned int SATURATION_COUNT;         /* 0x58  读: 饱和计数 */
    unsigned int RESERVED_5C;              /* 0x5C  保留 */
    unsigned int PARAM_GEN_CYCLES;         /* 0x60  读: 参数生成周期数 */
    unsigned int FRAME_CYCLES;             /* 0x64  读: 本帧 125 MHz 周期数 */
    unsigned int DROP_COUNT;               /* 0x68  读: 结果丢失计数 */

    /* ---- 0x6C 控制 ---- */
    unsigned int ROLL_SIGN;                /* 0x6C  写: bit0 roll 加/减方向 */

    /* ---- 0x70 ~ 0x74 中断 ---- */
    unsigned int IRQ_STATUS;               /* 0x70  写/W1C: 清中断状态 */
    unsigned int IRQ_MASK;                 /* 0x74  写: CBF/NREAD 中断屏蔽 */

    /* ---- 0x78 ~ 0x9C reader/NREAD 诊断 ---- */
    unsigned int READER_DEBUG;             /* 0x78  读: IQ reader 状态 */
    unsigned int SR_LINK_STATUS;           /* 0x7C  读: SRIO 状态 */
    unsigned int READER_ARADDR;            /* 0x80  读: 最近 AR 地址 */
    unsigned int READER_AXI_STATUS;        /* 0x84  读: 最近 AXI 响应 */
    unsigned int NREAD_EVENT_COUNT;        /* 0x88  读: 成功 NREAD 累计数 */
    unsigned int NREAD_LAST_INFO;          /* 0x8C  读: 最近 NREAD 状态/bank/长度提示 */
    unsigned int NREAD_ERR_COUNT;          /* 0x90  读: NREAD 错误累计数 */
    unsigned int NREAD_LAST_BYTES;         /* 0x94  读: 最近成功 NREAD 字节数 */
    unsigned int NREAD_OVERRUN_CNT;        /* 0x98  读: 回读覆盖计数 */

    /* ---- 0x9C ~ 0xA0 FIFO ---- */
    unsigned int FIFO_OVF_COUNT;           /* 0x9C  读: 结果 FIFO 溢出累计数 */
    unsigned int FIFO_OVF_FLAG;            /* 0xA0  读: FIFO 溢出标志 */

    /* ---- 0xA4 ~ 0xFF 保留 ---- */
    unsigned int RESERVED_A4[23];          /* 0xA4 ~ 0xFF, 23 个 word */

    /* ---- 0x100 ~ 0x10C 角度表 ---- */
    unsigned int ANGLE_INDEX;              /* 0x100 写: 0..255 角度表索引 */
    unsigned int ANGLE_Q29;                /* 0x104 写: signed Q3.29 角度值, 单位 rad */
    unsigned int ANGLE_COMMAND;            /* 0x108 写: bit0=1 提交角度表写入 */
    unsigned int ANGLE_READBACK;           /* 0x10C 读: 角度表读回 */
}ddr_cbf_t;
extern ddr_cbf_t *ptr_ddr_cbf;

/*
    W2 寄存器结构  0x43C10000 dsp rapidio
*/
typedef struct{
    unsigned int DDR_PR;    //地址更新中断
    unsigned int res;   
    unsigned int SEND_BYTE_CNT_0;       //外置传感器中断
    unsigned int SENDR_BASE_ADDR_0;     //待发送数据的DDR首地址
    unsigned int RECEIVE_BASE_ADDR_0;   //接收到的数据的DDR存储地址
    unsigned int SEND_START_0;          //发送启动标志,置1发送
    unsigned int SEND_TARGET_ADDR_0;    //从机端数据存放地址
    unsigned int SEND_BYTE_CNT_1;       //待发送的数据长度
    unsigned int SENDR_BASE_ADDR_1;     //待发送数据的DDR首地址
    unsigned int RECEIVE_BASE_ADDR_1;   //接收到的数据的DDR
    unsigned int SEND_START_1;          //发送启动标志
    unsigned int SEND_TARGET_ADDR_1;    //从机端数据存放地址
    unsigned int IQ_BASE_ADDR;          //IQ数据地址 - 不使用
    unsigned int RESERVED[8];
    unsigned int READ_TARGET_ADDR_0;    //DSP确认地址
    unsigned int READ_TARGET_ADDR_1;    //DSP确认地址
    unsigned int READ_BYTE_CNT_0;       
}ddr_w2_t;
extern ddr_w2_t *ptr_ddr_w2;

/*
    CFG 寄存器结构  0x43C10400
*/
typedef struct{
    unsigned int roll_wdata;            
    unsigned int roll_addr;
    unsigned int roll_we;
    unsigned int sinc_coeff[6];
    unsigned int sinc_addr;
    unsigned int sinc_we;
    unsigned int full_coeff;
    unsigned int full_addr;
    unsigned int full_we;
    unsigned int sub_coeff;
    unsigned int sub_addr;
    unsigned int sub_we;
    unsigned int roll_update_q29;
    unsigned int roll_update_source;       /* 0x048：动态横摇数据来源选择，参考PL定义为SOURCE，不是数组下标。 */
    unsigned int roll_update_push;         /* 0x04C：动态横摇参数提交脉冲，写1提交本帧横摇值。 */
    unsigned int frequency_code;           /* 0x050：频率编码，按PL协议由fc_kHz/10得到，合法编码为20..40。 */
}ddr_cfg_t;
extern ddr_cfg_t *ptr_ddr_cfg;

/*
    AXI DMA S2MM  0x40400000  DMA 寄存器
*/
typedef struct{
    unsigned int RESERVED[12];
    unsigned int S2MM_DMACR;        //0x30 
    unsigned int S2MM_DMASR;        //0x34 
    unsigned int S2MM_CURDESC_L;    //0x38
    unsigned int S2MM_CURDESC_H;    //0x3C
    unsigned int S2MM_TAILDESC_L;   //0x40
    unsigned int S2MM_TAILDESC_H;   //0x44
}ddr_dma_t;
extern ddr_dma_t *ptr_ddr_dma;

/*
    w3 扩展寄存器 0x43C20400
*/
typedef struct{
    unsigned int W3_MAGIC;          //wc21            
    unsigned int SAMPLE_FS_HZ;      //iq有效采样率元数据
    unsigned int CARRIER_FC_HZ;     //中⼼频率元数据
    unsigned int WORK_MODE;          //⼯作模式
    unsigned int EXT_STATUS;         //扩展状态
}ddr_w3_t;
extern ddr_w3_t *ptr_ddr_w3;

/*
    w4 参数镜像
*/
typedef struct {
    unsigned int MIR2;                 // 0x000
    unsigned int MIR2_ITEM_COUNT;      // 0x004
    unsigned int MIR2_WRITE_COUNT;     // 0x008
    unsigned int RES_00C;              // 0x00C
    unsigned int W1_CONTROL;           // 0x010
    unsigned int W1_IQ_POINTS;         // 0x014
    unsigned int W1_ROLL_INTERVAL;     // 0x018
    unsigned int RES_01C;              // 0x01C
    unsigned int W1_INPUT_BASE;        // 0x020
    unsigned int W1_INPUT_BYTES;       // 0x024
    unsigned int W1_FRAME_ID;          // 0x028
    unsigned int W1_INPUT_FORMAT;      // 0x02C
    unsigned int RES2[6];              // 0x030 ~ 0x047
    unsigned int W1_SRIO_TARGET_ADDR;  // 0x048
    unsigned int RES3[8];              // 0x04C ~ 0x06B
    unsigned int W1_ROLL_SIGN;         // 0x06C
    unsigned int RES4[1];              // 0x070
    unsigned int W1_IRQ_MASK;          // 0x074
    unsigned int RES5[34];             // 0x078 ~ 0x0FF
    unsigned int W1_ANGLE_INDEX;       // 0x100
    unsigned int W1_ANGLE_Q29;         // 0x104
    unsigned int RES6[62];             // 0x108 ~ 0x1FF
    unsigned int W2_CFG[20];           // 0x200 ~ 0x24F
    unsigned int RES7[44];             // 0x250 ~ 0x2FF
    unsigned int W3_SAMPLE_FS_HZ;      // 0x300
    unsigned int W3_CARRIER_FC_HZ;     // 0x304
    unsigned int W3_WORK_MODE;         // 0x308
    unsigned int W3_EXT_STATUS;        // 0x30C
} ddr_w4_t;
extern ddr_w4_t *ptr_ddr_w4;

typedef struct
{       
	unsigned int head;
    uint8_t BASH_TEST;
    uint8_t SONAR_SHOW;
    uint8_t SIDE_MOD;
	uint8_t PTD_MOD;
	float ClolorRatio;
    uint8_t INT_MOD;
    uint8_t WATER_DETECTION;
    uint8_t Beamtype;
    uint8_t FOCUS;
	uint8_t Water_Dro;
	uint8_t IMAGE_Work_Mod;
	uint8_t INS_MOD;
	uint8_t DATA_TYPE;
	uint8_t Roll_Oe;
	uint8_t Ctrl_bow;
    uint8_t Ctrl_heave;
	uint8_t THR_CON;
	float SIDELOBE_FACTOR; 
	float UP_LIM;
	float DOWN_LIM;
	float ANGLE_LIM;
	unsigned int IMAGE_H;
	unsigned int IMAGE_W;
	float SIDE_RATIO;
	float SVP_TEST; //手动声速
	unsigned int Beam_num;  //波束个数
	float BeamOA_st; //起始角度
	float BeamOA_fi; //终止角度
	unsigned int    Mems_Num;//每帧数据下包含的mems个数
	float Roll; //横摇值
	float angle; //纵摇值
	unsigned int    Ch_Beam;//波束显示的指定波束波束号
	float           Sync_Longitude;//同步经度
	float           Sync_Latitude;//同步纬度
	float           Sync_Height;//同步高程
    float           Sync_Factor;//同步品质因子
	float           Sync_Factorync_Speed;//同步航速
	float           Sync_Heave;//同步升沉
	float           Ping_Dis;//Ping间距离
	float			Heading;//航向
	unsigned short	Time_Date;//日- time tag
    unsigned short	Time_Year;//年- time tag
	float			Time_Sec;//秒- time tag
    unsigned short	Time_Min;//分- time tag
	unsigned short	Time_Hours;//时- time tag
    unsigned short	Ch_Test;//通道自检
	unsigned short	Ch_Error;//异常通道
	unsigned int	TVG_Gain;//固定增益
	unsigned int	TVG_Absorb;//吸收
	unsigned int	TVG_Spread;//扩散
	unsigned int	Roll_St;//横摇稳定
	unsigned int	Pitch_St;//纵摇稳定
	unsigned int	Beamform_Type;//Beamform方法
	unsigned int	Quality_Filter;//质量滤波
	unsigned int	Device_Type;//设备型号
	float			Install_Angle;//安装倾角
	float			Blind;//盲区比例
	unsigned int	Pitch_Oe;//纵摇补偿
	unsigned int	Median_Filter;//中值滤波
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
	unsigned int 	BeamOA;
	unsigned int    BeamOA_k;
	unsigned int	Tao_Coe;//用于卷积计算的系数
	unsigned int    end_tail[6]; //填充0xEEEEEEEE
	unsigned int    rsvd;
}frame_Parameter_t;

typedef struct{
	unsigned int ch_err[60];
}frame_ch_error_t;

typedef struct{
	float roll_time_diff[400];
}frame_roll_time_diff_t;

typedef struct{
	float Roll_Sequential_Value[400];
}frame_roll_sequential_value_t;

typedef struct{
	frame_Parameter_t f_parameter;
	frame_ch_error_t f_ch_err;
	frame_roll_time_diff_t f_roll_time_diff;
	frame_roll_sequential_value_t f_Roll_Sequential_Value;
}frame_t;

typedef char frame_parameter_size_check[(sizeof(frame_Parameter_t) == 260U) ? 1 : -1];
typedef char frame_size_check[(sizeof(frame_t) == 3700U) ? 1 : -1];

/***************************UIO配置结构体**********************/
typedef struct
{
    int               fd;                         //运行时匹配到的UIO设备文件描述符，例如/dev/uio8
    char              uiod[32];                   //运行时生成的实际设备路径，不再保存固定的UIO编号
    char              sysfs_path_file[128];       //运行时生成的map0/size路径，用于记录实际映射大小
    uintptr_t         physical_addr;              //期望匹配的设备树物理基地址，例如0x43C30000
    unsigned int      expected_size;              //设备树中期望的map0映射大小；UIO校验时会按页对齐
    unsigned int      mem_size;                   //实际从sysfs读取并通过校验的map0映射大小
    void              *mem_ptr;                   //mmap返回的用户态虚拟地址，访问DDR或寄存器必须使用此地址
}UIO_CONFIG_PARAMETER;


extern UIO_CONFIG_PARAMETER uio_iq_a;
extern UIO_CONFIG_PARAMETER uio_iq_b;
extern UIO_CONFIG_PARAMETER uio_frame;
extern UIO_CONFIG_PARAMETER uio_nread;
extern UIO_CONFIG_PARAMETER uio_bd;
extern UIO_CONFIG_PARAMETER uio_tvg;
extern UIO_CONFIG_PARAMETER uio_dma;
extern UIO_CONFIG_PARAMETER uio_fpga_register;
extern UIO_CONFIG_PARAMETER uio_cfb;
extern UIO_CONFIG_PARAMETER uio_w2;
extern UIO_CONFIG_PARAMETER uio_cfg;
extern UIO_CONFIG_PARAMETER uio_w3;
extern UIO_CONFIG_PARAMETER uio_w4;

/* 根据物理地址和映射大小动态查找并映射一个UIO节点，0表示成功。 */
extern int uio_init(UIO_CONFIG_PARAMETER* uio_parameter);

/* 初始化全部DDR、寄存器和CBF中断UIO；失败时返回-1并禁止启动业务线程。 */
extern int fpga_interface_init(void);

#endif
