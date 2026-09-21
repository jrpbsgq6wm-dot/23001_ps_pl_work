#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/stat.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include "main.h"
#include "tcptrans_link.h"
#include "drytoupper.h"
#include "drytowet.h"
#include "uppertodry.h"
#include "fpga_init.h"
#include "armtofpga.h"
#include "armtodsp.h"
#include "wettodry.h"
#include "tempsensor.h"
#include "eeprom.h"
#include <semaphore.h>

/*互斥信号量*/
sem_t sem_WET,sem_UPPER;                
/**/
struct timeval start_time = {0};
struct timeval end_time = {0};
/*线程*/
pthread_t thread[6] = {0};
/*互斥锁*/
pthread_mutex_t mut = {0}, mut2 = {0};



volatile int server_netStatus = 0;  //服务器状态
volatile int client_netStatus = 0;  //客户端状态
volatile int DryTOWet_Flag = 0; //干端发送湿端标志位
int flag_timer = 0;             //温度传感器NST175的定时器
int irq_on = 1;
int Fpga_start_mod = 0; //FPGA开始工作标志;
int Flag_ExtSenserBuf = 0;
unsigned int counter = 0;
int Bd_Date_TransBuf_Flag = 1;//波束下放buf切换


/******20240820*****/
unsigned char wet_status = 0;
unsigned char dry_fpga_status = 0;
unsigned char dsp_status = 0;
unsigned char GGAZDA_status = 0;
unsigned char Heading_status = 0;
unsigned char TSS1_status = 0;
unsigned char SV_status = 0;
unsigned char PPS_status = 0;
unsigned char rs_flag = 0;
/*********************/
char ZYNQ_VERSION[4] = {'V', '1', '0', '0'}; // zynq版本号

int Sonar_FristPing_flag = 0;//声纳第一ping标志位

//FILE* fb;

void thread_create(void)
{
    int temp;
    memset(&thread, 0, sizeof(thread));
    //将IQ数据上传到显控端线程
    if ((temp = pthread_create(&thread[0], NULL, thread_send_to_upper, NULL)) != 0){
        printf("send_to_dsp_upper thread created fail!\n");
    }else{
        printf("send_to_dsp_upper thread is created!\n");
    }
    //回复显控波束数据线程
    if ((temp = pthread_create(&thread[1], NULL, thread_dry_to_upper, NULL)) != 0)
        printf("dry to upper thread created fail!\n");
    else
        printf("dry to upper thread is created!\n");
    //处理显控下发到干端的命令
    if ((temp = pthread_create(&thread[2], NULL, thread_upper_to_dry, NULL)) != 0)
        printf("upper to dry thread created fail!\n");
    else
        printf("upper to dry thread is created!\n");
    //处理湿端指令反馈的线程函数
    if ((temp = pthread_create(&thread[3], NULL, thread_wet_to_dry, NULL)) != 0)
        printf("wet to dry thread created fail!\n");
    else
        printf("wet to drt thread is created!\n");
    //解析显控控制湿端指令并发送到湿端的线程
    if ((temp = pthread_create(&thread[4], NULL, thread_dry_to_wet, NULL)) != 0)
        printf("dry to wet thread created fail!\n");
    else
        printf("dry to wet thread is created!\n");
   /* if ((temp = pthread_create(&thread[5], NULL, thread_temp_sensor, NULL)) != 0)
        printf("temp sensor thread created fail!\n");
    else
        printf("temp sensor thread is created!\n");*/
    //创建网络检测线程
    if ((temp = pthread_create(&thread[5],NULL,thread_net_check,NULL)) != 0 ){
        printf("net_check thread created fail!\n");
    }else{
        printf("net_check thread created successfully!\n");
    }
}

void *thread_upper_to_dry(void *arg)
{
	/* pthread_create()传入的上下文当前未使用，显式标记避免编译器告警。 */
	(void)arg;
    upper_to_dry();
	return NULL;
}

void *thread_dry_to_upper(void *arg)
{
	/* pthread_create()传入的上下文当前未使用，显式标记避免编译器告警。 */
	(void)arg;
    dry_to_upper();
	return NULL;
}

void *thread_wet_to_dry(void *arg)
{
	/* pthread_create()传入的上下文当前未使用，显式标记避免编译器告警。 */
	(void)arg;
    wet_to_dry();
	return NULL;
}

void *thread_dry_to_wet(void *arg)
{
	/* pthread_create()传入的上下文当前未使用，显式标记避免编译器告警。 */
	(void)arg;
    dry_to_wet();
	return NULL;
}


void *thread_send_to_upper(void *arg)
{
	/* pthread_create()传入的上下文当前未使用，显式标记避免编译器告警。 */
	(void)arg;
    iq_to_upper();
	return NULL;
}

void *thread_net_check(void *arg){
	/* pthread_create()传入的上下文当前未使用，显式标记避免编译器告警。 */
	(void)arg;
    network_monitor();
	return NULL;
}

void thread_wait(void)
{
    if (thread[0] != 0)
    {
        pthread_join(thread[0], NULL);
        printf("tcp trans link thread is over \n");
    }
    if (thread[1] != 0)
    {
        pthread_join(thread[1], NULL);
        printf("dry to upper  thread is over \n");
    }
    if (thread[2] != 0)
    {
        pthread_join(thread[2], NULL);
        printf("upper to dry thread is over \n");
    }
    if (thread[3] != 0)
    {
        pthread_join(thread[3], NULL);
        printf("wet to dry is over \n");
    }
    if (thread[4] != 0)
    {
        pthread_join(thread[4], NULL);
        printf("dry to wet is over \n");
    }
    if (thread[5] != 0)
    {
        pthread_join(thread[5], NULL);
        printf("net check is over \n");
    }
}

/********************************************************************************
 *  * * 名称：      my_copy
 *  * * 功能：      汇编实现memcpy函数
 *  * * 入口参数：  volatile unsigned char *dst, volatile unsigned char *src, int sz
 *  * * 出口参数：  无
 *  * ****************************************************************************/
void my_copy(volatile unsigned char *dst, volatile unsigned char *src, int sz)
{
    unsigned int bulk_size;
    unsigned int tail_size;
    unsigned int i;
    int neon_size;

    /* 空指针或非正长度没有可复制的数据，直接返回。 */
    if (dst == NULL || src == NULL || sz <= 0)
    {
        return;
    }

    /*
     * NEON部分只处理完整的64字节块，剩余不足64字节的数据由尾部循环处理。
     * 这样不会像旧实现一样把sz向上取整后越过源缓冲区和目标缓冲区边界。
     */
    bulk_size = ((unsigned int)sz) & ~63U;
    tail_size = ((unsigned int)sz) - bulk_size;

    if (bulk_size != 0U)
    {
        /* 汇编循环每次消耗64字节，使用int变量与SUBS指令保持一致。 */
        neon_size = (int)bulk_size;
        asm volatile(
            "    NEONCopyPLD:                      \n"
            "    VLDM %[src]!,{d0-d7}              \n"
            "    VSTM %[dst]!,{d0-d7}              \n"
            "    SUBS %[sz],%[sz],#0x40            \n"
            "    BGT NEONCopyPLD                   \n"
            : [dst] "+r"(dst), [src] "+r"(src), [sz] "+r"(neon_size)
            :
            : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "cc", "memory");
    }

    /* 汇编部分已经推进了dst/src指针，这里只复制不足64字节的尾部。 */
    for (i = 0U; i < tail_size; ++i)
    {
        dst[i] = src[i];
    }
}

/*****************************************************************
 * * 名称：               crc16
 * * 功能：               计算两个字节的CRC
 * * 入口参数：           *data：数组的起始地址  len：数组的长度
 * *                      crc：crc的初值
 * * 出口参数：            两字节的crc
 * *****************************************************************/
unsigned short crc16(void *data, unsigned int len, unsigned short crc)
{
    unsigned char *pBuff = (unsigned char *)data;
    unsigned int i;
    for (i = 0; i < len; i++)
    {
        crc = crc ^ (unsigned short)pBuff[i];
        crc = (crc >> 8) ^ crc16_tab[crc & 0x0FF];
    }
    return crc;
}

/********************************************************************************
 * * 名称：                    setTimer
 * * 功能：                    设置定时器到时间标志位置1
 * * 入口参数：                     无
 * * 出口参数：                     无
 * *********************************************************************************/
void setTimer(int seconds, int mseconds)
{
    struct timeval temp;
    temp.tv_sec = seconds;
    temp.tv_usec = mseconds;
    select(0, NULL, NULL, NULL, &temp);
    flag_timer = 1;
}

int main(int argc, char *argv[])
{
    /*
     * TCP对端断开后，send()默认可能触发SIGPIPE并直接终止整个进程。
     * 全局忽略SIGPIPE后，send()会返回EPIPE或ECONNRESET，
     * 各业务线程可以沿用现有错误处理流程清理连接，不影响其他线程继续运行。
     */
    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    {
        perror("signal(SIGPIPE) failed");
        return 1;
    }

    /*
     * 先完成全部UIO窗口的动态匹配和mmap。
     * 只有所有DDR、寄存器以及CBF中断节点都映射成功，后续线程才允许访问
     * ptr_ddr_*、ptr_wet_fpga_register_data和ptr_mark_registers等全局指针。
     * 初始化失败时直接退出，避免业务线程在无效地址上运行导致段错误。
     */
    if (fpga_interface_init() != 0)
    {
        fprintf(stderr, "FPGA UIO interface initialization failed.\n");
        return 1;
    }
    /* 合并后的FPGA/CBF UIO节点同时提供CBF寄存器映射和中断。 */
    write(uio_cfb.fd, &irq_on, sizeof(irq_on));
    init_tcptrans_socket();
	sem_init(&sem_WET,0,1);
	sem_init(&sem_UPPER,0,0);
    pthread_mutex_init(&mut, NULL); //初始化互斥锁
    pthread_mutex_init(&mut2, NULL);
    thread_create(); //开启5个子线程
    thread_wait();   //等待回收子线程
	sem_destroy(&sem_WET);
	sem_destroy(&sem_UPPER);
    pthread_mutex_destroy(&mut);
    pthread_mutex_destroy(&mut2);
    return 0;
}
