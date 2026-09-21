#include <stdio.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
#include "main.h"
#include "tcptrans_link.h"
#include "armtofpga.h"
#include "beam_down.h"
#include "wettodry.h" 
#include "eeprom.h"

int pc_link_flag = 0;
struct sockaddr_in Servaddr_wet,Servaddr_upper;
int Socket_fd_upper = 0, Connect_fd_upper = 0, Socket_fd_wet = 0;
unsigned long snd_size = 0;
socklen_t optlen = 0;


/*206.1.11 netword restart*/
#define PING_IP "192.168.0.5"     // 你要检测的 IP
#define CHECK_INTERVAL 5         //  秒检测一次
#define MAX_FAILS 1               // 连续失败 2 次才重启网络
#define MAX_REBOOT_TIMES    50    //连续重启5次后退出线程
static volatile int g_running = 1;
char pc_ip[16] = {0};             //保存PC的IP

pthread_mutex_t net_mutex = PTHREAD_MUTEX_INITIALIZER;

void test(void)
{
    printf("hello world\n");
}

/********************************************************************************
 * 名称：                    setkeepalive
 * 功能：                    设置setkeepalive
 * 入口参数：            	 无
 * 出口参数：            	 无
 *********************************************************************************/
void setkeepalive(int lisfd)
{
    if (lisfd)
    {
        int opt = 1;
        int keepalive = 1;    // 开启KeepAlive
        int keepidle = 5;     // 空闲5秒后开始探测
        int keepintvl = 2;    // 探测间隔2秒
        int keepcnt = 5;      // 探测失败3次后断开连接
        int nodelay = 1;
        // 开启调整keepalive的选项
        setsockopt(lisfd, SOL_SOCKET, SO_KEEPALIVE, (const void *)&keepalive, sizeof(keepalive)); 
        // 距离上次发送数据多长时间后开始探测
        setsockopt(lisfd, IPPROTO_TCP, TCP_KEEPIDLE, (const void *)&keepidle, sizeof(keepidle));
        // 探测没有回应要坚持多少次
        setsockopt(lisfd, IPPROTO_TCP, TCP_KEEPCNT, (const void *)&keepcnt, sizeof(keepintvl)); 
        // 无数据交互下 每隔多长时间探测一次
        setsockopt(lisfd, IPPROTO_TCP, TCP_KEEPINTVL, (const void *)&keepintvl, sizeof(keepcnt)); 
        // 允许端口快速复用（解决拔网线后端口占用问题）
        setsockopt(lisfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        setsockopt(lisfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
        // 关闭Nagle算法（嵌入式场景下数据传输更及时）
        setsockopt(lisfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    }
}

void init_tcptrans_socket(void)
{
    if ((Socket_fd_upper = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        exit(0);
    }

	setkeepalive(Socket_fd_upper);

    bzero(&Servaddr_upper,sizeof(Servaddr_upper));
    Servaddr_upper.sin_family = AF_INET;
    Servaddr_upper.sin_addr.s_addr = inet_addr("192.168.0.4");  // 绑定到Ps侧网口
    Servaddr_upper.sin_port = htons(DEFAULT_PORT_DRY);
    if (bind(Socket_fd_upper, (struct sockaddr*)&Servaddr_upper, sizeof(Servaddr_upper)) == -1)
    {
        printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
        exit(0);
    }
    if (listen(Socket_fd_upper, 10) == -1)
    {
        printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
        exit(0);
    }
}

void error_process(void)
{
    int wet, upper;
    DBG("Network interrupt!.\n");
    ptr_fpga_register_data->wsm_con = 0;
	counter = 0;
	Fpga_start_mod = 0;
	Change_InsModeState_ClearBuf();
	server_netStatus = 0;
	Wet_to_Dry_flag = 0;
    rs_flag = 0;
    wet_status = 0;
    pc_link_flag = 0;
	upper = close(Connect_fd_upper);
	usleep(1000);
    if(upper == 0){
        DBG("upper=%d\n", upper);
    }else{
        DBG("ERRNO=%d\n",errno);
    }
    
}

/********************************************************************************
 *  * * 名称：                    recv_socket
 *  * * 功能：
 *  * socket接收函数（接收socket长度很大时用此函数能保证接受全）
 *  * * 入口参数：                     无
 *  * * 出口参数：                     无
 *  * *********************************************************************************/
unsigned int recv_socket(int fd, char* buf, unsigned int len)
{
    unsigned int lenth_recv = len;
    unsigned int lenth_temp = 0;
    char* buftem = buf;
    unsigned int num = 0;
    do
    {
        lenth_temp = recv(fd, buftem, lenth_recv, 0);
        if (lenth_temp <= 0)
        {
            break;
        }
        lenth_recv = lenth_recv - lenth_temp;
        buftem = buftem + lenth_temp;
        num = num + lenth_temp;
    } while (lenth_recv > 0);

    return num;
}


/********************************************************************************
 *  * * 名称：                    network_monitor
 *  * * 功能：网络监控函数
 *  * * 入口参数：                     无
 *  * * 出口参数：                     无
 *  * *********************************************************************************/
// 执行一次 ping 1次 超时1s
static int ping_once(const char *ip) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "ping -c 1 -W 3 %s > /dev/null 2>&1", ip);
    return system(cmd) == 0 ? 1 : 0;
}

// 重启网络服务（你现在手动执行的命令）
static void restart_network(void) {
    pthread_mutex_lock(&mut);
    DBG("[NETMON] Restarting SYS...\n");
    system("cp /mnt/emmc3/my_app_crash.log /mnt/emmc3/log.log && reboot");
    pthread_mutex_unlock(&mut);
}

// 监控线程
void network_monitor(void) {
    int ping_fail_count = 0;
    int pc_ok = 1;
    int wet_ok = 1;

    while(1){

        
    }

    while (g_running) {
        // 检查 PC IP 是否有效（非空且以 "192.168." 开头）
        if (strstr(pc_ip,"192.168.0.") != NULL) {
            pc_ok = ping_once(pc_ip);
        } else {
            // IP 无效，重置计数器并等待
            ping_fail_count = 0;
            printf("Invalid PC IP, skip monitoring\n");
            sleep(CHECK_INTERVAL);
            continue;
        }
        // 检查 WET 设备
        wet_ok = ping_once(PING_IP);
        // 两者都失败才触发重启
        if (!pc_ok && !wet_ok) {
            ping_fail_count++;
            if (ping_fail_count >= MAX_FAILS) {
                restart_network();
            }else{
                continue;
            }
        } else {
            // 只要有一个恢复，就重置所有计数器
            ping_fail_count = 0;
        }
        sleep(CHECK_INTERVAL);
    }
    // void 函数无需 return
}

void get_pc_ip(int sock_pc){
    struct sockaddr_in peer;
    socklen_t len = sizeof(peer);
    if(getpeername(sock_pc,(struct sockaddr*)&peer,&len) == 0){
        inet_ntop(AF_INET,&peer.sin_addr,pc_ip,sizeof(pc_ip));
        printf("PC IP :%s\n",pc_ip);
    }else{
        perror("getpeername");
    }
}
