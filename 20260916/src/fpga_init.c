#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "fpga_init.h"
#include "armtofpga.h"
#include "armtodsp.h"

int fd = 0;

/*
 * UIO编号由内核按照设备树节点探测顺序动态分配，因此不能把uioX编号当作
 * 固定接口保存。这里仅保存每个硬件窗口的物理基地址和期望映射大小，真正
 * 的/dev/uioX路径由uio_init()扫描sysfs后动态填写。
 *
 * 例如，uio_make_fpga_register的设备树地址是0x43C30000，即使它在不同内核
 * 或不同设备树顺序下变成/dev/uio6、/dev/uio13，都可以通过物理地址找到它。
 */
UIO_CONFIG_PARAMETER uio_iq_a = { .fd = -1, .physical_addr = DDR_IQ_A_ADDR, .expected_size = IQ_A_SIZE };
UIO_CONFIG_PARAMETER uio_iq_b = { .fd = -1, .physical_addr = DDR_IQ_B_ADDR, .expected_size = IQ_B_SIZE };
UIO_CONFIG_PARAMETER uio_frame = { .fd = -1, .physical_addr = DDR_FRAME_ADDR, .expected_size = FRAME_SIZE };
UIO_CONFIG_PARAMETER uio_nread = { .fd = -1, .physical_addr = DDR_NREAD_ADDR, .expected_size = NREAD_SIZE };
UIO_CONFIG_PARAMETER uio_bd = { .fd = -1, .physical_addr = DDR_BD_ADDR, .expected_size = BD_SIZE };
UIO_CONFIG_PARAMETER uio_tvg = { .fd = -1, .physical_addr = DDR_TVG_ADDR, .expected_size = TVG_SIZE };
UIO_CONFIG_PARAMETER uio_dma = { .fd = -1, .physical_addr = DDR_DMA_ADDR, .expected_size = DMA_SIZE };
UIO_CONFIG_PARAMETER uio_fpga_register = { .fd = -1, .physical_addr = DDR_FPGA_REGISTER_ADDR, .expected_size = FPGA_CFB_WINDOW_SIZE };
/* CFB是uio_fpga_register映射中的逻辑子窗口，不再单独调用uio_init()。 */
UIO_CONFIG_PARAMETER uio_cfb = { .fd = -1, .physical_addr = DDR_CFB_ADDR, .expected_size = CFB_SIZE };
UIO_CONFIG_PARAMETER uio_w2 = { .fd = -1, .physical_addr = DDR_W2_ADDR, .expected_size = W2_CFG_WINDOW_SIZE };
/* CFG是uio_w2映射中的逻辑子窗口，不再单独调用uio_init()。 */
UIO_CONFIG_PARAMETER uio_cfg = { .fd = -1, .physical_addr = DDR_CFG_ADDR, .expected_size = CFG_SIZE };
UIO_CONFIG_PARAMETER uio_wet_fpga_register = { .fd = -1, .physical_addr = DDR_WET_FPGA_REGISTER_ADDR, .expected_size = WET_W3_WINDOW_SIZE };
/* W3是uio_wet_fpga_register映射中的逻辑子窗口，不再单独调用uio_init()。 */
UIO_CONFIG_PARAMETER uio_w3 = { .fd = -1, .physical_addr = DDR_W3_ADDR, .expected_size = W3_SIZE };
UIO_CONFIG_PARAMETER uio_make_fpga_register = { .fd = -1, .physical_addr = DDR_MAKE_FPGA_REGISTER_ADDR, .expected_size = MAKE_W4_WINDOW_SIZE };
/* W4是uio_make_fpga_register映射中的逻辑子窗口，不再单独调用uio_init()。 */
UIO_CONFIG_PARAMETER uio_w4 = { .fd = -1, .physical_addr = DDR_W4_ADDR, .expected_size = W4_SIZE };

ddr_bd_t *ptr_ddr_bd = NULL;
ddr_cbf_t *ptr_ddr_cbf = NULL;
ddr_w2_t *ptr_ddr_w2 = NULL;
ddr_cfg_t *ptr_ddr_cfg = NULL;
ddr_dma_t *ptr_ddr_dma = NULL;
ddr_w3_t *ptr_ddr_w3 = NULL;
ddr_w4_t *ptr_ddr_w4 = NULL;

/* UIO节点号由内核按照探测顺序分配，不能作为稳定接口。 */
#define UIO_SYSFS_BASE_PATH "/sys/class/uio"
#define UIO_SCAN_LIMIT      64

/*
 * 从sysfs读取十六进制地址或长度。
 *
 * UIO驱动会在以下文件中提供map0的信息：
 *   /sys/class/uio/uioX/maps/map0/addr
 *   /sys/class/uio/uioX/maps/map0/size
 *
 * 读取失败时返回-1，调用者不能继续使用未初始化的地址或长度。
 */
static int read_sysfs_hex(const char *path, unsigned long long *value)
{
    FILE *fp;
    unsigned long long parsed_value = 0ULL;

    /* 以只读方式打开sysfs属性文件。 */
    fp = fopen(path, "r");
    if (fp == NULL)
    {
        return -1;
    }

    /* sysfs中的地址和大小通常带有0x前缀，%llx可以正确解析。 */
    if (fscanf(fp, "%llx", &parsed_value) != 1)
    {
        fclose(fp);
        return -1;
    }

    fclose(fp);
    *value = parsed_value;
    return 0;
}

/*
 * 根据map0/addr和map0/size查找目标UIO，而不是依赖固定的/dev/uioN编号。
 *
 * 查找流程：
 * 1. 扫描uio0到uio63的map0/addr；
 * 2. 找到与设备树物理地址相同的UIO节点；
 * 3. 读取map0/size并与程序期望值比较；
 * 4. 打开实际匹配到的/dev/uioX；
 * 5. 使用mmap的offset=0映射map0。
 *
 * 注意：mmap的第一个参数只是用户态虚拟地址提示，不是物理地址偏移，真正
 * 映射哪一块硬件资源由UIO设备文件和offset=0对应的map0决定。因此这里的
 * physical_addr只负责校验和定位，不能直接作为mmap的offset使用。
 */
int uio_init(UIO_CONFIG_PARAMETER *uio_parameter)
{
    int index;
    unsigned long long map_addr;
    unsigned long long map_size;
    char addr_path[160];
    char size_path[160];
    char device_path[64];
    void *mapped_addr;
    long page_size;
    unsigned long long expected_map_size;

    if (uio_parameter == NULL)
    {
        fprintf(stderr, "UIO init received a NULL parameter.\n");
        return -1;
    }

    /* 先清理运行时字段，避免失败后残留上一次初始化的数据。 */
    uio_parameter->fd = -1;
    uio_parameter->mem_ptr = NULL;
    uio_parameter->mem_size = 0U;
    uio_parameter->uiod[0] = '\0';
    uio_parameter->sysfs_path_file[0] = '\0';

    /* 逐个检查内核已经注册的UIO节点。不存在的节点直接跳过。 */
    for (index = 0; index < UIO_SCAN_LIMIT; ++index)
    {
        snprintf(addr_path, sizeof(addr_path), "%s/uio%d/maps/map0/addr", UIO_SYSFS_BASE_PATH, index);
        snprintf(size_path, sizeof(size_path), "%s/uio%d/maps/map0/size", UIO_SYSFS_BASE_PATH, index);

        /* 读取当前UIO的map0物理地址。 */
        if (read_sysfs_hex(addr_path, &map_addr) != 0)
        {
            continue;
        }

        /* 物理地址不匹配时，当前UIO不是目标窗口，继续扫描下一个。 */
        if ((uintptr_t)map_addr != uio_parameter->physical_addr)
        {
            continue;
        }

        /* 地址匹配后必须继续校验映射大小，防止设备树节点长度配置错误。 */
        if (read_sysfs_hex(size_path, &map_size) != 0 || map_size > UINT_MAX)
        {
            fprintf(stderr, "Invalid UIO map size for physical address 0x%08lX.\n",
                    (unsigned long)uio_parameter->physical_addr);
            return -1;
        }

        /*
         * UIO的map0/size按系统页大小对齐返回。
         * 例如设备树reg长度为0x60时，Linux通常会把可映射窗口报告为0x1000。
         * 因此不能直接拿设备树原始长度与sysfs长度比较，否则合法的DMA寄存器
         * 节点会被误判为大小错误。
         */
        page_size = sysconf(_SC_PAGESIZE);
        if (page_size <= 0)
        {
            fprintf(stderr, "Cannot determine system page size for UIO validation.\n");
            return -1;
        }

        /* 将设备树中的期望长度向上取整到完整页，得到合法map0长度。 */
        expected_map_size = 0ULL;
        if (uio_parameter->expected_size != 0U)
        {
            expected_map_size =
                (((unsigned long long)uio_parameter->expected_size +
                  (unsigned long long)page_size - 1ULL) /
                 (unsigned long long)page_size) *
                (unsigned long long)page_size;
        }

        /* 对齐后的长度仍必须匹配，防止设备树节点长度配置过大或过小。 */
        if (expected_map_size != 0ULL && map_size != expected_map_size)
        {
            fprintf(stderr,
                    "UIO map size mismatch at 0x%08lX: expected reg 0x%X "
                    "(aligned map 0x%llX), got 0x%llX.\n",
                    (unsigned long)uio_parameter->physical_addr,
                    uio_parameter->expected_size,
                    expected_map_size,
                    map_size);
            return -1;
        }

        /* 只有地址和大小都正确后，才生成并打开实际的UIO设备文件。 */
        snprintf(device_path, sizeof(device_path), "/dev/uio%d", index);
        uio_parameter->fd = open(device_path, O_RDWR);
        if (uio_parameter->fd < 0)
        {
            fprintf(stderr, "Cannot open %s: %s\n", device_path, strerror(errno));
            return -1;
        }

        /* offset=0选择UIO的map0，即设备树reg描述的这段DDR或寄存器窗口。 */
        mapped_addr = mmap(NULL, (size_t)map_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, uio_parameter->fd, 0);
        if (mapped_addr == MAP_FAILED)
        {
            fprintf(stderr, "Cannot mmap %s: %s\n", device_path, strerror(errno));
            close(uio_parameter->fd);
            uio_parameter->fd = -1;
            return -1;
        }

        /* 保存实际匹配结果，便于其他模块使用fd以及启动日志排查。 */
        strncpy(uio_parameter->uiod, device_path, sizeof(uio_parameter->uiod) - 1U);
        uio_parameter->uiod[sizeof(uio_parameter->uiod) - 1U] = '\0';
        strncpy(uio_parameter->sysfs_path_file, size_path,
                sizeof(uio_parameter->sysfs_path_file) - 1U);
        uio_parameter->sysfs_path_file[sizeof(uio_parameter->sysfs_path_file) - 1U] = '\0';
        uio_parameter->mem_size = (unsigned int)map_size;
        uio_parameter->mem_ptr = mapped_addr;

        /* 打印实际映射结果，现场可以据此确认UIO编号是否发生变化。 */
        printf("UIO mapped: %s, addr=0x%08lX, size=0x%X\n",
               uio_parameter->uiod,
               (unsigned long)uio_parameter->physical_addr,
               uio_parameter->mem_size);
        return 0;
    }

    /* 扫描结束仍未找到目标地址，不能继续给业务代码提供空映射。 */
    fprintf(stderr, "No UIO map matches physical address 0x%08lX.\n",
            (unsigned long)uio_parameter->physical_addr);
    return -1;
}

/*
 * 从一个已经完成mmap的4KB物理页中建立逻辑子窗口。
 *
 * 同页寄存器不能再次通过物理地址调用uio_init()，因为UIO的map0/addr
 * 会统一显示为页基地址。这里根据子窗口的物理地址计算页内偏移，
 * 让uio_cfb、uio_cfg、uio_w3和uio_w4继续保持原有逻辑接口。
 */
static int uio_bind_subwindow(UIO_CONFIG_PARAMETER *base,
                              UIO_CONFIG_PARAMETER *subwindow)
{
    uintptr_t offset;

    /* 基窗口和子窗口必须已经提供有效的映射信息。 */
    if ((base == NULL) || (subwindow == NULL) ||
        (base->mem_ptr == NULL) || (base->mem_ptr == (void *)-1) ||
        (base->fd < 0))
    {
        return -1;
    }

    /* 子窗口只能位于基窗口内部，不能反向计算或越过映射末端。 */
    if (subwindow->physical_addr < base->physical_addr)
    {
        return -1;
    }

    offset = subwindow->physical_addr - base->physical_addr;
    if ((offset > (uintptr_t)base->mem_size) ||
        (subwindow->expected_size > (base->mem_size - (unsigned int)offset)))
    {
        fprintf(stderr,
                "UIO subwindow out of range: base=0x%08lX, sub=0x%08lX, "
                "offset=0x%lX, size=0x%X, base_size=0x%X.\n",
                (unsigned long)base->physical_addr,
                (unsigned long)subwindow->physical_addr,
                (unsigned long)offset,
                subwindow->expected_size,
                base->mem_size);
        return -1;
    }

    /* 子窗口复用基窗口的fd，在基地址映射内加上页内偏移得到访问地址。 */
    subwindow->fd = base->fd;
    subwindow->mem_ptr = (void *)((unsigned char *)base->mem_ptr + offset);
    subwindow->mem_size = subwindow->expected_size;
    strncpy(subwindow->uiod, base->uiod, sizeof(subwindow->uiod) - 1U);
    subwindow->uiod[sizeof(subwindow->uiod) - 1U] = '\0';
    strncpy(subwindow->sysfs_path_file, base->sysfs_path_file,
            sizeof(subwindow->sysfs_path_file) - 1U);
    subwindow->sysfs_path_file[sizeof(subwindow->sysfs_path_file) - 1U] = '\0';

    return 0;
}

/******************************Interface_init***************************************
 * * 功能：                    FPGA接口初始化
 * * 入口参数：                     无
 * * 出口参数：              0成功，-1失败
 * *********************************************************************************/
int fpga_interface_init(void)
{
    /*
     * 所有业务需要的UIO对象统一放入列表中初始化。
     * 列表顺序只影响初始化顺序，不再决定uioX编号与硬件窗口的对应关系。
     */
    UIO_CONFIG_PARAMETER *uio_list[] = {
        &uio_iq_a,
        &uio_iq_b,
        &uio_frame,
        &uio_nread,
        &uio_bd,
        &uio_tvg,
        &uio_dma,
        &uio_fpga_register,
        &uio_w2,
        &uio_wet_fpga_register,
        &uio_make_fpga_register
    };
    unsigned int i;

    /* 任意一个关键窗口初始化失败，都禁止继续启动业务线程。 */
    for (i = 0U; i < sizeof(uio_list) / sizeof(uio_list[0]); ++i)
    {
        if (uio_init(uio_list[i]) != 0)
        {
            fprintf(stderr, "FPGA UIO initialization failed at item %u.\n", i);
            return -1;
        }
    }

    /*
     * 将同一4KB页内的逻辑寄存器绑定到对应基窗口：
     *   FPGA页 +0x400 -> CFB
     *   W2页  +0x400 -> CFG
     *   WET页 +0x400 -> W3
     *   MAKE页+0x400 -> W4
     */
    if ((uio_bind_subwindow(&uio_fpga_register, &uio_cfb) != 0) ||
        (uio_bind_subwindow(&uio_w2, &uio_cfg) != 0) ||
        (uio_bind_subwindow(&uio_wet_fpga_register, &uio_w3) != 0) ||
        (uio_bind_subwindow(&uio_make_fpga_register, &uio_w4) != 0))
    {
        fprintf(stderr, "FPGA UIO subwindow binding failed.\n");
        return -1;
    }

    /*
     * 所有映射成功后，再把用户态虚拟地址转换成对应的寄存器结构体指针。
     * 这些指针只允许在本函数成功返回后使用，避免访问NULL或MAP_FAILED。
     */
    ptr_ddr_bd = (ddr_bd_t *)uio_bd.mem_ptr;
    ptr_ddr_dma = (ddr_dma_t *)uio_dma.mem_ptr;
    ptr_ddr_cbf = (ddr_cbf_t *)uio_cfb.mem_ptr;
    ptr_ddr_w2 = (ddr_w2_t *)uio_w2.mem_ptr;
    ptr_ddr_cfg = (ddr_cfg_t *)uio_cfg.mem_ptr;
    ptr_fpga_register_data = (FPGA_REGISTERS *)uio_fpga_register.mem_ptr;
    ptr_fpga_register_addr = (FPGA_rapidio_REGISTERS *)uio_w2.mem_ptr;
    ptr_ddr_w3 = (ddr_w3_t *)uio_w3.mem_ptr;
    ptr_ddr_w4 = (ddr_w4_t *)uio_w4.mem_ptr;
    ptr_wet_fpga_register_data = (WET_FPGA_REGISTERS *)uio_wet_fpga_register.mem_ptr;
    ptr_mark_registers = (char *)uio_make_fpga_register.mem_ptr;

    fd = open(SYSFS_GPIO_EXPORT, O_WRONLY);
    if (fd == -1)
    {
        printf("ERR: export open error.\n");
    }
    else
    {
        write(fd, SYSFS_GPIO_RUN_LIGHT, sizeof(SYSFS_GPIO_RUN_LIGHT));
        close(fd);
    }

    return 0;
}
