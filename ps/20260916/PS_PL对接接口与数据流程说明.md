# PS/PL 波束形成接口与数据流程对接说明

**适用工程**：`20260916`  
**适用设备树**：`system.dts`  
**整理日期**：2026-09-20  
**文档用途**：用于 PS 软件与 PL、DSP 工程师联调时确认地址、寄存器、数据格式、时序、所有权和故障定位方法。

## 0. 阅读说明

本文按当前 PS 工程实际代码和当前设备树整理，不替代 PL 设计文档。内容分为三类：

- **当前已实现**：`20260916` 中已经由代码执行的操作。
- **当前约定**：PS 根据现有协议或参考代码采用的解释。
- **需要确认**：需要 PL/DSP 确认后才能冻结的字段、时序或协议。

出现问题时，先按本文的“故障定位”章节判断问题属于：Linux/设备树、UIO 映射、PS 参数、DMA、CBF、W2/SRIO、DSP/NREAD、还是显控发送链路。

---

## 1. 系统边界与总体流程

### 1.1 业务目标

湿端通过 TCP 向干端发送一帧 IQ 数据及传感器数据。干端 PS 接收并解析湿端数据，准备 IQ 输入区和 Frame 参数区，然后由 PL 完成后续帧生成、波束形成、DSP/SRIO 传输和 NREAD 回读。干端另一个线程等待 FPGA 通知，再从指定 DDR 区读取结果并发送给显控。

### 1.2 当前 PS 主流程

```text
显控设置参数
    -> PS 通过 TCP 下发/保存干端参数
    -> 干端根据参数向湿端发送采集配置
    -> wet_to_dry() 接收湿端一帧数据
    -> RevIQData_SetIQDataParaHead() / 湿端数据解析
    -> Beam_Down_Data_Manage()
        -> 解析当前湿端帧中的传感器、姿态和时间信息
        -> Set_SoundSpeed_Value()
        -> Set_Fpga_To_Dsp_Para()
        -> Set_Dsp_TransBuf()
            -> 取 IQData_Last + 128 作为 PL 的纯 IQ 输入
            -> IQA/IQB 乒乓写入 DDR
            -> 构造 Frame 的 PS 参数区
            -> 配置 W2、CFG、CBF、AXI DMA
            -> 写 CBF CONTROL.START
            -> 等待 AXI DMA 当前帧完成
    -> PL/DSP/SRIO 处理
    -> PL 产生 CBF/NREAD 完成中断
    -> dry_to_upper() 作为唯一读取者消费 UIO 中断
    -> dry_to_upper() 读取结果 DDR
    -> 拆包/组合/发送显控
```

### 1.3 当前帧关系

当前工程有意采用如下时序：

```text
IQ 输入：        IQData_Last + 128，即上一帧湿端数据的纯 IQ 载荷
Frame/参数：      当前帧解析得到的配置、传感器和姿态参数
```

因此当前逻辑是“上一帧 IQ + 当前帧 Frame/传感器参数”。这不是代码笔误，而是现有湿端接收和处理流程形成的业务约定；但 PL/DSP 必须确认这种跨帧组合是否符合算法要求。如果算法要求 IQ 与参数严格同帧，需要在 PS 侧改成同一帧快照，不能只修改 PL 寄存器。

---

## 2. 物理地址、设备树与 UIO 映射

### 2.1 DDR 和寄存器总体地址表

| 逻辑资源 | 物理基地址 | DTS `reg` 大小 | 当前 UIO 映射实际大小 | 程序对象/用途 | 主要写入方 |
|---|---:|---:|---:|---|---|
| IQA | `0x20000000` | `0x02000000` | `0x02000000` | `uio_iq_a`，纯 IQ 输入 | PS 写，PL 读 |
| IQB | `0x22000000` | `0x02000000` | `0x02000000` | `uio_iq_b`，纯 IQ 输入 | PS 写，PL 读 |
| Frame | `0x24000000` | `0x0BC00000` | `0x0BC00000` | `uio_frame`，PS 头和 PL 输出 | PS/PL 分区写 |
| NREAD | `0x30000000` | `0x02000000` | `0x02000000` | `uio_nread`，DSP/SRIO 回读结果 | PL/DSP 写，PS 读 |
| BD | `0x3FF00000` | `0x00010000` | `0x00010000` | `uio_bd`，AXI DMA SG 描述符 | PS 写，DMA 读写 |
| TVG | `0x40000000` | `0x00004000` | `0x00004000` | `uio_tvg`，传感器/TVG 配置缓冲区 | PS 写/读 |
| AXI DMA | `0x40400000` | `0x00000060` | 通常为 `0x00001000` | `uio_dma`，S2MM DMA 控制寄存器 | PS 配置，DMA 更新 |
| FPGA/CBF 页 | `0x43C00000` | `0x00001000` | `0x00001000` | `uio_fpga_cbf` | PS/PL |
| W2/CFG 页 | `0x43C10000` | `0x00001000` | `0x00001000` | `uio_w2_cfg` | PS/PL |
| WET/W3 页 | `0x43C20000` | `0x00001000` | `0x00001000` | `uio_wet_w3` | PS/PL |
| MAKE/W4 页 | `0x43C30000` | `0x00001000` | `0x00001000` | `uio_make_w4` | PS/PL |

### 2.2 同一 4 KB 页的逻辑窗口

Linux UIO 和 `mmap()` 通常按页映射。当前 DTS 已将同一页内的两个 0x400 字节窗口合并成一个 0x1000 字节 UIO 节点，程序再通过页内偏移取得逻辑指针。

| UIO 基页 | 基窗口 | 逻辑子窗口 | 页内偏移 |
|---:|---:|---:|---:|
| `0x43C00000` | FPGA 基寄存器 | CBF `0x43C00400` | `+0x400` |
| `0x43C10000` | W2 | CFG `0x43C10400` | `+0x400` |
| `0x43C20000` | 湿端 FPGA 基寄存器 | W3 `0x43C20400` | `+0x400` |
| `0x43C30000` | 组帧 FPGA 基寄存器 | W4 `0x43C30400` | `+0x400` |

因此：

```text
ptr_ddr_cbf = uio_fpga_cbf.mem_ptr + 0x400
ptr_ddr_cfg = uio_w2_cfg.mem_ptr   + 0x400
ptr_ddr_w3  = uio_wet_w3.mem_ptr   + 0x400
ptr_ddr_w4  = uio_make_w4.mem_ptr  + 0x400
```

不能再把 `0x43C00400` 单独作为一个 UIO 节点，否则内核页对齐后可能与 `0x43C00000` 节点冲突。`0x40400000` 的 DTS 资源只有 0x60 字节，但 UIO 可能显示为 0x1000，这是页对齐后的映射长度，程序只使用 DMA 结构定义的有效寄存器范围。

### 2.3 UIO 设备编号

程序现在按物理地址查找 `/sys/class/uio/uioX/maps/map0/addr` 和 `size`，不应把 `/dev/uio0`、`/dev/uio7` 等编号当成永久 ABI。不同设备树、内核启动顺序或新增 UIO 节点都可能改变编号。

启动时应记录：

```text
/sys/class/uio/uioX/name
/sys/class/uio/uioX/maps/map0/addr
/sys/class/uio/uioX/maps/map0/size
```

### 2.4 当前中断节点

当前 CBF/PL 业务中断配置为：

```dts
interrupt-parent = <0x4>;       /* Zynq GIC */
interrupts = <0x0 0x1d 0x4>;     /* SPI 0x1D，level-high */
```

它挂在 `uio_fpga_cbf@43c00000` 节点上，因此寄存器页和中断使用同一个 UIO fd。当前代码中 `uio_cfb.fd` 是该合并 UIO 页对应的 fd，CBF 逻辑寄存器指针则是同一映射的 `+0x400`。


---

## 3. AXI DMA SG 描述符结构

BD 基地址：`0x3FF00000`，每个 BD 64 字节，当前定义为：

| BD 偏移 | 字段 | 说明 |
|---:|---|---|
| `+0x00` | `next_desc_low` | 下一个 BD 物理地址低 32 位 |
| `+0x04` | `next_desc_high` | 下一个 BD 物理地址高 32 位，当前为 0 |
| `+0x08` | `buffer_addr_low` | S2MM 写入目标物理地址低 32 位 |
| `+0x0C` | `buffer_addr_high` | 目标地址高 32 位，当前为 0 |
| `+0x10` | `res[0]` | 保留 |
| `+0x14` | `res[1]` | 保留 |
| `+0x18` | `control_btt` | 低 23 位为本 BD 搬移字节数 |
| `+0x1C` | `status` | DMA 完成后由硬件回写；bit31 为完成，低位包含实际传输信息 |
| `+0x20..+0x3F` | `res2[8]` | 保留 |

当前 PS 的配置方式：

1. 依据 `dma_bytes` 计算 BD 数量。
2. 每个 BD 的目标地址为 `DDR_FRAME_ADDR + offset`。
3. 最后一个 BD 回环到第一个 BD。
4. `CURDESC` 指向 `0x3FF00000`。
5. `TAILDESC` 指向最后一个 BD。
6. CBF START 后，PL 产生的数据进入 AXI DMA S2MM，再由 DMA 写入 Frame DDR。

PL 侧需要确认：

- SG BD 链表是否要求物理连续、是否允许当前回环形式。
- `control_btt` 的最大长度、对齐要求和 status 位定义是否与当前代码一致。
- DMA 写入 Frame 的起始地址是否确实是 `0x24000000`。
- DMA 完成是否只表示 PL Frame 已写入，还是还代表 DSP/NREAD 已经完成；当前 PS 将二者视为不同阶段。

---

## 4. AXI DMA S2MM 控制寄存器

DMA 基地址：`0x40400000`。当前结构从资源偏移 `0x30` 开始：

| 偏移 | 寄存器 | 当前 PS 用法 |
|---:|---|---|
| `+0x30` | `S2MM_DMACR` | 写 RESET、RUN/STOP、IOC 中断使能 |
| `+0x34` | `S2MM_DMASR` | 先读原始状态，再检查错误，最后 W1C 清状态 |
| `+0x38` | `S2MM_CURDESC_L` | 写 BD 首地址低 32 位 |
| `+0x3C` | `S2MM_CURDESC_H` | 写 BD 首地址高 32 位 |
| `+0x40` | `S2MM_TAILDESC_L` | 写最后一个 BD 地址低 32 位 |
| `+0x44` | `S2MM_TAILDESC_H` | 写最后一个 BD 地址高 32 位 |

当前操作顺序：

```text
清/复位 DMA
 -> 等待 RESET 清零
 -> 读取并检查旧 DMASR
 -> 清除旧状态
 -> 写 CURDESC
 -> 设置 RUNSTOP 和 IOC
 -> 写 TAILDESC
 -> 等待 CBF START 后的 BD 完成
```

DMA 状态错误时，必须保留原始 `DMASR` 打印出来。定位 DMA 问题时不能先写 W1C 再读取，否则可能丢失错误现场。

---

## 5. CBF 寄存器结构

CBF 逻辑基地址：`0x43C00400`，对应 UIO 页 `0x43C00000 + 0x400`。当前代码结构如下。

### 5.1 识别、控制和输入参数

| 偏移 | 字段 | 方向 | 当前含义 |
|---:|---|---|---|
| `+0x00` | `MAGIC` | PL -> PS | 应为 `0x43424633` |
| `+0x04` | `VERSION` | PL -> PS | CBF 版本 |
| `+0x08` | `BUILD_ID` | PL <-> PS | 当前代码期望 `0x124` |
| `+0x0C` | `CONTROL` | PS -> PL | bit0 START，bit1 SOFT_RESET，bit2 CLEAR_ERRORS |
| `+0x10` | `STATUS` | PL -> PS | BUSY/DONE/ERROR 状态 |
| `+0x14` | `IQ_POINTS` | PS -> PL | IQ 点数，当前按 `IQ_Data_Length / 832` |
| `+0x18` | `ROLL_INTERVAL` | PS -> PL | 横摇更新间隔，当前代码需与 PL 确认语义 |
| `+0x1C` | `HISTORY_DELAY` | PL -> PS | 当前参考值为 10 |
| `+0x20` | `INPUT_BASE` | PS -> PL | IQA 或 IQB 物理地址 |
| `+0x24` | `INPUT_BYTES` | PS -> PL | 纯 IQ 字节数，不含前 128 字节湿端头 |
| `+0x28` | `FRAME_ID` | PS -> PL | 当前取湿端参数头的 `PingCount` |
| `+0x2C` | `INPUT_FORMAT` | PS -> PL | 当前写 `0x00011010` |

### 5.2 输出、错误和运行状态

| 偏移 | 字段 | 方向 | 含义 |
|---:|---|---|---|
| `+0x30` | `OUTPUT_VALID_SNAPSHOTS` | PL -> PS | 有效输出快照数 |
| `+0x34` | `OUTPUT_VALID_BYTES` | PL -> PS | 有效输出字节数 |
| `+0x38` | `ERROR_STATUS` | PL -> PS/W1C | CBF 错误状态 |
| `+0x3C` | `CURRENT_SNAPSHOT` | PL -> PS | 当前处理进度 |
| `+0x40` | `CURRENT_ROLL_INDEX` | PL -> PS | 当前横摇组索引 |
| `+0x44` | `ACTIVE_ROLL_Q29` | PL -> PS | 当前有效横摇值 |
| `+0x48` | `SRIO_TARGET_ADDR` | PS -> PL | 兼容/调试字段，需 PL 确认是否仍使用 |
| `+0x4C` | `SRIO_TRANSFER_BYTES` | PL -> PS | 实际 SRIO 发送长度 |
| `+0x50` | `DEBUG_CAPTURE_BASE` | PS -> PL | 调试捕获起点 |
| `+0x54` | `DEBUG_CAPTURE_LIMIT` | PS -> PL | 调试捕获上限 |
| `+0x58` | `SATURATION_COUNT` | PL -> PS | 饱和计数 |
| `+0x60` | `PARAM_GEN_CYCLES` | PL -> PS | 参数生成耗时 |
| `+0x64` | `FRAME_CYCLES` | PL -> PS | 本帧处理周期数 |
| `+0x68` | `DROP_COUNT` | PL -> PS | 结果丢失计数 |
| `+0x6C` | `ROLL_SIGN` | PS -> PL | 横摇正负方向，当前写 0 |

### 5.3 中断和 NREAD 诊断寄存器

| 偏移 | 字段 | 方向 | 含义 |
|---:|---|---|---|
| `+0x70` | `IRQ_STATUS` | PS/PL | 写 1 清中断状态 |
| `+0x74` | `IRQ_MASK` | PS -> PL | 当前代码写 1 开启约定的 CBF/NREAD 中断 |
| `+0x78` | `READER_DEBUG` | PL -> PS | IQ Reader 状态 |
| `+0x7C` | `SR_LINK_STATUS` | PL -> PS | SRIO 链路状态；当前代码检查高两位 |
| `+0x80` | `READER_ARADDR` | PL -> PS | 最近一次 AXI 读地址 |
| `+0x84` | `READER_AXI_STATUS` | PL -> PS | 最近一次 AXI 响应 |
| `+0x88` | `NREAD_EVENT_COUNT` | PL -> PS | NREAD 成功累计次数 |
| `+0x8C` | `NREAD_LAST_INFO` | PL -> PS | 最近一次 NREAD 的状态/Bank/长度提示 |
| `+0x90` | `NREAD_ERR_COUNT` | PL -> PS | NREAD 错误累计次数 |
| `+0x94` | `NREAD_LAST_BYTES` | PL -> PS | 最近一次成功 NREAD 字节数 |
| `+0x98` | `NREAD_OVERRUN_CNT` | PL -> PS | NREAD 覆盖累计次数 |
| `+0x9C` | `FIFO_OVF_COUNT` | PL -> PS | FIFO 溢出累计次数 |
| `+0xA0` | `FIFO_OVF_FLAG` | PL -> PS | FIFO 溢出标志 |

### 5.4 角度表接口

| 偏移 | 字段 | 方向 | 含义 |
|---:|---|---|---|
| `+0x100` | `ANGLE_INDEX` | PS -> PL | 角度表索引 0..255 |
| `+0x104` | `ANGLE_Q29` | PS -> PL | signed Q3.29，单位 rad |
| `+0x108` | `ANGLE_COMMAND` | PS -> PL | bit0 提交角度表写入 |
| `+0x10C` | `ANGLE_READBACK` | PL -> PS | 角度表读回 |

当前 `20260916` 主流程没有完成 sinc/apod/角度表的全部逐帧写入，相关寄存器是否由 PL 预加载、由上电初始化还是由 PS 每帧提交，必须由 PL/DSP 明确。

---

## 6. W2、CFG、W3、W4 寄存器

### 6.1 W2：发送和回读配置

W2 基地址：`0x43C10000`。

| 偏移 | 字段 | 当前 PS 写入/用途 |
|---:|---|---|
| `+0x00` | `DDR_PR` | 地址更新脉冲，当前按 0 -> 1 -> 0 使用 |
| `+0x08` | `SEND_BYTE_CNT_0` | 写 `frame_bytes` |
| `+0x0C` | `SENDR_BASE_ADDR_0` | 写 `DDR_FRAME_ADDR` |
| `+0x10` | `RECEIVE_BASE_ADDR_0` | 写 `DDR_NREAD_ADDR` |
| `+0x14` | `SEND_START_0` | 当前 S231 流程保持 0，由 PL/协议触发 |
| `+0x18` | `SEND_TARGET_ADDR_0` | 当前写 DSP Bank0 `0x80000000` |
| `+0x1C` | `SEND_BYTE_CNT_1` | 写 `frame_bytes` |
| `+0x20` | `SENDR_BASE_ADDR_1` | 写 `DDR_FRAME_ADDR` |
| `+0x24` | `RECEIVE_BASE_ADDR_1` | 写 `DDR_NREAD_ADDR` |
| `+0x28` | `SEND_START_1` | 当前 S231 流程保持 0 |
| `+0x2C` | `SEND_TARGET_ADDR_1` | 当前写 DSP Bank1 `0x84000000` |
| `+0x30` | `IQ_BASE_ADDR` | 当前不使用 |
| `+0x54` | `READ_TARGET_ADDR_0` | 当前写 DSP Bank0 |
| `+0x58` | `READ_TARGET_ADDR_1` | 当前写 DSP Bank1 |
| `+0x5C` | `READ_BYTE_CNT_0` | 写 `read_bytes` |

W2 配置完成后，PS 对 `DDR_PR` 产生更新脉冲。需要确认 PL 对两个 Bank 的实际发送/回读时序，以及 `SEND_START_0/1` 保持 0 是否是最终协议要求。

### 6.2 CFG：动态参数和配置表接口

CFG 逻辑基地址：`0x43C10400`，对应 W2 页 `+0x400`。

| 偏移 | 字段 | 当前含义 |
|---:|---|---|
| `+0x00` | `roll_wdata` | 横摇表/动态横摇写数据 |
| `+0x04` | `roll_addr` | 横摇地址 |
| `+0x08` | `roll_we` | 横摇写使能/脉冲 |
| `+0x0C..+0x20` | `sinc_coeff[6]` | Sinc 系数窗口 |
| `+0x24` | `sinc_addr` | Sinc 地址 |
| `+0x28` | `sinc_we` | Sinc 写使能 |
| `+0x2C` | `full_coeff` | Full Apod 系数 |
| `+0x30` | `full_addr` | Full Apod 地址 |
| `+0x34` | `full_we` | Full Apod 写使能 |
| `+0x38` | `sub_coeff` | Sub Apod 系数 |
| `+0x3C` | `sub_addr` | Sub Apod 地址 |
| `+0x40` | `sub_we` | Sub Apod 写使能 |
| `+0x44` | `roll_update_q29` | 当前帧动态横摇 Q29 值 |
| `+0x48` | `roll_update_source` | 横摇来源选择 |
| `+0x4C` | `roll_update_push` | 写 1 提交当前横摇值 |
| `+0x50` | `frequency_code` | 频率编码，Hz 到编码的转换尚未确认 |

当前 PS 每帧明确写入的是动态横摇接口；sinc、full apod、sub apod、角度表和频率编码尚未形成完整的已确认写入流程。

### 6.3 W3 和 W4

W3 逻辑基地址：`0x43C20400`，W4 逻辑基地址：`0x43C30400`。

W3 当前结构：

| 偏移 | 字段 | 语义 |
|---:|---|---|
| `+0x00` | `W3_MAGIC` | W3 识别字 |
| `+0x04` | `SAMPLE_FS_HZ` | IQ 有效采样率元数据 |
| `+0x08` | `CARRIER_FC_HZ` | 中心频率元数据 |
| `+0x0C` | `WORK_MODE` | 工作模式 |
| `+0x10` | `EXT_STATUS` | 扩展状态 |

W4 是参数镜像窗口，当前包括：

```text
+0x000..+0x008   MIR2、条目数、写入次数
+0x010..+0x02C   W1_CONTROL、IQ_POINTS、ROLL_INTERVAL、INPUT_BASE、INPUT_BYTES、FRAME_ID、FORMAT
+0x048           W1_SRIO_TARGET_ADDR
+0x06C           W1_ROLL_SIGN
+0x074           W1_IRQ_MASK
+0x100           W1_ANGLE_INDEX
+0x104           W1_ANGLE_Q29
+0x200..+0x24F   W2_CFG[20]
+0x300..+0x30C   W3 采样率、中心频率、工作模式、扩展状态
```

当前 PS 只验证 W3/W4 映射窗口，不把未确认的 W3/W4 字段当作有效配置写入。PL 工程师需要提供每个字段的：写入时机、提交脉冲、复位值、读写属性、是否必须每帧写入。

---

## 7. Frame DDR 数据格式和写权限

Frame 基地址：`FRAME_BASE = 0x24000000`。

### 7.1 Frame 总体分区

| 地址范围 | 长度 | 内容 | 负责写入方 |
|---|---:|---|---|
| `frame + 0 .. +11` | 12 B | PL 生成的帧前缀/头部 | PL |
| `frame + 12 .. +271` | 260 B | PS 参数区 `frame_Parameter_t` | PS |
| `frame + 272 .. +511` | 240 B | `CH_error[60]` | PS |
| `frame + 512 .. +2111` | 1600 B | `Roll_Time_Diff[400]` | PS |
| `frame + 2112 .. +3711` | 1600 B | `Roll_Sequential_Value[400]` | PS |
| `frame + 3712` 以后 | 由协议确定 | PL 生成的波束/结果区域及尾部 | PL/PL 侧 DMA |

PS 侧 `frame_t` 的大小为 3700 字节，写入起点是 `frame + 12`，因此正好覆盖 `+12..+3711`。PS 不应清零或覆盖 `+0..+11` 和 `+3712` 以后的 PL 区域。

### 7.2 Frame 参数区的重要字段

`frame_Parameter_t` 的开头字段包括：

```text
word 0    head
byte 4    BASH_TEST
byte 5    SONAR_SHOW
byte 6    SIDE_MOD
byte 7    PTD_MOD
...
word/float 字段按 frame_Parameter_t 的 C 结构顺序排列
end_tail[6] 写入 0xEEEEEEEE
rsvd 保留
```

需要特别注意：当前协议说明 `frame + 60` 位置由 PL 覆盖为当前有效声速。因此 PS 可以按照结构体正常构造初始值，但不能把该位置视为最终声速，也不能在 PL 运行后再次覆盖该字段。最终有效声速应由 PL 的寄存器/Frame 回写定义确认。

### 7.3 Frame 长度与 DMA 长度

当前 PS 将协议有效 Frame 长度、DMA 对齐长度、NREAD 回读长度分开计算：

```text
frame_bytes = 协议有效 Frame 长度
dma_bytes   = frame_bytes 按 DMA 要求对齐后的长度
read_bytes  = DSP/NREAD 协议要求的回读长度
```

不能用 `dma_bytes` 代替 `frame_bytes` 写入协议头，也不能用 `frame_bytes` 代替 DMA 的对齐长度。PL 需要确认最后的填充字节是否允许被忽略，以及 `NREAD_LAST_BYTES` 应返回有效长度还是对齐长度。

---

## 8. IQ 和湿端传感器数据布局

### 8.1 湿端原始接收区

湿端接收到的完整数据保存在 PS 用户态缓冲区 `IQData`。前 128 字节是湿端参数头：

```text
IQData + 0                       湿端参数头，SIZE_OF_WET_SONAR_FIRST = 128
IQData + 128                     IQ 声学数据起点
IQData + 128 + IQ_Data_Length    后续传感器区起点前的长度边界
```

当前工程约定：

- `RECV_WET_SONAR_FIRST.IQ_Data_Length` 表示 IQ 声学数据长度，不是传感器数据长度。
- 送入 PL 的纯 IQ 起点是 `IQData_Last + 128`。
- 传感器数据解析使用 `IQData + 128 + 8 + IQ_Data_Length` 及其后续结构偏移。
- IQ 数据长度应满足当前 PL 点格式要求，当前代码按 `832` 字节/点计算 `IQ_POINTS`。

### 8.2 传入 PL 的 IQ

当前 PS 不把湿端 128 字节参数头和传感器区复制到 IQA/IQB，而只复制：

```text
源：IQData_Last + SIZE_OF_WET_SONAR_FIRST
长：IQ_Data_Length
目标：IQA 0x20000000 或 IQB 0x22000000
```

因此 CBF 的两个关键输入必须一致：

```text
INPUT_BASE  = 当前 IQA/IQB 物理地址
INPUT_BYTES = IQ_Data_Length，不包含 128 字节参数头
IQ_POINTS   = IQ_Data_Length / 832
```

任何一个字段不一致，都可能表现为 PL 读地址错误、点数错误、帧长度错误或波束结果异常。

### 8.3 传感器数据

传感器数量和传感器数组从 IQ 数据尾部解析。PS 需要在使用计数计算地址前验证：

```text
128 + 8 + IQ_Data_Length + 传感器区长度 <= 实际接收长度
```

当前旧代码中多个传感器计数和数组偏移使用 `+4`、`+8` 以及每条传感器记录长度，PL 不直接读取这些湿端传感器字节；PL 主要接收 IQA/IQB 和 Frame 中 PS 已整理好的参数/横摇数组。

---

## 9. PS 侧逐帧执行顺序

下面是当前 `Set_Dsp_TransBuf()` 的实际顺序。PL 工程师可以用它与波形、寄存器抓取和仿真时序对照。

1. 根据 `Bd_Date_TransBuf_Flag` 选择 IQA 或 IQB。
2. 从 `IQData_Last + 128` 取得纯 IQ 地址和 `IQ_Data_Length`。
3. 计算 `IQ_POINTS`、`frame_bytes`、`dma_bytes`、`beam_bytes` 和 `read_bytes`。
4. 检查 CBF MAGIC、CBF BUSY/ERROR 和 SRIO 链路状态。
5. 将纯 IQ 数据复制到选中的 IQA/IQB DDR。
6. 在 `0x24000000 + 12` 写入 3700 字节 PS Frame 区。
7. 清理本帧 NREAD 区域，便于区分本帧新数据和旧数据。
8. 写 CFG 当前已确认的动态横摇参数，并产生提交脉冲。
9. 检查 W3/W4 映射窗口；当前未写入未确认字段。
10. 写 W2 的 Frame 发送地址、NREAD 接收地址、DSP Bank 地址和长度。
11. 写 CBF 的 IQ 物理基地址、IQ 字节数、IQ 点数、帧号和输入格式。
12. 填写 BD 链表，配置 AXI DMA S2MM。
13. 清 CBF 中断状态、设置 IRQ_MASK，并向 UIO fd 写 1，确保启动前 Linux UIO 中断处于使能状态。这个动作只做“预使能”，不读取或消费完成中断。
14. 写 `CBF.CONTROL = START`，触发 PL 读取 IQ 并开始处理。
15. 轮询 DMA DMASR 和所有 BD status，等待当前 Frame 的 S2MM 传输完成。
16. DMA 成功后才切换 IQA/IQB 乒乓标志。
17. `dry_to_upper()` 在线程中阻塞读取同一个 `uio_cfb.fd`；PL 完成并产生中断后，由它读取中断事件、重新使能 UIO，再读取 NREAD 并发送显控。

### 9.1 这段顺序的边界

“DMA 完成”只说明当前 S2MM 传输阶段完成，不能自动等价于：

```text
CBF 算法完成
 -> SRIO 发送完成
 -> DSP 回读完成
 -> NREAD 数据有效
 -> dry_to_upper() 已经读取并发送显控
```

当前工程还需要通过 CBF 的 `NREAD_EVENT_COUNT`、`NREAD_LAST_BYTES`、`NREAD_ERR_COUNT` 和 IRQ 行为确认后半段链路。单个 Frame、单个 NREAD 区和单个 BD 区的复用必须由“上一帧结果已被消费”来约束，不能只依赖 DMA 完成。

---

## 10. PL 侧期望执行流程

PL 侧可按以下逻辑核对当前 PS 写入内容：

1. 复位后提供正确的 CBF `MAGIC`、`VERSION`、`BUILD_ID`。
2. 在 CBF `STATUS` 中反映 BUSY、DONE、ERROR。
3. 读取 `INPUT_BASE` 和 `INPUT_BYTES`，从 IQA/IQB 读取纯 IQ。
4. 按 `IQ_POINTS` 和 `INPUT_FORMAT` 解释 IQ 数据。
5. 读取 Frame `+12..+3711` 的 PS 参数区。
6. 根据协议覆盖 PL 所有权区域，例如 `frame + 60` 的最终有效声速。
7. 读取 CFG 当前帧动态横摇参数及其提交事件。
8. 根据约定使用预加载或当前帧更新的 Sinc、Apod、角度和频率参数。
9. 生成 Frame 的 PL 区域和波束记录。
10. 通过 AXI-Stream/S2MM 将 PL 生成结果送到 DMA，DMA 写入 `0x24000000` 对应 Frame DDR。
11. 按 W2 配置向 DSP Bank0/Bank1 发送 Frame。
12. 接收 DSP/SRIO 回读结果，写入 `0x30000000` NREAD DDR。
13. 更新 NREAD 计数、长度、错误和溢出诊断寄存器。
14. 产生 GIC SPI `0x1D` 对应的 UIO 中断。

PL 需要明确中断产生的准确时点：是 DMA 完成、CBF 完成、DSP NREAD 完成，还是多个事件共用一个中断。PS 目前 `dry_to_upper()` 把该中断作为结果读取触发，因此这个定义必须冻结。

---

## 11. 中断和 UIO 所有权

### 11.1 当前中断处理

当前 `dry_to_upper()` 使用 `uio_cfb.fd`：

```text
read(uio_cfb.fd, &icount, 4)       等待 UIO 中断
write(uio_cfb.fd, &irq_on, 4)      重新使能 UIO 中断
从 uio_nread.mem_ptr 读取结果
```

`Set_Dsp_TransBuf()` 中的 `S231_PrepareIrq()` 也使用相同 fd 做中断清除和重新使能。

### 11.2 当前代码中的准确职责和风险

当前代码并不是两个线程都调用 `read()` 读取同一个中断。准确情况是：

- `dry_to_upper()` 是完成中断的主要消费者：调用 `read(uio_cfb.fd, ...)` 等待中断，随后写 1 重新使能，再读取 NREAD。
- `S231_PrepareIrq()` 不读取中断，只在 CBF START 前清 `CBF.IRQ_STATUS`、设置 `CBF.IRQ_MASK`，并向 UIO fd 写 1 做启动前预使能。

因此，1.2 的主流程应理解为：`Set_Dsp_TransBuf()` 负责“准备并启动”，`dry_to_upper()` 负责“等待完成中断、读取结果并发送”。`S231_PrepareIrq()` 不是结果处理函数，也不会替代 `dry_to_upper()`。

仍需注意的是，两个位置都在控制同一个 UIO fd 的使能状态。联调时必须保证：

1. `dry_to_upper()` 是唯一的中断读取者；
2. `S231_PrepareIrq()` 只能在 START 前执行，不能在结果已经到达后再次清除状态；
3. `dry_to_upper()` 读取中断后负责正常的“读事件 -> 写 1 重新使能”闭环；
4. 如果实测发现启动前写 1 与结果线程的重新使能发生竞争，应把 UIO fd 的使能操作也统一到中断线程，或改成独立的 DMA/CBF IRQ。

在没有确认 IRQ_STATUS、UIO 使能和 START 时序前，不能只根据“中断能进来”判断 NREAD 数据已经有效，还要同时检查 `NREAD_EVENT_COUNT` 和 `NREAD_LAST_BYTES`。

---

## 12. 显控结果读取链路的当前兼容性

当前 `dry_to_upper()` 保留了旧业务处理逻辑：

- 收到 FPGA 中断后从 `uio_nread.mem_ptr` 读取结果。
- 先检查旧协议头 `@@ST`。
- 读取旧结构中的 `All_lenth`。
- 按旧长度复制到 `Image_Data` 后发送显控。

新 S231 Frame/NREAD 协议包含 `@@@@`、Frame 长度、波束数据长度、3700 字节 PS 头、PL 波束记录和 `<<ST/IQ/$$` 等字段。两套协议的头部和长度解释并不天然相同。

因此当前必须与 PL/DSP/显控共同确认：

- `0x30000000` 中实际写的是旧 `@@ST` 数据，还是新 S231 NREAD 数据。
- 新 NREAD 的有效起点、有效长度和尾部格式。
- `dry_to_upper()` 是否仍由 CBF 中断触发。
- NREAD 结果是单帧覆盖、双 Bank 轮换，还是需要 PS 自己按 Bank 选择地址。

如果这些问题没有确认，PS 端即使 DMA、CBF 和 SRIO 都正常，显控仍可能因为包头或长度不匹配收不到数据。

---

## 13. 故障定位矩阵

| 现象 | 优先检查 | 典型定位点 |
|---|---|---|
| 程序启动提示 UIO 映射失败 | DTS 节点、`compatible`、UIO 驱动、map0 地址/大小 | `/sys/class/uio/uioX/maps/map0/{addr,size}`、`dmesg` |
| `0x40400000` 显示 0x1000 而不是 0x60 | 页对齐，不一定是错误 | 程序应按页对齐校验，实际只访问 `+0x30..+0x44` |
| `0x43C00400` 找不到独立 UIO | 同页窗口被合并 | 使用 `uio_fpga_cbf` 映射并加 `+0x400` |
| CBF MAGIC 错误 | CBF 偏移、bit 版本、UIO 映射错误 | 读取 `0x43C00400 + 0x00` |
| BUILD_ID 不符 | PS 与 PL 工程版本不一致 | 读取 CBF `+0x08`，当前期望 `0x124` |
| CBF 一直 BUSY | 上一帧未结束、START 重复、PL 卡死 | CBF `+0x10`、`+0x3C`、`+0x64` |
| SRIO 不通 | 链路、Bank 地址、W2 长度/地址 | CBF `+0x7C`、W2 `+0x18/+0x2C` |
| DMA 不完成 | DMA reset、CURDESC/TAILDESC、BD 地址、AXI Stream 无数据 | DMA `+0x30/+0x34`、BD `status` |
| DMA status 报错 | 先保存原始 DMASR，不要先 W1C | `S2MM_DMASR` 错误位、BD status |
| Frame 头正确但波束区域为空 | PL 没有产生流、DMA 目标不对、BD 未完成 | `0x24000000 + 3712`、DMA/CBF 状态 |
| NREAD 区仍是清理填充值 | DSP 未回读、W2 回读长度/地址错、NREAD 事件未发生 | `NREAD_EVENT_COUNT`、`NREAD_LAST_BYTES` |
| NREAD 计数增加但显控无数据 | `dry_to_upper()` 头部/长度协议不匹配 | 检查 `0x30000000` 前 32 字节和实际长度 |
| IQ 结果异常但 DMA 正常 | IQ 起点、长度、点数、格式不一致 | `INPUT_BASE`、`INPUT_BYTES`、`IQ_POINTS` |
| 每隔一帧异常 | IQA/IQB 所有权或 Frame/NREAD 尚未消费就复用 | 记录当前 IQ Bank、CBF frame id、NREAD 计数 |
| 参数生效滞后一帧 | 当前采用上一帧 IQ + 当前帧参数 | 对照 `PingCount`、Frame ID、湿端帧号 |
| I2C0 全部访问超时 | pinmux、SDA/SCL 拉低、上拉、电源、I2C IRQ | `dmesg`、`i2cdetect -y 0`、示波器、`/proc/interrupts` |
| NST175 单独地址也超时 | 先排除总线控制器问题，再排除 0x4E 地址 | I2C0 状态和波形，不要只改从地址 |

### 13.1 建议的现场日志

每一帧至少记录：

```text
PingCount / FRAME_ID
IQ bank 和 INPUT_BASE
IQ_Data_Length / IQ_POINTS
frame_bytes / dma_bytes / read_bytes
CBF STATUS / ERROR_STATUS / SR_LINK_STATUS
DMA DMASR
每个 BD status
NREAD_EVENT_COUNT / NREAD_LAST_BYTES / NREAD_ERR_COUNT
NREAD 区前 32 字节
实际发送给显控的字节数
```

有了这组日志，基本可以判断故障是在“湿端输入之前、PS 构帧之前、PL 处理期间、DMA 写 Frame、SRIO/DSP 回读、还是显控发送”。

---

## 14. 当前仍需 PL/DSP 冻结的接口问题

以下内容建议在 PS/PL 联调会上逐项确认并形成版本号：

1. CBF `BUILD_ID` 当前是否固定为 `0x124`，不匹配时是否应禁止启动。
2. `IQ_POINTS = IQ_Data_Length / 832` 是否是最终定义，832 字节中每个点的通道排列是什么。
3. `INPUT_BYTES` 是否只包含纯 IQ，是否要求 8/256 字节对齐。
4. `ROLL_INTERVAL` 的单位和来源，是否为 IQ 点数加一，还是来自上位机配置。
5. CFG 的 sinc、full apod、sub apod、angle、frequency 是否上电加载、配置变化时加载，还是每帧加载。
6. W3/W4 每个字段的写入权限、提交脉冲、复位值和每帧要求。
7. `frame + 60` 被 PL 覆盖的时机，以及最终有效声速的读取/验证方式。
8. W2 的两个 DSP Bank 是否都必须每帧写，`SEND_START_0/1` 为什么保持 0。
9. DMA 完成、CBF DONE、NREAD 完成是否共用一个 IRQ，IRQ_STATUS 的 bit 定义是什么。
10. `NREAD_LAST_BYTES` 返回有效字节数还是 256 字节对齐后的搬移长度。
11. NREAD 的实际包头、有效地址、尾部和显控发送协议是否仍兼容旧 `dry_to_upper()`。
12. 单帧 Frame、单帧 NREAD、单组 BD 的复用条件，以及 PL 是否允许下一帧覆盖上一帧资源。
13. 当前“上一帧 IQ + 当前帧 Frame/传感器参数”是否为最终算法时序。

---

## 15. 推荐联调顺序

### 阶段一：只验证 Linux 和地址

1. 确认所有 UIO 节点的物理地址和页大小。
2. 读取 CBF MAGIC、VERSION、BUILD_ID。
3. 读取 W2、CFG、W3、W4 的复位值。
4. 不启动业务，仅确认每个寄存器写入后 PL 可观察到。

### 阶段二：验证 IQ 输入和 Frame

1. 使用固定测试 IQ，绕过湿端 TCP。
2. 检查 IQA/IQB 起始地址、长度和前 128 字节是否被排除。
3. 检查 Frame `+12..+3711` 内容。
4. 确认 PL 没有覆盖 PS 区域，PS 没有覆盖 PL 区域。

### 阶段三：验证 DMA

1. 使用固定长度 AXI Stream。
2. 检查 BD 链表、CURDESC、TAILDESC。
3. 只验证 Frame DDR 是否收到完整 DMA 数据。
4. 检查每个 BD status 和 DMASR。

### 阶段四：验证 CBF 和 PL 输出

1. 写入 CBF 输入参数。
2. 单次 START，观察 BUSY、DONE、错误状态和输出长度。
3. 检查 `frame + 3712` 以后的 PL 输出。
4. 再加入 W2/SRIO/DSP。

### 阶段五：验证 NREAD 和显控

1. 确认 NREAD_EVENT_COUNT 增加。
2. 校验 NREAD_LAST_BYTES 和 NREAD 区包头。
3. 由一个线程独占 IRQ fd，确认中断不会被两个线程抢读。
4. 最后验证 `dry_to_upper()` 的拆包和显控 TCP 发送。

---

## 16. 版本和责任边界

### PS 侧负责

- 湿端 TCP 接收和长度检查。
- IQData、IQData_Last 和传感器数据解析。
- IQA/IQB 乒乓管理。
- Frame PS 区构造。
- W2、CFG 已确认字段、CBF 输入寄存器、DMA BD 和 DMA 控制。
- 结果中断接收、NREAD 读取和显控发送。
- 输出每帧关键状态日志。

### PL 侧负责

- CBF 寄存器实现和版本信息。
- IQ Reader 对 IQA/IQB 的读取。
- Frame PL 区生成和 Frame 所有权区域覆盖。
- AXI-Stream 到 S2MM 的输出时序。
- CBF 算法状态、角度/横摇/声速实际生效规则。
- W2/SRIO/DSP 发送与回读。
- NREAD 结果、计数、错误、溢出寄存器。
- 中断源、清除方式和中断产生时点。

### DSP/显控侧负责

- DSP Bank 地址和 Frame 输入格式解释。
- DSP 输出/NREAD 包格式、有效长度和尾部。
- 显控期望的数据头、长度和拆包方式。

---

## 17. 结论

当前 PS 侧已经形成“湿端 TCP -> IQA/IQB -> Frame -> CBF/CFG/W2/DMA -> PL/DSP/NREAD -> FPGA 中断 -> 显控”的主框架。地址分配和 UIO 同页合并关系已经明确，Frame 的 PS/PL 写权限也已经按 `+12..+3711` 与 PL 区域分开。

当前最需要与 PL/DSP 冻结的不是单个 C 语句，而是以下三个闭环：

1. **启动闭环**：CBF START 后，DMA 完成、CBF DONE 和中断分别代表什么。
2. **结果闭环**：NREAD 什么时候有效、有效多少字节、由谁清除/复用。
3. **协议闭环**：当前旧版 `dry_to_upper()` 的 `@@ST` 解析是否仍适用于新 S231 NREAD Frame。

只有这三个闭环确认后，才能判断现场问题究竟来自 PS 参数、PL 算法、DMA、SRIO/DSP、NREAD 还是显控 TCP。
