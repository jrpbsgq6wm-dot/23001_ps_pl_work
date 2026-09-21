# 数据链路重构工作区

本目录只服务于“湿端 IQ → PL → DSP → PL → PS → 显控”数据链路重构，不扩展到其他业务模块。

当前共同工作分支：

~~~text
rewrite/data-path
~~~

双方 Codex 都可以查看整个仓库，包括对方负责的目录。修改权限按目录划分。

PS Codex 负责修改：

- ps/20260916/ 下与数据链路相关的源码；
- ps/device-tree/system.dts；
- UIO 映射、寄存器结构、DMA/IRQ 处理、DSP 结果解析和显控上传。

PL Codex 负责修改：

- pl/ 下的数据链路配置；
- PL 寄存器、DMA、FIFO、中断和 DSP 接口；
- PL build ID、配置导出和硬件依据。

PL Codex 可以查看 PS 源码和设备树，PS Codex 可以查看 PL 配置；任何一方都不能直接修改对方负责的目录。发现问题时，写入 docs/协作记录/，由目录所属 Codex 完成修改。

寄存器和数据格式没有经过 PL/PS 工程师确认前，两个 Codex 都不能把猜测写成正式实现。
