# 23001 PS/PL GitHub 协作规则

## 当前目标

本仓库用于重构以下数据链路：

~~~text
湿端 IQ → PL → DSP → PL → PS → 显控
~~~

当前共同工作分支是 main，旧系统基线由 baseline-20260921 标签保存。本次不重构温度、升级、PTP、电源和其他无关业务。

## 目录职责

- ps/20260916/：PS 源码，由 PS Codex 修改，PL Codex 只读查看；
- ps/device-tree/：PS 设备树，由 PS Codex 修改，PL Codex 只读查看；
- pl/：PL 配置、寄存器导出、地址映射、构建说明和 PL 侧实现，由 PL Codex 修改；
- docs/数据链路/：寄存器、数据帧、DMA、中断和 DSP 接口；
- docs/协作记录/：问题、证据、回复、工程师裁定和验证记录；
- docs/协作沟通/：协作方式和其他非接口沟通。

PS Codex 和 PL Codex 都可以查看整个仓库，但不能修改对方负责的代码目录。发现对方目录的问题时，写入协作记录，由目录所属 Codex 修改。

## Git 使用方式

两台电脑各自克隆同一个仓库，不能共用一个本地工作目录。双方直接在 main 上协作，不使用 Pull Request，也不设置第三个集成 Codex。

开始工作前：

~~~powershell
git fetch origin
git switch main
git pull --ff-only
git status --short --branch
~~~

完成工作后：

~~~powershell
git diff --check
git status --short
git add pl docs
git commit -m "pl: ..."
git push origin main
~~~

提交前必须确认没有修改对方代码目录。禁止强制推送、覆盖他人提交或凭猜测解决协议冲突。推送被拒绝时先获取远程更新，发生冲突就暂停并记录。

## 接口规则

涉及寄存器、数据帧、DMA、IRQ、DSP 时序、设备树、错误、复位、超时、性能或显控格式的内容，必须写入 docs/数据链路/ 和必要的 docs/协作记录/。

以下内容必须由 PS/PL 工程师裁定：真实 IQ 格式、DSP 输入输出、最终寄存器地址、DMA 缓冲区和 Cache、IRQ 时序、复位状态、错误恢复、性能指标和硬件验收。

普通文件组织、已确认接口的封装、构建脚本、测试脚本和注释可以由对应 Codex 自行完成。

## 提交信息

~~~text
ps: ...
pl: ...
docs: ...
test: ...
~~~

硬件验证完成后，再在 main 上建立正式版本标签，例如 pspl-20260921-r1。

PL 工程师第一次接入仓库时，使用本地交付的《PL工程师_23001协作仓库与PL Codex使用说明》，其中包含 GitHub 授权、SSH、克隆、Codex 打开仓库和日常协作步骤。
