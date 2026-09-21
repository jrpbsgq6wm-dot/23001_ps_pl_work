# 23001 PS/PL GitHub 协作规则

## 仓库职责

这个仓库保存 PS 代码、PS 侧设备树副本、PL 配置导出、接口文档和联调记录。

- `ps/`：由 PS 侧维护，包含源码、设备树和 PS 侧配置。
- `ps/20260916/`：PS 源码和编译工程文件。
- `ps/device-tree/`：由 PS 侧维护，目前只放 `system.dts`。设备树正式纳入仓库前，必须确认它对应的硬件版本。
- `pl/`：由 PL 侧维护，只放 PL 配置、寄存器导出、地址映射、构建脚本和版本说明。
- `docs/issues/`：一项接口问题一个文件，避免双方同时编辑同一个长文档。
- `docs/interface/`：双方确认后的接口结论。
- `docs/test-reports/`：硬件联调结果。

## 分支规则

- `main`：最近一次通过联调的稳定组合，不直接提交。
- `ps/<issue>`：PS 源码或设备树修改。
- `pl/<issue>`：PL 配置或 PL 接口导出修改。
- `integration/<topic>`：整合 PS 与 PL 分支并进行编译、烧录和验证。

每次修改前先执行：

```powershell
git fetch --all --prune
git status
git log --oneline --decorate -10
```

不要在两个 Codex 会话中使用同一个工作目录。两台电脑各自克隆仓库；每次提交都必须先拉取远程更新，禁止强制推送和覆盖别人的分支。

## 接口变更规则

未标记为“双方确认”的接口不能进入 PS 代码或设备树。PL 侧提交必须给出寄存器地址、位定义、单位、写入时机、数据长度、中断时序和配置版本。PS 侧实现必须在问题记录中填写对应的提交号。

当前待确认问题见 `docs/issues/`，重点包括 NREAD 有效长度、IRQ 时序、ROLL_INTERVAL、frequency_code 和 BUILD_ID 兼容策略。

## 提交信息

提交信息使用以下前缀：

```text
ps: ...
pl: ...
docs: ...
test: ...
```

一个提交只完成一类明确的事情。通过硬件验证后，在 `main` 上建立带日期和版本号的标签，例如 `pspl-20260921-r1`。
