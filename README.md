# robot-main

VEX V5 机器人控制程序，基于 **PROS 4** 内核（`kernel@4.2.2`），当前以自动赛（autonomous）为主。

## 硬件配置

| 部件 | 端口 | 说明 |
| --- | --- | --- |
| 左驱动电机组 | 1、2 | 端口取负号表示反转 |
| 右驱动电机组 | 3、4 | 正转 |
| IMU（惯性测量单元） | 21 | 放在 21 号口，1–20 号口留给电机与电磁阀 |
| 主控手柄 | — | `pros::Controller master` |

## 功能

- `drive_straight(power_mv, run_ms, P_gain)`：用 IMU 做朝向闭环的直行，电压输出限幅 ±12000 mV，避免过压损坏电机。
- `autonomous()`：设置刹车模式为 `E_MOTOR_BRAKE_BRAKE`，半速（6000 mV）走 1.5 秒。
- `opcontrol()`：检测手柄 A 键按下，触发一次 0.8 秒直行。
- `initialize()`：初始化 LCD，并等待 IMU 校准完成（最多等 2 秒，IMU 未接时不会卡死）。

## 依赖：PROS SDK

本项目**不是**自包含的，构建需要 PROS 内核 4.2.2 的完整 SDK。仓库里只跟踪了其中的文本文件，预编译静态库被 `.gitignore` 的 `*.a` 规则排除在版本管理之外。

**必须补齐的 3 个静态库**（约 6 MB），否则链接阶段会失败：

```
firmware/libc.a
firmware/libm.a
firmware/libpros.a
```

### 已入库 / 未入库

| 文件 | 是否入库 | 说明 |
| --- | --- | --- |
| `include/pros/*.h`、`*.hpp` | 是 | PROS 内核头文件 |
| `include/api.h`、`include/main.h` | 是 | 项目入口头文件 |
| `common.mk`、`firmware/v5*.ld` | 是 | 构建脚本与链接脚本 |
| `project.pros` | 是 | 记录内核版本 4.2.2、target v5 |
| `firmware/libc.a`、`libm.a`、`libpros.a` | **否** | 预编译二进制库，需本地补齐 |
| `bin/`、`.d/`、`.vscode/`、`compile_commands.json` | 否 | 构建产物与本地配置 |

### 工具链

- ARM 交叉工具链 `arm-none-eabi-gcc` / `arm-none-eabi-g++`（PROS 安装包自带）
- 构建标准：C++ `gnu++26`、C `gnu23`（见 `common.mk`）
- 推荐用 VS Code 的 **PROS** 扩展（`sigbots.pros`），它会一并安装 PROS CLI 和工具链

## 克隆后补齐 SDK

任选一种方式。

### 方式 A：从本地内核模板直接拷贝（最快）

前提：本机已安装过 PROS 4.2.2 内核模板，路径一般为
`%APPDATA%\PROS\templates\kernel@4.2.2\firmware\`。

PowerShell：

```powershell
$k = "$env:APPDATA\PROS\templates\kernel@4.2.2\firmware"
Copy-Item "$k\libc.a","$k\libm.a","$k\libpros.a" firmware\
```

Git Bash：

```bash
cp "$APPDATA/PROS/templates/kernel@4.2.2/firmware/"{libc,libm,libpros}.a firmware/
```

如果该目录不存在，先执行下面的 `pros c fetch kernel@4.2.2` 把模板下载下来。

### 方式 B：用 PROS CLI 应用内核模板

在项目根目录执行：

```bash
pros c fetch kernel@4.2.2   # 下载内核模板
pros c apply kernel@4.2.2   # 把内核文件（含 3 个 .a）写入当前项目
```

`apply` 会覆盖 `common.mk` 和 `include/pros/` 下的 SDK 文件。版本一致时内容相同，不会破坏你的代码。

### 方式 C：从零新建项目再搬代码

用 VS Code PROS 扩展新建一个 V5 项目（内核选 4.2.2），然后把本仓库的 `src/`、`include/main.h`、`project.pros` 覆盖进去。

## 构建与烧录

### VS Code（推荐）

安装 PROS 扩展后打开仓库根目录，用命令面板或状态栏的 **Build** / **Upload**。

### 命令行

```bash
pros make            # 编译，产物在 bin/monolith.bin
pros upload          # 烧录到已连接的主控
pros upload . COM3   # 指定串口
```

也可以直接用 `make`，但需要保证 `make` 和 `arm-none-eabi-*` 在 PATH 中。VS Code 扩展自带的工具链默认位于：

```
%APPDATA%\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin
```

## 目录结构

```
.
├── Makefile / common.mk    构建入口（默认目标 quick）
├── project.pros            项目元数据：内核 4.2.2，target v5
├── src/main.cpp            主程序
├── include/
│   ├── main.h、api.h       项目头文件
│   └── pros/               PROS SDK 头文件
├── firmware/
│   ├── v5.ld、v5-hot.ld、v5-common.ld   链接脚本
│   └── lib*.a              需本地补齐，不入库
└── .gitignore
```

## 已知问题

- `src/main.cpp:4` 引用了 `pros/controller.hpp`，该头文件在 PROS 4.2.2 中**不存在**，编译会报
  `fatal error: pros/controller.hpp: No such file or directory`。
  `pros::Controller` 由 `pros/misc.hpp`（经 `api.h`）提供，改为 `#include "pros/misc.hpp"` 或直接删掉这一行即可。
- `src/main.cpp:5` 是一句裸字符串字面量（目前的问题备注），语法上合法但会被编译器告警，建议改成注释。
- `drive_straight()` 目前只按时间跑，没有距离闭环；`opcontrol()` 中触发直行时没有中断保护。

## 内核升级

```bash
pros c fetch kernel@<新版本>
pros c apply kernel@<新版本> --upgrade
```

升级后 `firmware/lib*.a` 会更新为本地文件（仍被 `.gitignore` 忽略），其他协作者需要按上面的方式同步内核版本。
