# robot-main

VEX V5 机器人控制程序，基于 **PROS 4** 内核（`kernel@4.2.2`），当前以自动赛（autonomous）为主。

## 硬件配置

| 部件 | 端口 | 说明 |
| --- | --- | --- |
| 右驱动电机组 `right_drive` | 1、2、3 | 前进基准为正；装配反向的 3 号口在代码里取负 |
| 左驱动电机组 `left_drive` | 8、9、10 | 与右组镜像：基准 8 取负，装配反向的 9、10 取正 |
| 抬升电机组 `lift` | 11、20 | 双边驱动；`HOLD` 刹车防止断电后被重力拽落 |
| IMU（惯性测量单元） | 21 | 1–20 号口留给电机与电磁阀；**当前未安装**，程序检测到后自动降级为开环 |
| 主控手柄 | — | `pros::Controller master`（MASTER 口） |

符号约定是"每端口"级别：装配图标白的 3、9、10 三个电机与同侧同伴反着装。实车复核方法（验轮测试）：opcontrol 按住手柄"上"键，六个驱动电机按代码符号同向转——某个轮子倒退就翻转它端口号的符号，六个全反则是整组前进方向定义反了。

## 功能

自动赛流程（`autonomous()`）：刹车设为 `BRAKE` → 启动独立任务 `lift_sequence()` 与底盘并发 → 直行 1.5 s（期间完成抬升并保持）→ 原地左转 90° → 进入第二段直行 0.2 s 后通知下落（1.2 s 落完，重心变化避开转弯）→ 再直行 1.5 s → `join()` 等时序收尾，防止任务残留到遥控期抢电机。

各模块：

- `initialize()`：初始化 LCD；底盘开环校准 IMU 最多等 2 s；抬升组设 `HOLD` 刹车；用 `imu.get_rotation()` 是否返回错误值在线检测 IMU 是否真装，结果写入 `imu_ok`，决定后续走闭环还是开环。
- `drive_straight(power_mv, run_ms, P_gain)`：IMU 朝向 P 闭环直行，纠偏方向"左加右减"，电压限幅 ±12000 mV；`imu_ok` 为 false 时纠偏归零，退化为纯定时直行（IMU 缺失时读数是 NaN，不关掉会把电压算成垃圾值）。
- `turn_left(deg, P_gain, min_mv, tol_deg, timeout_ms)`：原地 pivot 左转角度闭环，偏差越大转得越猛、最低电压克服静摩擦，1.5° 容差到位即停，2.5 s 超时保护；无 IMU 时按 `TURN_FB_MS`（1.1 s）开环定时转，90° 精度需实车标定。
- `lift_sequence()`：一次性"抬起 → 保持 → 下落"时序，跑在独立 `pros::Task` 里与底盘并发；等主流程写入 `g_lift_down_at` 绝对时间戳后才下落，带 5 s 等待上限防死等，跑完自行退出。
- `opcontrol()`：上键 = 验轮测试（六电机同符号正转）；B 键 = 无场控环境下原地跑一遍完整自动赛（约 4–5 s，期间阻塞其他按键）；A 键 = 0.8 s 直行测试；L1/L2 = 手动控制抬升升降，用于标定 `LIFT_UP_SIGN` 与双边方向。

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

- `drive_straight()` 目前只按时间跑，没有距离闭环；`opcontrol()` 中触发直行时没有中断保护。

## 内核升级

```bash
pros c fetch kernel@<新版本>
pros c apply kernel@<新版本> --upgrade
```

升级后 `firmware/lib*.a` 会更新为本地文件（仍被 `.gitignore` 忽略），其他协作者需要按上面的方式同步内核版本。
