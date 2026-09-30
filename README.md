# rm-aboard-firmware

基于**大疆 RoboMaster A 型开发板**（STM32F427IIH6）的步兵机器人电控固件，HAL + FreeRTOS(CMSIS-RTOS v2) + CMake。

> ⚠️ **项目状态：开发中（WIP）**。板级驱动（CAN / UART / 电机 / 遥控器 / IMU）已成型，控制律层尚未移植，机器人还不能跑。

## 由来与迁移说明

本仓库是 **COD 战队**步兵机器人电控代码框架的**迁移 + 硬件适配**成果：

| | 原工程 | 本仓库 |
| --- | --- | --- |
| 目标硬件 | **DM 开发板** | **大疆 RoboMaster A 型开发板** |
| MCU | STM32H723VGT6（Cortex-M7 @550MHz，LQFP100） | STM32F427IIH6（Cortex-M4F @168MHz，UFBGA176） |
| 应用层框架 | COD 战队步兵代码 | 沿用（`User/APP` + `User/BSP` + `User/Components` 分层） |
| 工程/构建 | STM32CubeMX + CMake | 沿用同一套构建方式（见下方[硬件平台](#硬件平台)） |

迁移的主要工作量集中在**板级适配**，应用层逻辑与设备驱动接口尽量保持原样：

- **BSP 层重写**：新增 `bsp_mcu` / `bsp_can` / `bsp_uart`，在 CubeMX 生成的外设句柄之上封装；`MCU_Init()` 挂在 `main()` 的 `USER CODE BEGIN 2` 段内统一调用。
- **CAN 适配双路总线**：A 板 CAN1 走 FIFO0（FilterBank 0）、CAN2 走 FIFO1（FilterBank 14），收发中断回调改为 `HAL_CAN_RxFifo0/1MsgPendingCallback`。
- **UART 改为 TOIDLE + DMA 双缓冲**：遥控器与 IMU 的接收不再走 HAL 的常规 API，而是按 RM 社区通行做法手写寄存器：置 `ReceptionType = HAL_UART_RECEPTION_TOIDLE`、打开 `UART_IT_IDLE`、直接配置 `DMA_SxCR_DBM` 与 M0AR/M1AR 双缓冲，再由 `HAL_UARTEx_RxEventCallback` 回调在双缓冲间切换。
- **RTOS 改为静态创建**：任务由 CubeMX 以静态方式生成（`osThreadNew` + 静态 TCB/栈），任务函数在 `User/APP/Src/` 下覆盖 `freertos.c` 的 `__weak` 空壳。
- **构建系统不变**：原工程同为 STM32CubeMX + CMake，本仓库沿用同一套流程，`User/**` 目录由 `GLOB_RECURSE` 自动收集。

### MCU 换代带来的差异（H723 → F427）

两块板卡同属 STM32 + HAL + CubeMX 体系，**应用层代码基本可以平移**，但以下几点在迁移时必须重新处理：

| 维度 | STM32H723VGT6 | STM32F427IIH6 | 迁移注意 |
| --- | --- | --- | --- |
| 内核 / 频率 | Cortex-M7 @550MHz | Cortex-M4F @168MHz | **算力约差 3 倍**，原代码里耗时较重的算法需重新评估能否按原频率跑完 |
| DMA 架构 | DMAMUX 请求路由，DMA1/DMA2 | DMA1/DMA2 固定 Stream-Channel 映射 | DMA 请求与 Stream 需在 CubeMX 里重新分配（如 USART1_RX→DMA2_Stream2） |
| 数据缓存 | 有 D-Cache | **无 D-Cache** | H7 上为 DMA 做的 cache 一致性处理（invalidate / MPU 配置）在 F4 上不再需要，但**不能直接照搬**，否则逻辑冗余甚至误伤 |
| 时钟树 | 550MHz 主频、HSE 配置不同 | 168MHz，HSE 12MHz | `SystemClock_Config()` 由 CubeMX 重新生成 |

**共同点**：DMA 双缓冲（`DMA_SxCR_DBM` + M0AR/M1AR）与 TOIDLE 空闲中断的接收结构在两块板子上是同一套思路，`bsp_uart.c` 里 `ReceptionType = HAL_UART_RECEPTION_TOIDLE` 的手写多缓冲实现因此可以直接沿用。

> 📌 **术语澄清**：本文里的「DM 开发板」指原工程的目标板卡；而代码中 `DM_` 前缀的函数与结构体（`DM_Motor_*`）指的是**达妙（Damiao）电机**，两者含义不同，沿用原名未做改动。

目前应用层已经搬过来，但**控制律（PID / 滑模）尚未迁移**，因此整体仍处于未完成状态，详见[已知缺口](#已知缺口)。

## 硬件平台

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F427IIH6（UFBGA176，Cortex-M4F @168MHz） |
| 晶振 | HSE 12MHz，PLLM=6 / PLLN=168 / PLLP=2 |
| RTOS | FreeRTOS 10.x，CMSIS-RTOS v2，tick 1000Hz，静态创建任务 |
| 构建 | CMake + Ninja + arm-none-eabi-gcc |
| 配置工具 | STM32CubeMX 6.14.1（[`rm-aboard-firmware.ioc`](rm-aboard-firmware.ioc)） |

### 外设分配

| 外设 | 引脚 | 配置 | 用途 |
| --- | --- | --- | --- |
| CAN1 | PD0 / PD1 | 1 Mbps，FIFO0，FilterBank 0 | 底盘 3508 ×4、YAW（GM6020）、拨弹轮 2006 |
| CAN2 | PB5 / PB6 | 1 Mbps，FIFO1，FilterBank 14 | 达妙 DM 电机（Pitch） |
| USART1 | PA9 / PB7 | 100000 bps，9 位，偶校验，DMA2_Stream2 RX | DBUS 遥控器接收机 |
| UART7 | PE7 / PE8 | 异步 | 维特 Wit901c 陀螺仪 |
| UART8 | PE0 / PE1 | 异步，DMA1_Stream6 RX | 对外通信 / 上位机 |
| SWD | PA13 / PA14 | — | 调试下载 |

## 目录结构

```
Core/                      CubeMX 生成：时钟、外设初始化、中断、FreeRTOS 骨架
User/
├── APP/                   应用任务层（任务函数在此覆盖 freertos.c 的 __weak 空壳）
│   └── Src/               CAN_Task / INS_Task / Control_Task / Detect_Task
├── BSP/                   板级支持：bsp_can、bsp_uart、bsp_mcu
└── Components/
    ├── Debug/             Vofa+ 上位机波形调试
    └── Device/            Motor（DJI + 达妙）、Remote_Control、Wit901c
Drivers/                   STM32F4 HAL + CMSIS
Middlewares/               FreeRTOS 内核
cmake/stm32cubemx/         CubeMX 生成的构建脚本
```

`CMakeLists.txt` 用 `GLOB_RECURSE` 自动收集 `User/**/*.c` 与头文件目录，新增文件无需手动改构建脚本（重新配置即可）。

## 任务划分

| 任务 | 优先级 | 栈 | 当前实现 |
| --- | --- | --- | --- |
| `INS_Task` | 40（High） | 4 KB | 已接入 `IMU_Process()` 与遥控器通道读取；姿态解算待补 |
| `Control_Task` | 32（AboveNormal） | 4 KB | 空循环，控制律未实现 |
| `CAN_Task` | 24（Normal） | 4 KB | 已能通过 `CAN1` 下发 DJI 电机电流；DM 电机指令已注释待启用 |
| `Detect_Task` | 16（BelowNormal） | 4 KB | 空循环，离线检测未实现 |

任务在 [`Core/Src/freertos.c`](Core/Src/freertos.c) 中注册，实体函数以 `__weak` 空壳形式声明，由 `User/APP/Src/` 下的同名函数覆盖。

## 上手流程

1. 安装 `arm-none-eabi-gcc`、CMake ≥ 3.22、Ninja，并确保在 PATH 中（或修改 [`cmake/gcc-arm-none-eabi.cmake`](cmake/gcc-arm-none-eabi.cmake)）。
2. 配置并编译：

   ```bash
   cmake --preset Debug
   cmake --build --preset Debug
   ```

   产物位于 `build/Debug/rm-aboard-firmware.elf`。
3. 用 VS Code 的 [Cortex-Debug](https://github.com/Marus/cortex-debug) 扩展 + OpenOCD（CMSIS-DAP）下载调试，配置见 [`.vscode/launch.json`](.vscode/launch.json)。
4. 修改外设配置后，用 STM32CubeMX 打开 `.ioc` 重新生成代码。**注意**：生成前确认 `USER CODE BEGIN/END` 段内的代码不被覆盖，`Core/Src/main.c` 中的 `MCU_Init()` 调用位于该保护段内。

## 已知缺口

- [ ] 控制律层（PID / 滑模）尚未移植，`Control_Task` 与 `Detect_Task` 仍为空循环；`INS_Task` 只做了数据搬运，姿态解算与串级 PID 待实现。
- [ ] `Remote_Control`（DBUS）解析已读数，但未做失联保护与档位映射。
- [ ] 无单元测试、无 CI。

## 说明

- 应用层代码框架源自 **COD 战队**步兵机器人工程，本仓库的工作是将其迁移到大疆 A 型开发板并完成板级适配；如原战队有开源协议或署名要求，以其规定为准。
- 本项目与 DJI 官方无隶属关系。
- `Drivers/`、`Middlewares/` 下的第三方代码版权归 STMicroelectronics 与 FreeRTOS 所有，遵循各自目录中的 LICENSE。
