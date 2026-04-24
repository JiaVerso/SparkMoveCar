# SparkMoveCar

SparkMoveCar 是一个基于 STM32F427XX 微控制器的机器人底盘控制系统。该项目集成了 RT-Thread 实时操作系统，采用 C/C++ 混合编程，通过 CMake 构建，旨在实现对 DJI 电机（如 M3508/C620）和 达妙电机（DM4310）的精确控制。

## 🚀 项目概览

- **核心芯片**: STM32F427IIH6 (高性能 ARM Cortex-M4)
- **操作系统**: RT-Thread Nano
- **构建系统**: CMake + GCC/Clang (ARM None EABI)
- **编程语言**: C11, C++17

## 🛠 硬件环境

- **控制板**: 通用 STM32F427 开发板（如 RoboMaster A 型主控板）
- **驱动电机**: 
  - DJI M3508 直流无刷减速电机 + C620 电调 (CAN 通信)
  - 达妙 DM4310 动力电机 (CAN 通信)
- **外设**:
  - 多路 CAN 总线 (CAN1/CAN2)
  - UART 串口通信 (支持 DMA + FIFO + 空闲中断)
  - 板载 LED 状态指示
  - 24V 外部供电控制 (PMOS 驱动)

## 📂 项目结构

```text
SparkMoveCar/
├── App/                # 应用层测试代码
├── Bsp/                # 硬件抽象层 (LED, CAN, UART, Math)
├── Core/               # 核心启动代码及 C++ 业务入口 (app_main.cpp)
├── Device/             # 具体设备驱动 (DJI 电机, 达妙电机, 串口绘图仪)
├── Modules/            # 通用算法模块 (PID 控制器, FIFO 环形缓冲区)
├── Middlewares/        # RT-Thread 操作系统组件
├── cmake/              # CMake 编译配置文件
└── .doc/               # 开发笔记与技术总结 (process.md)
```

## 🏗 软件架构

### 1. 启动流程
项目修改了启动汇编，引导程序先进入 RT-Thread 的 `entry()` 线程进行系统初始化（时钟、外设、调度器），随后自动创建 `main` 线程并跳转至 `Core/Src/app_main.cpp` 中的 `app_main()`。

### 2. 核心特性
- **C++ 友好**: 支持在嵌入式环境中使用 C++ 类、对象及 STL 模板。
- **高效串口**: 采用 DMA + 环形缓冲区 (FIFO) + IDLE 中断，确保高频数据收发不丢包。
- **CAN 通信**: 
  - 实现了基于过滤器和掩码的 ID 过滤。
  - 封装了 DJI 和 达妙电机的协议解包与控制指令发送。
- **控制算法**: 集成了位置/速度串级 PID 控制，支持前馈补偿。

## 🔧 开发与构建

### 环境依赖
- [CMake](https://cmake.org/) (>= 3.22)
- [GNU Arm Embedded Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) 或 Clang
- [Make](https://www.gnu.org/software/make/) 或 [Ninja](https://ninja-build.org/)

### 编译指令
```bash
mkdir build
cd build
cmake .. -G "Unix Makefiles" # 或使用 Ninja
make -j
```

## 📝 技术笔记 (摘自 process.md)

- **RT-Thread 移植**: 解决了 Clang 编译器下 `rt_thread_mdelay()` 在非线程环境下调用的 HardFault 问题。
- **UART 调试**: 总结了波特率误差（晶振配置）、内存管理及 `static` 关键字在固件优化中的作用。
- **CAN 总线**: 详细记录了 120Ω 匹配电阻、位填充机制、总线仲裁及总线负载率（建议 <70%）的重要性。
- **电机控制**: 记录了串级 PID 实现思路及达妙电机特殊的位域打包方式。

## ⚖ 许可证

该项目遵循 [LICENSE](LICENSE) 文件中所述的开源协议。

---
*Developed by USTC-RoboWalker Team*
