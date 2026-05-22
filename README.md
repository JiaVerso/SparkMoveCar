# SparkMoveCar

![SparkMoveCar hardware](docs/spark_move_car.jpg)

SparkMoveCar is a STM32F427 based mobile robot chassis control project. It targets a six-wheel rover-style platform with CAN motor control, SBUS remote input, UART debugging, RT-Thread Nano scheduling, and C/C++ mixed embedded development.

> Note: the picture above is reserved for the real vehicle image. Put the real photo or rendering at `docs/spark_move_car.jpg` and GitHub/Gitee will render it automatically.

## 1. News

- **2026-05-22**: README is reorganized in a FAST-LIO2-like style.
- **2026-05-13**: Chassis control module added, including SBUS input mapping and wheel-speed control interface.

## 2. Features

- STM32F427IIH6 / ARM Cortex-M4 embedded chassis controller.
- RT-Thread Nano runtime with STM32 HAL peripheral initialization.
- CMake based build system with C11, C++17, and ARM GCC/Clang toolchain support.
- CAN1 / CAN2 motor communication abstraction.
- DJI M3508 + C620 motor feedback parsing and current command output.
- DM4310 motor device support through CAN.
- SBUS remote-control input decoding for velocity, steering, and emergency stop.
- Ackermann-style chassis parameters, wheel-speed target conversion, and PID speed loop.
- UART DMA + FIFO infrastructure for debugging, plotting, and command parsing.
- IMU / Quaternion EKF modules reserved for attitude estimation experiments.

## 3. Hardware Platform

| Part | Description |
| --- | --- |
| MCU | STM32F427IIH6 |
| Control board | STM32F427 development board, such as RoboMaster Type-A style controller |
| RTOS | RT-Thread Nano |
| Drive motors | DJI M3508 + C620, DM4310, or VESC-compatible wheel motor setup |
| Communication | CAN1, CAN2, UART, SBUS |
| Debug interface | UART8 / USART1, serial plotter utilities |
| Power | 24 V external power path with board-level switching |

### Chassis Parameters

| Parameter | Value |
| --- | --- |
| Wheel count | 4 active wheel controllers by default |
| Wheel diameter | 0.1524 m |
| Wheelbase | 0.600 m |
| Track width | 0.585 m |
| Maximum linear speed | 1.0 m/s |
| Maximum yaw rate | 1.0 rad/s |
| Maximum steering angle | 0.500 rad |

## 4. Software Architecture

```text
SparkMoveCar/
|-- App/                         # Application layer and chassis control
|   `-- Chassis/                 # Chassis motor table, kinematics, SBUS mapping
|-- Bsp/                         # Board support package: CAN, UART, IMU, math, SBUS
|-- Core/                        # STM32CubeMX generated startup and main logic
|   |-- Inc/
|   `-- Src/
|-- Device/                      # Device drivers: DJI motor, DM4310, N630, BMI088, plotter
|-- Modules/                     # Reusable algorithms: PID, FIFO, Quaternion EKF
|-- Middlewares/                 # RT-Thread middleware
|-- Drivers/                     # STM32 HAL, CMSIS, CMSIS-DSP
|-- cmake/                       # Toolchain and STM32CubeMX CMake glue
|-- CMakeLists.txt               # Top-level build script
|-- CMakePresets.json            # Build presets
`-- SparkMoveCar.ioc             # STM32CubeMX project configuration
```

The project is organized as a typical embedded layered system:

- **Core** initializes clocks, GPIO, DMA, CAN, SPI, UART, and RT-Thread related runtime.
- **Bsp** wraps low-level hardware interfaces into reusable drivers.
- **Device** implements concrete peripherals and motor protocols.
- **Modules** keeps controller and math utilities independent from board logic.
- **App/Chassis** combines SBUS input, kinematic parameters, motor feedback, PID control, and CAN command output.

## 5. Dependencies

Install the following tools before building:

- CMake 3.22 or newer
- GNU Arm Embedded Toolchain, Arm GNU Toolchain, or compatible Clang ARM toolchain
- Ninja or Make
- STM32CubeMX, optional, only needed when regenerating peripheral configuration
- OpenOCD / ST-LINK Utility / STM32CubeProgrammer, optional, for flashing and debugging

The project already contains STM32 HAL, CMSIS, CMSIS-DSP, and RT-Thread source files, so no extra package manager is required for the firmware sources.

## 6. Build

### Configure

```bash
cmake -S . -B build -G Ninja
```

If you prefer Make:

```bash
cmake -S . -B build -G "Unix Makefiles"
```

### Compile

```bash
cmake --build build -j
```

Generated firmware artifacts are produced in the `build/` directory according to the selected toolchain configuration.

## 7. Run On Board

1. Connect the STM32F427 controller to ST-LINK.
2. Connect CAN bus, SBUS receiver, motor drivers, and 24 V power stage.
3. Build the firmware.
4. Flash the generated ELF/HEX/BIN file with your preferred STM32 flashing tool.
5. Power-cycle the chassis and verify that CAN feedback, SBUS input, and motor output behave as expected.

Recommended bring-up order:

1. Verify board power and LED state.
2. Verify UART debug output.
3. Verify SBUS channel values.
4. Verify CAN feedback frames without enabling high current.
5. Enable motor control with a current limit and lifted wheels.
6. Tune PID parameters after confirming wheel direction.

## 8. Chassis Control

The chassis module uses SBUS channels to generate motion commands:

| SBUS channel | Meaning |
| --- | --- |
| CH1 | yaw / steering related input |
| CH2 | forward and backward velocity command |
| CH4 | steering angle command |
| CH8 | emergency stop switch |

Main interfaces:

```c
void ChassisMotor_InitAll(void);
void ChassisMotor_CANRxDispatch(Struct_CAN_Rx_Buffer *rx_buffer);
void ChassisMotor_SetWheelTargetRpm(ChassisWheel_e wheel, float wheel_rpm);
void ChassisMotor_SetChassisSpeed(float vx_mps, float wz_radps);
void ChassisMotor_UpdateFromSbusChannels(const uint16_t channels[CHASSIS_SBUS_CH_COUNT]);
void ChassisMotor_ControlLoop(void);
void ChassisMotor_SendAllCurrent(void);
```

The current main loop calls `ChassisMotor_ControlLoop()` periodically after peripheral and motor initialization.

## 9. Development Notes

- Keep CubeMX generated files under `Core/`, `Drivers/`, and `cmake/stm32cubemx/` consistent with `SparkMoveCar.ioc`.
- The project enables `-Werror`; warning-free builds are required.
- Some `.c` files can be compiled as C++ through CMake when they need C++ linkage compatibility.
- Use current limits and lifted wheels during early motor tests.
- Keep CAN bus termination at 120 ohm on both ends of the bus.
- Keep CAN bus load below a practical safety margin during high-rate motor feedback tests.

## 10. Related Documents

- Development notes: `.doc/process.md`
- License: `LICENSE`

## 11. Acknowledgements

SparkMoveCar is developed for robot chassis control experiments and competition-oriented embedded development.

Thanks to:

- RT-Thread
- STMicroelectronics STM32 HAL / CMSIS
- DJI RoboMaster motor ecosystem
- USTC RoboWalker Team

## 12. License

This project follows the license described in `LICENSE`.
