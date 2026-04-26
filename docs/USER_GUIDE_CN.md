# Apollo-11 Guidance Computer - ESP32-C3 RV32IMAC 移植指南

## 目录

- [概述](#概述)
- [硬件要求](#硬件要求)
- [软件环境](#软件环境)
- [快速开始](#快速开始)
- [项目结构](#项目结构)
- [构建与烧录](#构建与烧录)
- [运行测试](#运行测试)
- [代码说明](#代码说明)
- [已知问题](#已知问题)
- [故障排除](#故障排除)

---

## 概述

本项目是**阿波罗11号制导计算机(AGC)**算法的C语言移植版本，运行在**ESP32-C3开发板**上进行验证。

### 目标平台

| 参数 | 值 |
|------|-----|
| 芯片 | ESP32-C3 |
| 架构 | RISC-V RV32IMAC |
| SRAM | 160KB |
| Flash | 4MB |
| FPU | **无** (软件模拟) |

### 为什么选择软浮点?

ESP32-C3的RV32IMAC架构**没有硬件浮点单元(FPU)**，所有`double`运算必须通过软件模拟。这意味着：
- 算法精度与x86版本**完全一致**
- 性能约为硬件FPU的**1/10~1/100**
- 代码体积增加约50-100KB

> 功能完整优先，不做定点数学优化。

---

## 硬件要求

### 开发板

- **ESP32-C3-DevKitC-02** 或兼容板
- USB数据线
- (可选) 如果需要连接DSKY外设：自行设计的DSKY模拟器板

### 硬件连接

```
ESP32-C3          PC
   │               │
   ├─── USB ───────┤  (烧录和监控)
   │               │
   └───────────────┘
```

---

## 软件环境

### 必需软件

1. **ESP-IDF v5.0+**
   ```bash
   # 下载ESP-IDF
   git clone --recursive https://github.com/espressif/esp-idf.git
   cd esp-idf
   git checkout v5.3.2
   ./install.sh

   # 激活环境 (每次新终端需要)
   source export.sh
   ```

2. **Python 3.8+**

3. **Git**

### 验证环境

```bash
# 验证ESP-IDF安装
idf.py --version
# 应显示: v5.3.2

# 检查串口驱动 (Windows)
# 安装ESP32-C3 USB驱动: https://www.silabs.com/products/development-tools/software/serial-bridge
```

---

## 快速开始

### 步骤1: 克隆代码

```bash
git clone https://github.com/55Curry/apollo11_in_C.git
cd apollo11_in_C/apollo_esp32
```

### 步骤2: 设置目标芯片

```bash
idf.py set-target esp32c3
```

### 步骤3: 配置项目

```bash
idf.py menuconfig
# 一般不需要修改，默认配置即可
# 按ESC退出
```

### 步骤4: 构建

```bash
idf.py build
```

### 步骤5: 烧录和监控

```bash
# 查找串口
# Windows: 在设备管理器中查看 COM端口
# Linux/Mac: ls /dev/tty*

idf.py -p /dev/ttyUSB0 flash monitor
# Windows: idf.py -p COM3 flash monitor
```

### 步骤6: 观察输出

如果一切正常，你将看到：

```
Apollo-11 Guidance Computer Test Suite
Target: ESP32-C3 RV32IMAC (soft-float)

[TEST] Math Suite
  PASSED

[TEST] Vecmath Suite
  PASSED

[TEST] Orbit Suite
  PASSED
========================================
  ALL TESTS PASSED SUCCESSFULLY!
========================================
Apollo Guidance Computer initialized successfully
```

---

## 项目结构

```
apollo_esp32/
├── CMakeLists.txt              # 项目根CMake (必须)
├── Kconfig.projbuild           # ESP-IDF配置 (可选)
├── README.md                   # 本文档
├── main/
│   ├── main.c                 # FreeRTOS入口
│   ├── test_runner.c          # 测试执行器
│   └── CMakeLists.txt         # main组件CMake
└── components/
    └── apollo/                # Apollo算法组件
        ├── CMakeLists.txt     # apollo组件CMake
        ├── include/           # 16个头文件
        │   ├── apollo.h
        │   ├── apollo_types.h
        │   ├── apollo_math.h
        │   ├── apollo_vecmath.h
        │   ├── apollo_orbit.h
        │   ├── apollo_attitude.h
        │   ├── apollo_nav.h
        │   ├── apollo_guidance.h
        │   ├── apollo_entry.h
        │   ├── apollo_tvc.h
        │   ├── apollo_rcs.h
        │   ├── apollo_dap.h
        │   ├── apollo_display.h
        │   ├── apollo_exec.h
        │   ├── apollo_interpreter.h
        │   ├── apollo_io.h
        │   └── apollo_waitlist.h
        └── src/               # 14个源文件
            ├── apollo_math.c
            ├── apollo_vecmath.c
            ├── apollo_orbit.c
            ├── apollo_attitude.c
            ├── apollo_nav.c
            ├── apollo_guidance.c
            ├── apollo_entry.c
            ├── apollo_tvc.c
            ├── apollo_rcs.c
            ├── apollo_dap.c
            ├── apollo_display.c
            ├── apollo_exec.c
            ├── apollo_interpreter.c
            └── apollo_io.c
```

---

## 构建与烧录

### 完整命令序列

```bash
#!/bin/bash
# 保存为 build_and_flash.sh 并运行

export IDF_PATH=~/esp/esp-idf
source $IDF_PATH/export.sh

cd apollo_esp32

# 清理旧构建
idf.py fullclean

# 设置目标
idf.py set-target esp32c3

# 构建
idf.py build

# 烧录 (根据你的系统修改端口)
idf.py -p /dev/ttyUSB0 flash

# 打开串口监控
idf.py -p /dev/ttyUSB0 monitor
```

### Windows PowerShell

```powershell
# 打开ESP-IDF Command Prompt (开始菜单)

cd "d:\code\项目\apollo\Apollo-11-C\apollo_esp32"
idf.py set-target esp32c3
idf.py build
idf.py -p COM3 flash monitor
```

### 只烧录已构建的固件

```bash
# 不重新构建，直接烧录
idf.py -p /dev/ttyUSB0 erase-flash flash
```

---

## 运行测试

### 测试列表

| 测试 | 文件 | 功能 |
|------|------|------|
| Math | test_math.c | `apollo_spsin()` / `apollo_spcos()` 三角函数精度 |
| Vecmath | test_vecmath.c | 向量加减、点积、叉积运算 |
| Orbit | test_orbit.c | 轨道根数转换、Kepler方程求解 |

### 添加新测试

1. 在 `main/test_runner.c` 中添加测试函数声明：
   ```c
   extern int test_new_feature_suite(void);
   ```

2. 在测试任务中添加调用：
   ```c
   failures += test_new_feature_suite();
   ```

3. 在 `main/test_runner.c` 文件底部实现测试函数

### 串口输出重定向

测试通过ESP-IDF的 `ESP_LOGI()` 输出到UART，可以通过以下方式过滤：

```bash
idf.py -p /dev/ttyUSB0 monitor | grep APOLLO
idf.py -p /dev/ttyUSB0 monitor | grep TEST
idf.py -p /dev/ttyUSB0 monitor | grep FAIL
```

---

## 代码说明

### 核心算法模块

| 模块 | 功能 | 依赖 |
|------|------|------|
| apollo_math | 正弦/余弦近似算法 | 无 |
| apollo_vecmath | 向量3x1和矩阵3x3运算 | math |
| apollo_orbit | 轨道力学、Kepler方程 | math, vecmath |
| apollo_attitude | 姿态四元数/DCM | math, vecmath |
| apollo_nav | 导航状态向量 | math, vecmath, orbit |
| apollo_guidance | 制导算法 Lambert targeting | math, vecmath, orbit |
| apollo_entry | 重新进入控制 | math, vecmath |
| apollo_tvc | 推力矢量控制 | math, vecmath |
| apollo_rcs | 喷气控制系统 | math |
| apollo_dap | 数字自动驾驶仪 | attitude, rcs |
| apollo_exec | 作业调度器(模拟) | 无 |
| apollo_interpreter | 解释器(模拟) | exec |

### 关键数据结构

```c
// 三维向量
typedef struct {
    double x, y, z;
} apollo_v3_t;

// 3x3矩阵
typedef struct {
    double m[3][3];
} apollo_mat3_t;

// 轨道状态 (位置+速度)
typedef struct {
    apollo_v3_t r;  // 位置 m
    apollo_v3_t v;  // 速度 m/s
} apollo_state_t;

// 轨道根数
typedef struct {
    double a;  // 半长轴 m
    double e;  // 离心率
    double i;  // 轨道倾角 rad
    double o;  // 升交点赤经 rad
    double w;  // 近地点幅角 rad
    double nu; // 真近点角 rad
} apollo_elements_t;

// 四元数
typedef struct {
    double q0, q1, q2, q3;
} apollo_quat_t;
```

### 内存占用估计

| 模块 | 代码大小 | RAM需求 |
|------|---------|---------|
| apollo_math | ~10KB | ~1KB |
| apollo_vecmath | ~15KB | ~500B |
| apollo_orbit | ~20KB | ~2KB |
| apollo_attitude | ~15KB | ~1KB |
| 其他模块 | ~100KB | ~5KB |
| **总计** | **~160KB** | **~10KB** |

> 软浮点数学库会额外增加约50-100KB代码体积

---

## 已知问题

### 1. 性能

RV32IMAC无FPU，所有浮点运算软件模拟：
- 单次三角函数约需 1-10ms
- 完整轨道传播约需 100ms-1s
- 这是预期行为，不是bug

### 2. apollo_multithread.c 排除

该文件使用POSIX threads (`pthread`)，ESP32使用FreeRTOS，不兼容：
- 已从CMakeLists.txt中排除
- 如需多线程，使用FreeRTOS `xTaskCreatePinnedToCore()`

### 3. 调度器为模拟实现

`apollo_exec.c` 和 `apollo_waitlist.c` 中的作业调度是纯软件模拟：
- 非实时操作系统
- 仅用于算法验证

### 4. 无实际外设

所有I/O (`apollo_io.c`) 是内存模拟：
- 无实际GPIO/UART/I2C交互
- 仅适合算法验证测试

---

## 故障排除

### 构建失败

**错误**: `Could not find a component 'freertos'`
```
# 解决方案：检查ESP-IDF是否正确安装
source $IDF_PATH/export.sh
idf.py --version
```

**错误**: `target is not supported`
```
# 确保ESP-IDF版本 >= 5.0
idf.py --version  # 应该是v5.x
```

### 烧录失败

**错误**: `Failed to connect to device`
```
# 检查串口驱动
# Windows: 安装Silicon Labs CP210x驱动
# Linux: ls /dev/tty* 检查权限 (sudo chmod 666 /dev/ttyUSB0)
# 或查找正确端口
python -m esptool --port /dev/ttyUSB0 flash_id
```

### 输出乱码

**现象**: 串口输出乱码
```
# 解决方案：检查串口波特率是否正确
idf.py -p /dev/ttyUSB0 monitor -b 115200
```

### 无限重启

**现象**: `rst:0x3 (RTC_WDT_RTC_RESET)`
```
# 检查电源供电是否充足
# 降低日志级别: idf.py menuconfig -> Component config -> Log output level -> Warning
```

---

## 下一步

### 性能优化 (可选)

如果需要提升性能，可以考虑：

1. **使用float替代double**
   - 精度降低约6位小数
   - 性能提升约3-5倍
   - 需要修改 `apollo_types.h` 中的向量/矩阵定义

2. **定点数学 (Q15/Q31)**
   - 类似原始AGC的15位定点表示
   - 需要完全重写 `apollo_math.c`

3. **查表法 + 插值**
   - 预计算sin/cos表
   - 线性插值求值
   - 适合导航级精度

### 添加外设驱动 (可选)

如需连接真实外设：

1. 修改 `apollo_io.c` 实现真实GPIO/I2C/SPI
2. 添加 `esp_timer` 驱动用于实时中断
3. 集成 FreeRTOS + DSKY 模拟器硬件

---

## 参考资料

- [ESP32-C3技术规格](https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf)
- [ESP-IDF编程指南](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/)
- [Apollo 11原始代码 (Virtual AGC)](http://www.ibiblio.org/apollo/)
- [AGC原始设计文档](https://www.ibiblio.org/apollo/Documents_and_Drawings/AGC_Sim存放.htm)

---

*文档更新日期: 2026-04-26*
