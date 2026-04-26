# Apollo-11 ESP32-C3 RV32IMAC 移植版本

阿波罗11号制导计算机算法在ESP32-C3上的移植，用于RV32IMAC架构验证测试。

## 项目结构

```
apollo_esp32/
├── CMakeLists.txt              # 项目根CMake
├── Kconfig.projbuild           # ESP-IDF配置
├── main/
│   ├── main.c                  # FreeRTOS入口
│   ├── test_runner.c            # 测试执行器
│   └── CMakeLists.txt
└── components/
    └── apollo/                  # Apollo算法库
        ├── CMakeLists.txt
        ├── include/             # 头文件
        └── src/                 # 源代码
```

## 构建步骤

### 1. 设置ESP-IDF环境

```bash
# 如果使用官方安装方式
source ~/esp/esp-idf/export.sh

# 或使用ESP-IDF安装目录
export IDF_PATH=~/esp/esp-idf
source $IDF_PATH/export.sh
```

### 2. 配置并构建项目

```bash
cd Apollo-11-C/apollo_esp32

# 设置目标芯片
idf.py set-target esp32c3

# 构建项目
idf.py build
```

### 3. 烧录和运行

```bash
# 查看串口
idf.py -p /dev/ttyUSB0 flash monitor

# 或在Windows上
idf.py -p COM3 flash monitor
```

## RV32IMAC软浮点说明

ESP32-C3使用RV32IMAC架构，**无硬件浮点单元(FPU)**：
- 所有`double`运算通过软浮点库(`-msoft-float`)模拟
- 性能约为硬件FPU的1/10~1/100
- 代码体积增加约50-100KB
- **算法精度与x86版本完全一致**

## 测试内容

- **Math Test**: `apollo_spsin()` / `apollo_spcos()` 三角函数
- **Vecmath Test**: 向量运算、点积、叉积
- **Orbit Test**: 轨道根数转换、Kepler方程求解

## 已知限制

1. `apollo_multithread.c` 使用pthread，已排除（ESP32使用FreeRTOS）
2. `apollo_exec.c` / `apollo_waitlist.c` 中的调度逻辑为模拟实现，非实时
3. 所有I/O为内存模拟，无实际外设交互

## 扩展建议

如需进一步优化性能（针对RV32IMAC无FPU特性）：

1. **定点数学**: 使用Q15/Q31定点替代double
2. **查表法**: 预计算sin/cos表 + 线性插值
3. **CORDIC算法**: 适合无FPU的三角函数实现
4. **float替代double**: 精度稍低但性能更好
