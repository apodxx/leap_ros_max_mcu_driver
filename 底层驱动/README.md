# 麦轮小车底层驱动系统

基于STM32H743的智能投送机器人底层驱动，包含四路电机控制、编码器反馈、IMU惯性导航和精确路径跟踪。

## 📁 文件结构

```
底层驱动/
├── tb6612_motor.h/c          - TB6612四路电机驱动（X型麦轮布局）
├── encoder.h/c                - 四路编码器驱动（TIM1/2/3/5编码器模式）
├── imu660rc.h/c              - IMU660RC六轴陀螺仪驱动（I2C接口）
├── mecanum_kinematics.h/c    - 麦轮运动学控制（正逆运动学、里程计）
├── pid_controller.h/c        - PID控制器（速度和位置闭环）
├── path_tracking.h/c         - 路径跟踪系统（基于PID的导航控制）
├── navigation_controller.h/c - 导航主控制器（集成所有模块）
└── main_example.c            - 完整示例程序
```

## 🔌 硬件引脚配置

### TB6612 四路电机驱动
- **共用使能**: STBY = PB0
- **电机A (左前)**: PWM_A=PD12, AIN1=PD8, AIN2=PD9
- **电机B (左后)**: PWM_B=PD13, BIN1=PD10, BIN2=PD11
- **电机C (右前)**: PWM_C=PD14, CIN1=PC0, CIN2=PC1
- **电机D (右后)**: PWM_D=PD15, DIN1=PA7, DIN2=PB14

### 四路编码器
- **编码器1 (对应电机A)**: E1A=PA0, E1B=PA1 (TIM5_CH1/CH2)
- **编码器2 (对应电机B)**: E2A=PB4, E2B=PB5 (TIM3_CH1/CH2)
- **编码器3 (对应电机C)**: E3A=PA8, E3B=PE11 (TIM1_CH1/CH2)
- **编码器4 (对应电机D)**: E4A=PA15, E4B=PB3 (TIM2_CH1/CH2)

### IMU660RC 陀螺仪
- **I2C接口**: SCL=PB8, SDA=PB9 (I2C1)
- **I2C地址**: 0x68 (SA0=低) 或 0x6A (SA0=高)
- **控制引脚**: CS=PB1, SA0=PC5, INT1=PB2

## 🚗 麦轮X型布局

```
        前
    A ↗   ↖ C
      \   /
       \ /
    中心点
       / \
      /   \
    B ↙   ↘ D
        后
```

- **A**: 左前轮 (Front Left)
- **B**: 左后轮 (Rear Left)
- **C**: 右前轮 (Front Right)
- **D**: 右后轮 (Rear Right)

## 🔧 使用步骤

### 1. 添加文件到项目

将 `底层驱动/` 文件夹中的所有 `.c` 和 `.h` 文件添加到你的STM32项目中：
- 在Keil/IAR/STM32CubeIDE中添加源文件
- 将头文件路径添加到Include路径

### 2. 配置CubeMX（如果使用）

**I2C1配置**:
- 模式: I2C
- 速度: Fast Mode (400kHz)
- PB8: I2C1_SCL
- PB9: I2C1_SDA

**注意**: 电机PWM和编码器定时器由驱动代码内部配置，无需在CubeMX中配置。

### 3. 集成示例代码

参考 `main_example.c` 中的代码：

```c
#include "navigation_controller.h"

I2C_HandleTypeDef hi2c1;
NavController_t nav_controller;

int main(void)
{
    /* 系统初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    
    /* 初始化导航控制器 */
    NavController_Init(&nav_controller, &hi2c1, 0.02f);  // 20ms控制周期
    
    /* 定义路径点 */
    Waypoint_t path[] = {
        {0.0f, 0.0f, 0.0f, 0.2f},    // x, y, theta, speed
        {1.0f, 0.0f, 0.0f, 0.2f},
        {1.0f, 1.0f, 1.57f, 0.2f},
    };
    
    /* 启动自动导航 */
    NavController_StartAutoNav(&nav_controller, path, 3);
    
    /* 主循环 */
    while (1)
    {
        NavController_Update(&nav_controller);  // 周期性调用
        HAL_Delay(1);
    }
}
```

## 📊 参数调整

### 机器人物理参数
在 `mecanum_kinematics.h` 中修改：

```c
#define ROBOT_WHEEL_BASE        0.20f   // 轮距(前后) 单位:米
#define ROBOT_TRACK_WIDTH       0.20f   // 轮距(左右) 单位:米
#define ROBOT_WHEEL_RADIUS      0.05f   // 轮子半径 单位:米
#define ENCODER_PPR             1560    // 编码器每转脉冲数
```

### PID参数调整

```c
/* 设置位置和角度PID参数 */
PathTracker_SetPIDParams(&nav_controller.path_tracker,
    1.5f, 0.01f, 0.2f,   // 位置PID: Kp, Ki, Kd
    2.0f, 0.05f, 0.3f);  // 角度PID: Kp, Ki, Kd
```

**调试建议**:
1. 先调节位置Kp，观察响应速度
2. 如有稳态误差，增加Ki
3. 如有震荡，增加Kd或减小Kp
4. 角度控制通常需要更大的Kp值

### 速度限制

在 `mecanum_kinematics.c` 的 `Mecanum_InverseKinematics()` 函数中：

```c
const float PWM_PER_MPS = 2000.0f;  // PWM值/速度比例
```

根据电机实际特性调整此值。

## 🎮 控制模式

### 1. 手动控制模式

```c
/* 设置速度: vx(前进), vy(左移), omega(旋转) */
NavController_SetManualVelocity(&nav_controller, 0.2f, 0.0f, 0.0f);
```

### 2. 自动路径跟踪模式

```c
/* 定义路径点数组 */
Waypoint_t waypoints[] = {
    {x, y, theta, speed},
    // ...
};

/* 启动自动导航 */
NavController_StartAutoNav(&nav_controller, waypoints, count);
```

### 3. 停止和复位

```c
NavController_Stop(&nav_controller);   // 停止所有运动
NavController_Reset(&nav_controller);  // 复位位姿和里程计
```

## 📈 状态监控

```c
/* 获取当前位姿 */
Position_t pos = NavController_GetPosition(&nav_controller);
printf("X=%.3f Y=%.3f Theta=%.2f\n", pos.x, pos.y, pos.theta);

/* 获取当前状态 */
NavState_t state = NavController_GetState(&nav_controller);
// NAV_STATE_IDLE, NAV_STATE_MANUAL, NAV_STATE_AUTO
```

## ⚙️ 高级配置

### IMU融合权重调整

```c
nav_controller.imu_fusion_weight = 0.8f;  // 0.0-1.0
// 1.0 = 完全使用IMU航向角
// 0.0 = 完全使用里程计航向角
```

### 到达容差调整

```c
nav_controller.path_tracker.position_tolerance = 0.05f;  // 位置容差(米)
nav_controller.path_tracker.angle_tolerance = 0.1f;      // 角度容差(弧度)
```

## 🐛 故障排查

### IMU初始化失败
- 检查I2C接线和地址
- 使用I2C扫描工具确认设备地址
- 确认SA0引脚电平

### 电机不转
- 检查STBY引脚是否使能(PB0=高)
- 检查PWM输出波形
- 检查电机驱动板供电

### 编码器读数异常
- 检查编码器接线
- 确认编码器电源
- 使用示例程序单独测试编码器

### 路径跟踪不准确
1. 校准编码器PPR值
2. 测量实际轮距参数
3. 调整PID参数
4. 检查IMU校准状态

## 📝 注意事项

1. **坐标系定义**: 
   - X轴: 机器人前方为正
   - Y轴: 机器人左侧为正
   - 角度: 逆时针为正（弧度制）

2. **单位系统**:
   - 位置: 米(m)
   - 速度: 米/秒(m/s)
   - 角度: 弧度(rad)
   - 角速度: 弧度/秒(rad/s)

3. **控制周期**: 建议20-50ms，在主循环中周期性调用 `NavController_Update()`

4. **电机方向**: 如果实际运动方向与预期相反，需要在电机驱动中调整IN1/IN2逻辑或修改运动学公式符号

## 📚 API参考

详细API说明请参考各头文件中的注释。

## 🔗 相关资料

- STM32H7参考手册
- TB6612电机驱动芯片数据手册
- IMU660RC传感器数据手册
- 麦轮运动学理论

---

**作者**: Claude (Anthropic)  
**日期**: 2026-09-10
**版本**: v1.0

## 与 Qt 上位机联调

STM32 工程通过 WIFI6B21-SPI 以 UDP 客户端连接上位机，目标地址和 WiFi 参数在 `Drivers/User/Inc/wifi_spi.h` 中配置，目标端口为 `8081`。启动 Qt 上位机后点击“启动 UDP”，收到 MCU 遥测后即可使用“自动正方形”“发送手动速度”“急停”“复位”按钮。

协议详情见上位机目录中的 `NETWORK_PROTOCOL.md`。上位机工程位于 `../upper_computer_QT`，其 UDP 接收和控制代码在 `mainwindow.cpp`。
