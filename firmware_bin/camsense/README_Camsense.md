# Camsense 激光雷达固件切换说明

## 结论

当前主控板固件用的是 **LD-STL-26（STL-26）**：

- `main/config/network_config.h`：`kLidarBaudRate = 230400`
- `main/tasks/sensors/lidar_task.cpp`：`Ldstl26Lidar`
- 启动日志：`LDSTL26: UART1 230400`、`LIDAR_TASK: Lidar task started on UART1, RX=41`

已按参考工程 `leap_ros_mcu_driver-main (1)/` 切换到 **Camsense**。

## 源码改动

| 文件 | 改动 |
|---|---|
| `main/tasks/sensors/lidar_task.cpp` | 改用 `CamsenseLidar`，角度按参考工程做 `(360 - angle) % 360` 镜像 |
| `main/config/network_config.h` | `kLidarBaudRate` 从 `230400` 改为 `115200` |
| `main/CMakeLists.txt` | `ldstl26_lidar` 改为 `camsense_lidar` |
| `components/camsense_lidar/CMakeLists.txt` | 补 IDF 6 的 `esp_driver_uart` / `esp_driver_gpio` |

Camsense 组件本身与参考工程一致，直接复用。

## 接线

- Camsense TX → ESP32-S3 **GPIO41**
- Camsense 5V → 5V，GND → GND
- 串口：UART1，**115200 8N1**
- 原来 STL-26 要拔掉，不要和 Camsense 同时接在 UART1 上

注意：Camsense 如果是 5V TTL 输出，确认 ESP32 GPIO41 不会过压；必要时加电平转换。

## 烧录

固件已编译好：

```text
leap_ros_max_camsense.bin   1,154,080 字节
SHA256 ac4c35d64180c8c7532db1eb988369b71b808934ce58dcb8a04026b3a5ffe2a5
```

**只烧应用分区 `0x10000`，不要烧 `0x0` 合并镜像**，否则会擦掉 NVS 里的 WiFi 和 micro-ROS agent 配置。

```bat
flash_camsense_app.bat COM5
```

或手动：

```bat
python -m esptool --chip esp32s3 -p COM5 -b 460800 ^
  --before default-reset --after hard-reset ^
  write-flash -z --flash-mode dio --flash-freq 80m --flash-size 4MB ^
  0x10000 leap_ros_max_camsense.bin
```

烧录后板子重启，WiFi 和 agent 配置保留。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source /root/chuyun_ws/install/local_setup.bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

- 有 `/scan` 数据：Camsense 工作正常。
- 没有数据：先确认波特率 115200、TX 接 GPIO41、供电和 GND；再检查 Camsense 型号是否确实是 0x55 0xAA 协议。
- 角度方向不对：修改 `lidar_task.cpp` 里的镜像公式。
