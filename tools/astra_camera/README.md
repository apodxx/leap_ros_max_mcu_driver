# Orbbec Astra Pro 深度相机（ROS 2 Humble / Jetson）

用 Orbbec 官方 `OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux.tar.gz` 里的 `ros2_astra_camera`
在 Jetson Orin NX 上跑通 Astra Pro：深度 + RGB + 点云。

实测环境：Jetson Orin NX，Ubuntu 22.04，ROS 2 Humble，相机序列号 `18060620419`。

---

## 1. 硬件识别

```
2bc5:0403  Orbbec Astra Pro        <- 深度（OpenNI2）
2bc5:0501  Orbbec Astra Pro HD     <- RGB（标准 UVC，必须单独开）
```

`ros2 run astra_camera list_devices_node` 会打印 `Device: Astra  URI: 2bc5/0403@1/9`。
注意：OpenNI 里名字是 **Astra**，但它带独立的 UVC 彩色相机，实际是 Astra Pro 的形态。

---

## 2. 依赖

```bash
sudo apt install -y libgflags-dev nlohmann-json3-dev libuvc-dev \
  ros-humble-image-publisher ros-humble-image-geometry
```

**libuvc 要用源码版**（apt 的 0.0.6 没有 `LibUVC::UVCShared` 这个 CMake target，编译会报
`Target "astra_camera" links to target "LibUVC::UVCShared" but the target was not found`）：

```bash
git clone --depth 1 https://github.com/libuvc/libuvc.git
cd libuvc && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false
sudo cmake --build build --target install && sudo ldconfig
```

**magic_enum**（0.8.0，记得关掉 tests，否则在 Jetson 上会编译一堆测试用例）：

```bash
curl -L -o magic_enum.tar.gz https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz
tar xzf magic_enum.tar.gz && cd magic_enum-0.8.0
cmake -S . -B build -DMAGIC_ENUM_OPT_INSTALL=ON -DMAGIC_ENUM_OPT_BUILD_TESTS=OFF
sudo cmake --build build --target install && sudo ldconfig
```

---

## 3. 编译与 udev

```bash
mkdir -p ~/ros2_ws/src
tar xzf OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux.tar.gz -C ~/ros2_ws/src/
cd ~/ros2_ws/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 4. 为什么加 astra_pro 配置

包里自带的 `astra_mini_params.yaml` 是 `enable_color: false`，只出深度和 IR。
Astra Pro 的 RGB 走独立 UVC，所以本目录提供：

- `astra_pro_params.yaml`：`enable_color: true`、`uvc_camera.enable: true`、`vid 0x2bc5 / pid 0x0501`，
  深度和 RGB 都 640×480@30，IR 关闭省 USB 带宽
- `astra_pro.launch.py`：加载上面的参数，并启动 `PointCloudXyzNode`

安装到包里（或直接把两个文件拷进 `src/ros2_astra_camera/astra_camera/{params,launch}/` 后重编译）：

```bash
cp astra_pro_params.yaml ~/ros2_ws/src/ros2_astra_camera/astra_camera/params/
cp astra_pro.launch.py   ~/ros2_ws/src/ros2_astra_camera/astra_camera/launch/
cd ~/ros2_ws && colcon build --packages-select astra_camera
```

---

## 5. 运行

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch astra_camera astra_pro.launch.py
# 或者本目录的脚本：./start_astra_pro.sh
```

后台常驻：

```bash
nohup ~/ros2_ws/start_astra_pro.sh > /tmp/astra.log 2>&1 &
pkill -f component_container     # 停止
```

### 话题

| 话题 | 类型 | 实测频率 |
|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` 16UC1（mm）640×480 | ~17.7 Hz |
| `/camera/color/image_raw` | `sensor_msgs/Image` bgr8 640×480（MJPEG） | ~16.5 Hz |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | ~27 Hz |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz |
| `/camera/ir/image_raw` | 默认关闭 | — |
| `/tf`, `/tf_static` | `camera_depth_frame ↔ camera_color_frame`（基线 -25 mm） | 10 Hz |

实测抓帧：深度最大 1521 mm、有效像素 26.6%、中位数 1205 mm；彩色与深度图见 PR 描述里的两张截图。

> 深度只有 ~17 Hz 是 USB 2.0 带宽被 MJPEG 彩色占用所致。把 `astra_pro_params.yaml` 里
> `enable_color` 改成 `false`，深度能回到 ~29.7 Hz。

---

## 6. 验证抓帧

```bash
python3 grab_frame.py          # 订阅一帧彩色 + 一帧深度，存 /tmp/astra_color.png、/tmp/astra_depth.png
```

同时会打印深度统计（min/max/有效像素比例/中位数），用来确认不是“有话题没数据”。

---

## 7. 踩过的坑

| 现象 | 原因 / 处理 |
|---|---|
| 编译报 `LibUVC::UVCShared` 找不到 | apt 的 libuvc 太旧，按第 2 节源码装 0.0.9 |
| magic_enum 编译很慢 | 默认会编测试，加 `-DMAGIC_ENUM_OPT_BUILD_TESTS=OFF` |
| 有 `/camera/color/*` 话题但没数据 | `enable_color` 为 false，或没开 `uvc_camera.enable` / pid 不对 |
| 话题时有时无、`fastrtps_port7411 open_and_lock_file failed` | 重复启动了多个 `component_container`；`pkill -f component_container` 后清 `/dev/shm/fastrtps_*` 再起 |
| `ros2 node list` 为空但话题正常 | ROS 2 daemon 缓存问题，`ros2 daemon stop` 后重试 |
| 普通用户打不开相机 | 没装 udev 规则：`scripts/install.sh` + `udevadm trigger` |
