# SRM 校内赛自瞄 —— 配置与运行说明

本文对应代码目录中的校内赛改造版本。协议依据《SRM 校内赛自瞄通信协议》（最终版，来源分支
`infantry_leg&prostrate`，参考提交 `f7b83a58e8dc8af13ae5f328687c2fd7462e8b80`）。

> **目标平台**：x86_64 Ubuntu 22.04（实验室/赛场小电脑）。
> 本文命令均在该平台上执行。

---

## 目录

1. [本次改造做了什么](#1-本次改造做了什么)
2. [环境准备](#2-环境准备)
3. [硬件与权限](#3-硬件与权限)
4. [编译](#4-编译)
5. [配置文件详解](#5-配置文件详解)
6. [分阶段验证流程](#6-分阶段验证流程)
7. [标定流程](#7-标定流程)
8. [故障排查](#8-故障排查)
9. [安全注意事项](#9-安全注意事项)
10. [待办清单](#10-待办清单)

---

## 1. 本次改造做了什么

### 1.1 新增文件

| 文件 | 说明 |
| --- | --- |
| `src/contest.cpp` | 校内赛主程序（相机 + 识别 + 跟踪 + 瞄准 + 新协议通信） |
| `configs/contest.yaml` | 校内赛配置文件，含全部参数注释 |
| `tests/contest_gimbal_test.cpp` | **协议验证工具**，不接相机、不跑神经网络 |
| `tools/config.hpp` | 配置/资源路径运行时回退解析 |
| `contest_deploy.md` | 本文 |

### 1.2 修改文件

| 文件 | 改动 |
| --- | --- |
| `io/gimbal/gimbal.hpp` / `.cpp` | **重写**：实现 18 字节上行 / 30 字节下行帧、流式组帧、读超时、yaw 解缠与 ±180° 归一化、双样本线性插值 |
| `tasks/auto_aim/solver.hpp` / `.cpp` | 新增 `set_R_gimbal2world(yaw, pitch, roll)`，用 `Rz·Ry·Rx` 直接构造姿态，不再依赖四元数 |
| `tasks/auto_aim/aimer.cpp` | 去掉 `if (bullet_speed < 14) bullet_speed = 23;` 硬编码，改为弹速非法时回落到协议默认 22.0 |
| `tasks/auto_aim/tracker.cpp` / `.hpp` | 去掉硬编码的图像中心 `1440x1080`，改为由检测结果反推实际分辨率 |
| `tools/thread_safe_queue.hpp` | 新增 `try_pop(value, timeout)`，用于避免阻塞死锁 |
| `CMakeLists.txt` | 只编译本赛制需要的目标；不再编译 ROS2 / uav / sentry / auto_buff |
| `io/CMakeLists.txt` | 新增轻量目标 `contest_gimbal`（只含通信层，不依赖相机 SDK） |

### 1.3 设计要点（与协议强相关）

| 协议约束 | 代码处理 |
| --- | --- |
| 无帧头/CRC，`body_len` 是唯一帧边界 | `Gimbal::read_thread` 维护持久缓冲区，`body_len != 28` 时丢弃整个缓冲区等待下一次完整帧 |
| USB 包边界 ≠ 协议帧边界 | 一次 `read()` 可能拿到半帧、整帧或多帧，按 `2 + body_len` 切分 |
| 只有 yaw/pitch，没有四元数 | `Solver::set_R_gimbal2world(yaw, pitch, roll)`，因为相机刚连于云台、云台只有两个自由度 |
| 无时间戳 | 以"收到时刻"打标，并对 yaw/pitch 做线性插值 |
| yaw 要求归一化到 ±180° | 对外存储与发送都归一化；**插值内部解缠**，跨越 ±180° 时补 ±360°，避免算出 360° 的假速度 |
| 省略记录时下位机不清零旧字段 | 无有效目标时仍发完整 18 字节帧，并把当前回传 yaw/pitch 回填（`Gimbal::hold()`），避免把云台拽向零位 |
| `fire_flag` 只是视觉侧请求 | 只用 `0 / 1`，不通过异常 ID、超长帧或更高频来绕过下位机安全逻辑 |

### 1.4 为什么 `set_R_gimbal2world(yaw, pitch, roll)` 是正确的

原工程靠下位机回传的四元数得到 `R_gimbal2world`，新协议没有四元数。替代方案是
用两个角直接构造旋转矩阵，依据如下（已数值验证，误差为机器精度）：

1. `tools::rotation_matrix({yaw, pitch, roll})` 恒等于 `Rz(yaw) · Ry(pitch) · Rx(roll)`
   （最大偏差 `1.1e-16`）；
2. 令 `roll = 0`，按 `R(2,0) = -sin(pitch)`、`atan2(R(1,0), R(0,0)) = yaw`
   反解，能精确还原 `(yaw, pitch)`（最大偏差 `2.2e-16`）。
   这与 `tools::eulers(R, 2, 1, 0)` 的分解自洽，
   **因此"下发角度"和"状态估计"处于同一坐标系，不会出现符号或轴向不一致**；
3. 枪管方向（云台 +x 轴）在世界系中的方位角为
   `yaw`、仰角为 `-pitch`，与代码约定"世界系下 pitch 向上为负"一致
   （最大偏差 `1.1e-15`）。

**已知局限（万向锁）**：当 `|pitch|` 接近 90° 时，`Rz·Ry` 的表达会退化、
yaw 不再可靠。程序在 `|pitch| > 85°` 时会打印警告。
自瞄场景 pitch 一般在 ±30° 内，不受影响。

---

## 2. 环境准备

### 2.1 系统依赖

```bash
sudo apt update
sudo apt install -y \
    git g++ cmake \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen
```

说明：

- **不需要** `can-utils`：本赛制不走 SocketCAN。
- **不需要** Ceres：原工程的打符模块用，本赛制已不编译。
- **不需要** ROS2：`CMakeLists.txt` 里的 ROS 目标已移除。

### 2.2 OpenVINO（必需）

推理后端，路径**硬编码在两处**，必须与实际安装一致：

- `CMakeLists.txt` 第 21 行附近
- `tasks/auto_aim/CMakeLists.txt` 第 3 行

默认期望 **2024.6.0** 安装在 `/opt/intel/openvino_2024.6.0/`：

```bash
cd ~
wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f1e1cc13_x86_64.tgz
tar -xf l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f1e1cc13_x86_64.tgz
sudo mv l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f1e1cc13_x86_64 /opt/intel/openvino_2024.6.0
```

若安装到其他路径或其他版本，同步修改上面两处 `OpenVINO_DIR`。

> **GPU 推理是可选的**。CS016 分辨率不大、模型很小，`device: CPU` 通常已经足够，
> 且能省掉 Intel compute-runtime 的一堆依赖。建议**先用 CPU 跑通**，
> 有性能需求再按原 `readme.md` 第 122–139 行安装 compute-runtime 并把
> `device` 改成 `GPU`。

### 2.3 相机 SDK

**两家相机的 SDK 都已随仓库自带**，编译和链接**不需要**额外下载开发包：

| SDK | 头文件 | 库 |
| --- | --- | --- |
| 海康 | `io/hikrobot/include/` | `io/hikrobot/lib/amd64/libMvCameraControl.so` |
| MindVision | `io/mindvision/include/` | `io/mindvision/lib/amd64/libMVSDK.so` |

> ⚠️ **两家都必须有，即使你只用海康。**
> `io/camera.cpp` 是一个"二选一的工厂"，它在**同一个文件里**同时
> `#include` 了两家的头文件（`hikrobot/hikrobot.hpp` 与 `mindvision/mindvision.hpp`）。
> 所以任何编译 `camera.cpp` 的目标都必须同时具备两家的 include 路径。
> 这也是本工程 `io` 目标把相机 SDK 依赖声明为 `PUBLIC` 的原因。
> 如果少了 MindVision 的 include 路径，会出现
> `CameraApi.h: 没有那个文件` 之类的编译错误。

**海康运行时的额外要求**：Linux 下要让 `MV_CC_EnumDevices` 枚举到 USB 相机，
需要安装 MVS 的 **USB 驱动**：

1. 下载并安装 `MVS_STD_GML_V2.1.2`（或更新版本）到默认路径 `/opt/MVS`；
2. 在安装目录下以 root 运行 USB 驱动安装脚本（MVS 安装包自带，通常为
   `MVS/Driver/` 下的 `.run` 或 udev 规则脚本）；
3. 用 `lsusb` 确认能看到海康相机，`vid_pid` 通常为 `2bdf:0001`。

> 若枚举不到相机，参考原 `readme.md` 步骤 3.2 的说明。
> 注意：**编译期不需要这个驱动，只有运行时需要**。
> 所以 `contest_gimbal_test`（只验证串口协议）在没有相机驱动的机器上也能编译和运行。

---

## 3. 硬件与权限

### 3.1 串口权限

USB CDC 设备节点在 Ubuntu 上默认是 `crw-rw---- root dialout`，
即只有 `root` 与 `dialout` 组成员能打开。

**先实测**，不要提前假设：

```bash
ls -l /dev/ttyACM*
```

- 若显示 `crw-rw---- 1 root dialout` → 需要下面的操作之一；
- 若显示 `crw-rw-rw-` → 什么都不用做。

**方式 A（推荐）**：加入 dialout 组

```bash
sudo usermod -a -G dialout $USER
# 重新登录，或临时生效：
newgrp dialout
```

**方式 B**：写 udev 规则（同时可固定设备名）

```bash
sudo tee /etc/udev/rules.d/99-usb-serial.rules <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", ATTRS{serial}=="xxxx", SYMLINK+="gimbal", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

获取 `idVendor` / `idProduct` / `serial`：

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'
```

方式 A 与方式 B **只需其一**：

- 方式 A 解决"权限"；
- 方式 B 的 `MODE="0666"` 也解决"权限"，`SYMLINK` 额外解决"设备名会跳变"。

**方式 C（应急，无需 sudo、无需重新登录）**：

```bash
sg dialout -c ./build/contest configs/contest.yaml
```

> **注意**：小电脑是多人共用的。若没有 sudo，向管理员申请 `usermod -a -G dialout $USER`，
> 理由是它只给"你这个用户"加一个组标签，不改动任何全局配置，也不影响其他参赛者环境。

### 3.2 设备名配置

默认配置写的是 `/dev/ttyACM0`。若你建了软链，把配置改成：

```yaml
com_port: "/dev/gimbal"
```

### 3.3 相机

CS016 通过 USB3 连接。确认：

```bash
lsusb | grep -i hik
```

---

## 4. 编译

```bash
cd <仓库根目录>
cmake -B build
make -C build -j$(nproc)
```

产出两个程序：

| 程序 | 用途 |
| --- | --- |
| `build/contest_gimbal_test` | **先跑这个**，验证通信协议 |
| `build/contest` | 主程序 |

> 原仓库的调试程序（`camera_detect_test`、`detector_video_test`、`camera_test` 等）
> 在本赛制改造中**未被编译**，以缩短编译时间。需要时按 `CMakeLists.txt` 末尾注释里
> 给出的三行加回来即可。

`CMAKE_BUILD_TYPE` 已固定为 `Release`。若需调试符号：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
```

### 4.1 关于工作目录

程序**不再要求必须在仓库根目录启动**：`tools/config.hpp` 会按以下顺序寻找配置与模型：

1. 原样（绝对路径，或相对当前工作目录）
2. 相对可执行文件所在目录
3. 相对可执行文件所在目录的上一级

启动时会打印实际使用的路径，例如：

```
[Config] Loading /home/xxx/sp_vision_25/configs/contest.yaml
[Config] cwd = /home/xxx/sp_vision_25
```

若资源找不到，日志会给出搜索过的目录，按提示修正即可。

---

## 5. 配置文件详解

配置文件：`configs/contest.yaml`

### 5.1 敌方颜色

```yaml
enemy_color: "red"    # "red" | "blue"
```

`Tracker` 用它过滤装甲板。**必须与靶标贴的灯条颜色一致**，否则目标会被全部过滤掉，
表现为"识别得到但一枪都不出"。

### 5.2 姿态正方向（通电后 30 秒内确定）

```yaml
yaw_sign: 1
pitch_sign: 1
```

协议只规定 yaw/pitch 是"角度制"，**没有定义正负方向**。代码内部约定：
世界坐标系下 **pitch 向上为负**。

**实测方法**：

1. 跑 `build/contest`，观察 1 Hz 状态行；
2. 手动把云台抬到明显**仰角**；
3. 看 `gimbal pitch` 的符号：
   - 显示为负 → 与代码一致，`pitch_sign: 1`；
   - 显示为正 → 改为 `pitch_sign: -1`。
4. yaw 同理：手动向一个方向转，看符号是否与你的预期一致。

### 5.3 通信

```yaml
com_port: "/dev/ttyACM0"
baudrate: 115200          # USB CDC 实际不生效，仅作 serial 库参数占位
rx_timeout_ms: 20         # 必须 > 0，否则 read() 永久阻塞
default_bullet_speed: 22.0
```

`rx_timeout_ms` **不要设为 0**。0 表示非阻塞模式，会导致读线程空转占用 CPU。

`default_bullet_speed` 对应协议 4.2：裁判系统未注入弹速时下位机内部默认 22.0。
本程序对 `bullet_speed <= 0` 或超出 `[10, 30]` 的值都会回落到该默认值。

### 5.4 神经网络

```yaml
yolo_name: yolov5
yolov5_model_path: assets/yolov5.xml
classify_model: assets/tiny_resnet.onnx
device: CPU
min_confidence: 0.8
use_traditional: true
```

- `use_traditional` 的含义：**不是**"用传统方法检测"，而是用传统方法
  **二次矫正神经网络给出的四角点**。流程是把神经网络的四点外扩成 ROI，
  在 ROI 内重新二值化找轮廓、重新配对灯条并覆盖角点。
  - `true`：角点精度更高（推荐，靶标单一、背景干净时很稳）；
  - `false`：纯神经网络角点，更快但角点略抖。

- `min_confidence` 可以调高到 `0.9` 来压制误检。

### 5.5 ROI

```yaml
use_roi: false
roi: {x: 0, y: 0, width: -1, height: -1}
```

开启后只在 ROI 内识别，能明显提速。`width/height` 为 `-1` 表示该维度不裁切。
**开启前必须知道相机实际分辨率**，否则 ROI 会落到图像外。

### 5.6 tracker

```yaml
min_detect_count: 5
max_temp_lost_count: 15
outpost_max_temp_lost_count: 75
```

靶标连续旋转时侧面会短暂消失，可把 `max_temp_lost_count` 调到 20–30。

### 5.7 aimer（**命中率的两个关键 offset**）

```yaml
yaw_offset: 0.0             # degree  ★
pitch_offset: 0.0           # degree  ★
comming_angle: 55
leaving_angle: 20
decision_speed: 7
low_speed_delay_time: 0.015   # s  ★
high_speed_delay_time: 0.015  # s  ★
```

- **`yaw_offset` / `pitch_offset`**：同时吸收"相机-枪管外参残差"与"手眼标定残差"。
  这是保证命中的核心参数，必须实测修正。改动建议以 0.5° 为步长。
- **`low/high_speed_delay_time`**：补偿
  图像传输 + 推理 + 通信 + 下位机控制 的总延迟。
  靶标 5 m、弹速 22 m/s 时飞行时间约 **0.23 s**，对延迟极其敏感。
  起步值沿用原工程（`standard4.yaml` 的 0.015），**必须实测**。

### 5.8 shooter

```yaml
first_tolerance: 3     # degree
second_tolerance: 2    # degree  ★
judge_distance: 2      # m
auto_fire: true
```

靶标固定 5 m > `judge_distance`，所以**实际生效的是 `second_tolerance`**。

140×125 mm 靶标在 5 m 处视角约 **1.4°**，`second_tolerance: 2` 偏松，
会浪费子弹。**建议从 0.5–1.0 度开始试**，这是提高命中率最直接的旋钮。

### 5.9 相机与标定

```yaml
camera_name: "hikrobot"
exposure_ms: 2.5
gain: 16.9
vid_pid: "2bdf:0001"
```

靶标原地旋转，压低曝光（1–3 ms）可减少运动模糊；配补光灯后可降低增益。

```yaml
camera_matrix: [...]       # ★ 必须自己标定
distort_coeffs: [...]      # ★
R_gimbal2imubody: [1,0,0, 0,1,0, 0,0,1]   # 新协议无 IMU，不参与运算
R_camera2gimbal: [...]     # ★ 必须自己标定
t_camera2gimbal: [...]     # ★ 必须自己标定
```

`R_camera2gimbal` / `t_camera2gimbal` 决定 `armor.xyz_in_gimbal` 与 `xyz_in_world`，
直接影响测距与 yaw 优化。配置里给的是原工程 `standard3` 的占位值，**必须重新标定**。

### 5.10 planner（本赛制不使用）

协议没有 yaw/pitch 的角速度与角加速度通道，纯角度闭环，因此**不走 MPC Planner 路线**。
`fire_thresh` / `max_yaw_acc` / `Q_yaw` / `R_yaw` 等仅为兼容保留，`contest` 不会读取。

---

## 6. 分阶段验证流程

**严格按顺序做，每一步通过后再进入下一步。** 这样出问题时能立刻定位到层。

### 阶段 1：只验证通信协议（不接相机）

```bash
./build/contest_gimbal_test configs/contest.yaml
```

预期输出（20 Hz 刷新一行）：

```
[t=  12.34s] rx=  1852 ( 150.0Hz) tx=     0 | yaw=  +12.34 pitch=   +3.21 roll=   +0.10 | mode=0 color=0 b=22.00 m/s
```

**验证项**：

| 检查 | 通过标准 | 不通过说明 |
| --- | --- | --- |
| `rx` 持续增长 | Hz 接近下位机发送频率 | 参考[故障排查](#8-故障排查) |
| `parse_errors` 不增长 | 日志无 `Illegal body_len` / `Unknown record id` | 帧格式理解有误 |
| 手动转动云台 | `yaw`/`pitch` 跟着变 | 字段偏移或顺序理解有误 |
| `b`（弹速） | 非 0，或为 22.0（裁判系统未注入） | 下位机未发弹速 |
| 符号一致性 | 抬头时 `pitch` 为**负** | 把 `pitch_sign` 改为 `-1` |

**交互命令**：

```
h                  回到当前回传角度（fire_flag = 0）
s <yaw> <pitch>    发送指定角度（度），不开火
f <yaw> <pitch>    发送指定角度（度），请求开火
q                  退出
```

用 `f 0 0` 之类的命令确认**云台确实转到指定角度**，说明 18 字节上行帧被正确接受。

一次性发送模式（用脚本批量验证时方便）：

```bash
# 只发一帧 yaw=0 pitch=0，然后打印 1 秒的回传状态并退出
./build/contest_gimbal_test configs/contest.yaml --send-yaw=0 --send-pitch=0

# 发一帧并请求开火
./build/contest_gimbal_test configs/contest.yaml --send-yaw=-10 --send-pitch=5 --fire
```

> 注意：因为 OpenCV 的 `CommandLineParser` 会把 `-f` 当成"负数参数"而报错，
> 所以这里用 `--send-yaw=` / `--send-pitch=` / `--fire` 这种等号形式的选项。

### 阶段 2：只验证相机与识别

```bash
./build/contest configs/contest.yaml --no-fire
```

**先确认相机分辨率**：日志里 `Camera` 若无输出，用 OpenCV 或 MVS 客户端看一下实际尺寸。
若与 `roi` 配置不符，先关掉 `use_roi`。

**验证项**：

| 检查 | 通过标准 |
| --- | --- |
| 图像能读到 | 无 `Empty img!, camera drop!` |
| 靶标能被检出 | `targets=1` |
| 靶标名字正确 | 日志中 `ARMOR_NAMES` 不是 `not_armor` |
| 测距合理 | `d` 接近实际距离（5 m 附近） |
| pitch 方向 | 云台向靶标方向转动，不是反方向 |

若 `targets` 一直为 0，重点查两件事：

1. `enemy_color` 是否与靶标灯条颜色一致；
2. 靶标上的数字是否被识别成有效的 `ArmorName`。
   分类器输出直接写入 `armor.name`，若输出 `not_armor`（8）或 `base`（7），
   会被 `check_name` / `check_type` 过滤掉。
   若靶板贴的是 **"1"**，会被识别成 `one`，在 `check_type` 里被过滤，
   表现为**完全检不到**。

**调试建议**：临时把 `use_traditional` 设为 `false`，若此时能检出而 `true` 不能，
说明传统矫正的 `threshold` 需要调整（靶标灯条亮度不同，阈值可能要调低到 100–130）。

### 阶段 3：开火闭环

```bash
./build/contest configs/contest.yaml
```

先在**不装弹**的情况下确认 `fire=1` 出现的时机是否合理（云台已经跟上目标时才出现）。

### 阶段 4：参数标定

见下一节。

---

## 7. 标定流程

顺序很重要：**内参 → 外参 → offset → 延迟 → 容差**。

三个程序一次性解决前两步：

| 程序 | 作用 | 需要什么 |
| --- | --- | --- |
| `calib_capture` | 采集图片 + 同时记录云台 yaw/pitch/roll | 相机 **+ 云台通电** |
| `calib_camera` | 由图片解算内参 | 只要有图片（**不需要云台**） |
| `calib_handeye` | 由图片 + 记录的角度解算外参 | 需要内参已填回 yaml |

> 这三个程序是新加的（原 `calibration/*.cpp` 依赖 `io::CBoard`/CAN 和四元数，本赛制用不了）。
> 原文件保留未删，只改了默认路径与图案类型两处，仍然可用于原 RoboMaster 环境。
> 配置用 `configs/calibration_contest.yaml`（与 `configs/calibration.yaml` 分开）。

### 7.0 准备标定板

```yaml
pattern_cols: 10          # 点的个数，不是外框尺寸
pattern_rows: 7
center_distance_mm: 40    # ★ 用卡尺实测，别信印刷标称值
pattern_type: circle      # circle = 对称圆点阵；chessboard = 棋盘格
```

**注意默认图案是圆点阵**（`cv::findCirclesGrid`），与 `configs/calibration.yaml` 原有的
10×7 约定一致。如果你手上是**棋盘格**，改成 `chessboard`。

### 7.1 采集数据（内参 + 外参共用同一批）

```bash
./build/calib_capture configs/calibration_contest.yaml -o calib_imgs
```

窗口会显示：

| 叠加内容 | 用途 |
| --- | --- |
| `yaw / pitch / roll` | 确认角度正常、无零漂、正方向对 |
| `saved N (this session M)` | 累计位姿数 / 本次会话存了几张 |
| `min pair angle: xx.x deg` | **已存位姿里最小的两两夹角** ← 核心指标 |
| `vs saved (max): xx.x deg` | 当前位姿与已存位姿的最大夹角 |
| `PATTERN OK` / `NO PATTERN` | 没检到图案时按 `s` 会被拒绝保存 |

**按键**：`s` 保存一组，`q` 退出。

#### 转角提示怎么读

`min pair angle` 会按颜色分级：

| 显示 | 含义 | 该怎么做 |
| --- | --- | --- |
| `-- (first shot)` | 只有一张，还没有"对" | 继续采 |
| `GOOD`（绿，≥25°） | 所有位姿两两都分得很开 | 可以采下一张 |
| `OK (>=17.2)`（黄） | 刚过 TSAI 门槛 | 建议再拉开一些 |
| `TOO SMALL (<17.2)`（红） | **TSAI 会丢弃这些运动对** | 把云台转得更开，别在这里继续采 |

`vs saved (max)` 是绿色说明**这一张给位姿集合带来了新的旋转信息**；显示橙色说明
它和已有位姿太接近，**存了价值不大**，应该换个角度。

> ⚠️ **为什么是 17.2°**：`cv::calibrateHandEye` 的 TSAI 方法会丢弃
> `|P| = 2·sin(θ/2) < 0.3` 的运动对，反解出来就是 `θ < 17.2°`。
> 而你的云台只有 2 个自由度，位姿不散开时有效方程会不足——
> 此时函数会打印 `Not enough informative motions` 并**静默返回单位阵**。
> 只跑一次默认方法的话，你会把一个错误的外参当成成功结果。
> 所以**盯住屏幕上这个数字**，比事后看日志高效得多。

程序退出时还会给出总结，若 `min pair angle < 17.2°` 会直接报错提醒重采。

#### 旧数据怎么处理

如果 `calib_imgs/` 里已经有上一次采集的数据，程序**不会静默覆盖**，而是先扫描并询问：

```
  输出文件夹 calib_imgs 里已有旧数据
    图片 20 张（最大编号 20）
    可读姿态 20 组
    无法解析的姿态 0 组（可能是旧版 capture.cpp 写的四元数格式）
  [c] 清空旧数据，从 1 开始重新采集   （板子/相机动过时选这个）
  [a] 保留旧数据，接着往后编号        （板子/相机完全没动时才能选）
  [q] 退出，什么都不做
```

也可以跳过询问，用参数直接指定：

```bash
--on-existing=clear    # 清空旧数据，从 1 开始
--on-existing=append   # 保留旧数据，从 max+1 继续
--on-existing=ask      # 默认，交互询问
```

**怎么选——这是有对错之分的**：

| 情况 | 选择 | 原因 |
| --- | --- | --- |
| 标定板或相机**动过** | **`clear`** | 板子一动，`AX=XB` 的前提（目标在世界系不动）就破了，新旧混用会**静默算出错误外参** |
| 标定板、相机、车**全都没动** | 可以 `append` | 两次的数据同源，可以累积 |
| 只想重标外参、保留内参的图 | 建议**分别放不同目录** | 例：内参采到 `calib_intrinsic/`，外参采到 `calib_extrinsic/` |

**建议的保守做法：每次标定用一个新的输出目录。**
例如 `-o calib_imgs_0917`、`-o calib_imgs_0918`。这样永远不会有"混用了不同批次"的风险，
而且出问题时可以回溯是哪一批数据算出来的。

**清空操作是安全的**：只删除文件名是**纯数字**的 `.jpg` / `.txt`，
其它文件（比如你放进去的说明、备份）一律保留。

**关于旧版四元数文件**：如果你之前用原工程 `capture.cpp` 采过数据，
那些 `.txt` 是单行 `wxyz` 四元数。程序能识别出来（多了第 4 个数），
会列入"无法解析"：
- **转角提示不会把它们计入**（因为解读方式不同，硬算会得到错误角度）；
- `calib_handeye` 也会跳过它们；
- 但它们对应的 `.jpg` **仍可用于内参标定**，不会被浪费。

#### 采集要点

| 项目 | 要求 |
| --- | --- |
| 张数 | 15~25 组 |
| 云台位姿 | **要散开**：yaw 覆盖 ±30°、pitch 覆盖 ±15°（看屏幕上的 `min pair angle`） |
| 标定板位置 | 相对于**世界固定不动**；但要充满画面、倾斜、铺到四角 |
| 车体 | 全程**不许移动**（`t_gimbal2world` 被硬编码为 0） |
| 镜头 | 锁死对焦环，固定曝光与增益 |
| 平整度 | 标定板必须贴平 |

产出的配对文件：`calib_imgs/1.jpg` + `calib_imgs/1.txt`（三行，度，yaw/pitch/roll）。

### 7.2 内参

```bash
./build/calib_camera --config-path=configs/calibration_contest.yaml calib_imgs
```

程序会逐张显示识别结果（**按任意键继续**），最后输出可直接粘贴的两行：

```
===== 复制下面两行到 yaml =====
# 重投影误差: 0.1832px
camera_matrix: [1785.48..., 0, 672.48..., 0, 1785.02..., 559.89..., 0, 0, 1]
distort_coeffs: [-0.0760..., 0.1118..., 0.00053..., -0.00275..., 0]
================================
```

把它粘贴回 **`configs/calibration_contest.yaml`**，替换那里的占位值。

**验收：重投影误差要 < 0.5 px。** 同时看 `fx`：
`fx × 像元尺寸 ≈ 镜头焦距`。你的 CS016 像元约 3.45 μm、镜头 6 mm，
所以 `fx` 应约 1740~1800。**对不上就说明标定错了。**

> 注意 `cv::calibrateCamera` 用了 `CALIB_FIX_K3`，所以 `distort_coeffs` 只有 5 个数，
> 第 5 个恒为 0。这与 `contest.yaml` 的写法一致。

### 7.3 外参

**前提：7.2 的结果已经填回 yaml**（`calib_handeye` 需要内参做 PnP）。

```bash
./build/calib_handeye --config-path=configs/calibration_contest.yaml calib_imgs
```

程序会把 **DEFAULT（不指定方法，即原实现行为）+ 5 种方法**各打印一份结果，并附上判读信息：

```
===== 方法 PARK =====
===== 复制下面两行到 configs/contest.yaml =====
R_camera2gimbal: [...]
t_camera2gimbal: [...]
===============================================

相机轴在云台系中的方向（用于判断结果是否合理）：
  相机 x  ...
  光轴 z  +7.31° 相对云台 +x（枪管方向）
  相机位置  +0.1316 +0.1038 +0.0249 m（相对云台原点）
```

**怎么选**：

1. 先用 **DEFAULT / TSAI**；
2. 如果它报 `Not enough informative motions` 或结果与其它方法差很多 → 位姿不够散开，**重新采集**；
3. 各方法结果接近（光轴偏角相差 1° 以内）才说明数据质量够好。

**判读光轴偏角**：你确认过相机是刚性固连、光轴与云台 0 方向无偏角，
所以这个角应在 **1~2°** 量级（纯装配公差）。
若算出 7° 以上，不是标定错了就是机械确实歪了。

**把结果粘到 `configs/contest.yaml` 的 `R_camera2gimbal` / `t_camera2gimbal`。**

#### 外参的最终验收（必须做）

不要只看数字。用**重投影**验证：

1. 把标定板放在云台正前方一个已知距离（比如 3 m）、和云台大致等高；
2. 云台转到 `yaw=0, pitch=0`；
3. 用 `auto_aim::Solver::world2pixel()` 把该已知世界点投影回图像；
4. **投影点应落在标定板实际位置上**，误差几个像素内算合格。

代码里已有现成工具：`solver.world2pixel()`（世界点→像素）、
`solver.reproject_armor()`（装甲板位姿→四角像素），
`src/auto_aim_debug_mpc.cpp` 就是把估计位置画回图像上的例子。

> **关于"相机↔枪口"的提醒**：`t_camera2gimbal` 描述的是
> **相机相对云台坐标系原点**（yaw 轴与 pitch 轴交点）的位姿，这是手眼标定
> 在几何上能给出的东西。而代码算瞄准角时用的也是这个原点
> （`atan2` 作用在 `xyz_in_gimbal` 上），所以两者自洽。
> 只有当"两轴交点不在炮膛轴线上"时才会引入随距离变化的误差（约 `偏移/距离`）。
> 你之前确认车体固定、比赛固定 5 m，所以只要 **`yaw_offset`/`pitch_offset` 在 5 m 处标定**，
> 这个误差会被吸收掉。若以后需要跨距离都准，再加枪口偏移参数。

### 7.4 角度偏置（`yaw_offset` / `pitch_offset`）

**最简单也最有效的方法**：打固定靶，看弹着点相对靶心的偏差。

- 偏左/偏右 → 调 `yaw_offset`；
- 偏高/偏低 → 调 `pitch_offset`。

步长建议 0.5°，每次记录弹着点，直到居中。

### 7.5 延迟（`low_speed_delay_time` / `high_speed_delay_time`）

**方法一（慢动作视频）**：拍摄击发瞬间，测"图像采集 → 子弹出膛"的时间。

**方法二（扫参）**：固定靶 + 固定云台，让 `low_speed_delay_time` 依次取
`0.000 / 0.005 / 0.010 / 0.015 / 0.020 / 0.025 / 0.030`，每档打 20–50 发，
统计命中数，取命中率最高的档位。

**方法三（日志量化）**：`contest` 已输出每帧的
`图像时间戳`（`t`）与 `rx` 计数，配合原工程的 `recorder` 可量化图像传输与推理耗时。

> 由于协议**不提供时间戳**，程序只能以"收到时刻"打标，USB 传输抖动会直接
> 叠加到预测时间上。这是本协议的固有局限，只能靠实测延迟参数吸收。

### 7.6 开火容差（`second_tolerance`）

`second_tolerance` 越小，开火越保守、命中率越高，但单位时间射击窗口越少。
建议从 `1.0` 开始，逐步降到 `0.5`，观察命中率与总用弹量的平衡。

---

## 8. 故障排查

### 8.1 启动即失败

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `Failed to open serial /dev/ttyACM0` | 设备名不对 / 权限不足 / 未插线 | `ls -l /dev/ttyACM*`；确认 dialout 组 |
| `Cannot find config file` | 路径不对 | 日志会打印 cwd 与 exe 目录，按提示传路径 |
| `OpenVINO not found` | 未装或路径不符 | 修改两处 `OpenVINO_DIR` |
| `Not found camera!` | 海康 USB 驱动未装 | 见 2.3 |

### 8.2 收不到下行帧

```
[Test] 3 秒内没有收到任何下行帧
```

依次检查：

1. `com_port` 是否正确（`ls -l /dev/ttyACM*`）；
2. 当前用户是否在 `dialout` 组（`id | grep dialout`）；
3. 下位机是否上电并确实在发送；
4. **下位机帧长是否为 30 字节**（`body_len = 28`）。
   若下位机实际是其他长度，程序会持续打印
   `Illegal body_len XXX, dropping buffer.` —— 这说明协议版本不一致，
   需要按下位机实际协议调整 `io/gimbal/gimbal.hpp` 里的常量。

### 8.3 能收到但解析失败

```
[Gimbal] Unknown record id N, frame dropped.
[Gimbal] Incomplete frame (id1=.., id2=..), dropped.
```

说明"ID 顺序或数据体长度"的理解与下位机不一致。
对照协议文档核对 `io/gimbal/gimbal.hpp` 里的这四个常量：

| 常量 | 含义 | 协议值 |
| --- | --- | --- |
| `DL_GIMBAL_RECORD_ID` | 下行 ID 1 | `1` |
| `DL_SHOOT_RECORD_ID` | 下行 ID 2 | `2` |
| `DL_GIMBAL_BODY_SIZE` | 下行 ID 1 数据体长度 | `20` |
| `DL_SHOOT_BODY_SIZE` | 下行 ID 2 数据体长度 | `4` |

上行方向的对应常量是 `UL_GIMBAL_BODY_SIZE`（`8`）与 `UL_SHOOT_BODY_SIZE`（`4`）。

> 命名说明：`UL_` = uplink = 上位机→下位机，`DL_` = downlink = 下位机→上位机。
> 因为 `Gimbal_Receive_s` 是"下位机 Receive"（即上行），
> `Gimbal_Send_s` 是"下位机 Send"（即下行），两者容易混淆，所以常量统一用 UL/DL 前缀。

所有结构体都有 `static_assert` 长度校验，改错长度会**编译期**报错，而不是运行期出错。

### 8.4 云台被拽向零位

若忘记回填（老代码行为），无目标时视觉会持续把 yaw/pitch 写成 0，云台会往零位甩。
本程序用 `Gimbal::hold()` 回填当前角度解决。
若仍出现，检查 1 Hz 日志里 `hold` 计数是否在增长。

### 8.5 检不到靶标

见 [阶段 2](#阶段-2只验证相机与识别) 的排查清单，核心是
`enemy_color` 与靶标数字分类两件事。

---

## 9. 安全注意事项

1. **`fire_flag` 只是请求**。实际发射还需满足下位机火控状态、摩擦轮状态与人工许可。
   非零值不会绕过这些条件。
2. **不要用异常手段催促发射**。协议第 5 条第 4 点明确禁止通过异常 ID、超长帧或
   高频重复帧绕过下位机安全逻辑。
3. **调试时优先用 `--no-fire`**，确认瞄准正确后再开火。
4. **云台角度有裁剪**。程序对 pitch 做了 ±45° 的防御性裁剪，
   但这只是兜底；机械限位由下位机保证。
5. **异常估计会抑制开火**。当目标水平距离 > 30 m、或 yaw/pitch 非有限值时，
   程序不发开火请求并回填保持。
6. **退出时会明确清零**。程序退出前调用 `gimbal.hold(false)`，
   保证 `fire_flag = 0`。
7. 未获安全员许可不得发射。

---

## 10. 待办清单

以下项目**尚未完成**，需要在实验室小电脑上继续：

- [ ] 确定部署目录与用户名，把 `contest.yaml` 里的模型/资源路径改成绝对路径
- [ ] 实测并填写 `pitch_sign` / `yaw_sign`
- [ ] 标定 `camera_matrix` / `distort_coeffs`
- [ ] 标定 `R_camera2gimbal` / `t_camera2gimbal`
- [ ] 标定 `yaw_offset` / `pitch_offset`
- [ ] 标定 `low_speed_delay_time` / `high_speed_delay_time`
- [ ] 根据实测收紧 `second_tolerance`
- [ ] 确认靶标数字能被正确分类为有效 `ArmorName`
- [ ] 确认相机实际分辨率，必要时启用 `use_roi` 并填写 ROI
- [ ] 确认下位机下行帧确实为 30 字节（`body_len = 28`）
- [ ] 视需要补一个延迟标定程序（`delay_calib`）
- [ ] 视需要补充 `autostart.sh` / `.desktop` 自启动配置
      （注意原 `autostart.sh` 引用了仓库里不存在的 `watchdog.sh`）

---

## 附：常用命令速查

```bash
# 串口权限检查
ls -l /dev/ttyACM*
id | grep dialout

# 相机识别
lsusb | grep -i hik

# 编译
cmake -B build && make -C build -j$(nproc)

# 阶段 1：只测协议
./build/contest_gimbal_test configs/contest.yaml

# 阶段 2：不装弹调试瞄准
./build/contest configs/contest.yaml --no-fire

# 阶段 3：完整闭环
./build/contest configs/contest.yaml

# 查看下位机原始字节（确认帧长与内容，需要先停掉占用串口的程序）
sudo cat /dev/ttyACM0 | xxd | head -20
```
