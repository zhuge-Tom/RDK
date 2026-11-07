# RDK：ROS 2 红蓝装甲检测、跟踪与 PnP 初版

海康 MV-CS016-10UC 通过 USB3 接 RDK S100。RDK 完成取图、红蓝灯条配对、角点检测、跟踪与预测，电脑通过 SSH 转发在浏览器查看结果。

~~~text
海康相机 → RDK MVS SDK → ROS 2 相机节点 → C++ 装甲检测
→ 跟踪 / PnP / 预测 → 可视化 JPEG → ROS 2 浏览器桥接 → SSH → 电脑浏览器
~~~

main 对应“相机接 RDK，电脑看浏览器”。电脑直连海康相机的独立流程在 [pc-direct-hikrobot 分支](https://github.com/zhuge-Tom/RDK/tree/pc-direct-hikrobot)。

**2026-11-07 更新**：红色已接入与蓝色共用的检测、PnP、跟踪和预测管线，支持红/蓝/双颜色启动；同时改进倾斜配对和近距离 LED 断点连接，浏览器新增检测诊断。用户反馈现场初测效果可用，当前不支持整块装甲旋转约90°的姿态。现场反馈不等于测距、动态预测或全角度召回率验收。

启动见[红色识别与切换](docs/red-armor.md)，实现与验证见[灯条几何优化](docs/armor-geometry.md)，测距阶段见[项目进度](docs/blue-armor-progress.md)。

## 1. 当前能做什么

| 功能 | 状态 |
|---|---|
| 红蓝装甲框、四角点 | 已在实际相机使用；蓝色默认，支持 red / both |
| 二维跟踪、像素速度、100 ms 预测中心 | C++ 实现；无标定时也能运行 |
| ROS 2 节点、类型化消息、CameraInfo | 已接入，四个 C++ 节点同进程通信 |
| 浏览器简洁 / 调试模式 | 支持角点、轨迹、帧率和状态显示 |
| 保存标定照片 | 通过 ROS 服务保存全分辨率、无绘图的 PNG |
| 照片筛选、隔离、相机内参计算 | 提供脚本；标定默认只使用 accepted 目录 |
| 装甲 PnP、相机坐标系三维跟踪与预测 | 已实现；显式未核验调试模式已用于静态测距 |

当前开发机已用微调光圈后入选的 16 张照片重算候选内参，拟合 RMS 约 0.189 px。用户已更正拍摄顺序：这 16 张对应当前固定的对焦和光圈设置，仓库新增的 10 张是微调前的旧照片，未通过初版清晰度筛选，没有参与拟合。灯条尺寸 W 仍约 0.13 m；用户后来将 H 更正为 58–60 mm，运行取中间值 0.059 m，取代最初约 0.04 m。当前可用未核验 PnP 调试模式做实际距离对比，测距精度仍待核验；拟合误差不等于实际测距误差。

截至 2026-11-07，已保存 1.00、1.50、2.00、2.50 米四组中心参考，每组 10 秒、50 次状态抽样；均值分别约为 1.002、1.480、1.993、2.479 米。3 米附近那组量到装甲侧面，单独暂存，不用于中心误差比较。当前阶段是静态 PnP 初测完成、动态跟踪与预测实测待进行。见[进度和误差记录](docs/blue-armor-progress.md)及[原始数值记录](docs/measurements/2026-11-static/README.md)。

核心算法为 C++；Python 负责 ROS / HTTP 桥接与离线标定。坐标采用 OpenCV 相机光学系：X 向右、Y 向下、Z 向前。输出四角点顺序为左上、右上、右下、左下；只提供视觉数据及可视化。

## 2. 硬件和软件要求

| 位置 | 要求 |
|---|---|
| RDK | S100，Ubuntu 22.04 ARM64，ROS 2 Humble，GCC 11 / CMake 3.22 或更新版本 |
| 相机 | 实测为海康 MV-CS016-10UC，USB3，当前封装按 BayerBG8 取图 |
| RDK SDK | 海康 MVS Linux aarch64 SDK，默认安装到 /opt/MVS |
| RDK 图像库 | C++ OpenCV 4.5.4；Python OpenCV 与 NumPy |
| 电脑 | Ubuntu、Git、OpenSSH 客户端、curl 和浏览器 |

电脑只看浏览器时不需要安装 ROS 2 或 MVS。相机由 MVS SDK 打开，通常不表现为 /dev/video0；使用本程序时应关闭占用该相机的 MVS 客户端或其他取图程序。

若使用其他型号、GigE 或其他像素格式，需要先适配 [MVS 相机封装](src/mvs_camera.hpp)。当前枚举的是 USB 相机，选择枚举到的第一台。

## 3. 连接 RDK

本文默认：

| 配置 | 默认值 |
|---|---|
| RDK SSH 用户及地址 | sunrise@192.168.1.50 |
| 电脑有线地址 | 192.168.1.100/24 |
| RDK 部署目录 | /home/sunrise/rdk_blue_vision |
| 电脑浏览器端口 | 8876 |

先给 RDK 供电，将相机接 RDK USB3，再用网线连接电脑和 RDK。在 RDK 本地确认其有线地址；实际地址不同可通过 RDK_HOST 覆盖。

电脑端查看网卡名称：

~~~bash
ip -brief link
~~~

以下以 USB 网卡 enx00e04c0a4d30 为例。只有尚未创建 RDK-direct 连接时才执行创建命令：

~~~bash
RDK_IFACE=enx00e04c0a4d30
nmcli connection add type ethernet ifname "$RDK_IFACE" con-name RDK-direct \
  ipv4.method manual ipv4.addresses 192.168.1.100/24 \
  ipv4.never-default yes ipv6.method disabled
~~~

已有该连接时，直接启用并检查：

~~~bash
RDK_IFACE=enx00e04c0a4d30
nmcli connection up RDK-direct ifname "$RDK_IFACE"
cat "/sys/class/net/$RDK_IFACE/carrier"
ip route get 192.168.1.50
ping -I "$RDK_IFACE" -c 2 -W 2 192.168.1.50
ssh sunrise@192.168.1.50
~~~

carrier 应为 1，路由应经过上述有线网卡。VPN 可能改变路由，应以绑定网卡的 ping 为准。SSH 使用板端自己的账户和密码。

## 4. 在 RDK 安装依赖

以下命令在 **RDK 的终端**执行。板端已有 ROS 2 Humble 时直接补齐依赖；没有 ROS 2 时先按 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)配置软件源、安装 Humble。

~~~bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config libopencv-dev python3-opencv python3-numpy \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-rclcpp-components \
  ros-humble-rclpy ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-geometry-msgs ros-humble-std-srvs ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime

source /opt/ros/humble/setup.bash
/usr/bin/python3 -c 'import rclpy, cv2, numpy; print("Python OpenCV:", cv2.__version__)'
~~~

从 [海康官方下载页](https://www.hikrobotics.com/en/machinevision/service/download?module=0)选择适合 ARM64 的 Linux MVS SDK，解压并按包内说明运行安装程序。可参照[厂商 Linux 安装指南](https://www.hikrobotics.com/en2/source/vision/video/2022/12/19/Standard%20Camera_How%20to%20Install%20MVS%20in%20Linux_22.12.17.pdf)；安装包文件名随版本变化，RDK 要选 aarch64。

安装后在 RDK 确认：

~~~bash
test -f /opt/MVS/include/MvCameraControl.h
test -f /opt/MVS/lib/aarch64/libMvCameraControl.so
lsusb
~~~

本机实测 SDK 动态库为 libMvCameraControl.so.4.7.0.3。若普通用户无法打开 USB 相机，按 MVS 安装说明配置设备权限并重新插拔相机。

## 5. 在电脑克隆、部署和启动

以下命令在 **电脑终端**执行：

~~~bash
git clone --branch main https://github.com/zhuge-Tom/RDK.git "$HOME/RDK"
cd "$HOME/RDK"

bash scripts/deploy-blue-vision.sh
bash scripts/run-blue-vision-browser.sh
~~~

部署脚本上传源码到 RDK，在板端编译、安装并运行 CTest。启动脚本建立 SSH 转发，启动 ROS 2 流程并尝试打开浏览器：

- 普通页面：http://127.0.0.1:8876/
- 调试页面：http://127.0.0.1:8876/#debug

红色识别在电脑使用下面的入口（先结束旧会话）：

~~~bash
cd "$HOME/RDK"
bash scripts/run-red-vision-browser.sh

# 或同时检测红蓝，二选一启动
# ARMOR_COLOR=both bash scripts/run-blue-vision-browser.sh
~~~

上述命令默认仅开启二维检测与预测。需要PnP时，按第7节提供自己的标定；原开发设备也可显式选择仓库内的未核验调试快照。

运行终端需要保持打开，Ctrl+C 结束这次相机及浏览器服务。重新启动前先结束旧会话，避免相机或端口被重复占用。

若用户、地址或部署目录不同，部署和启动都使用相同的环境变量：

~~~bash
export RDK_HOST=your_user@your_rdk_ip
export RDK_DIR=/home/your_user/rdk_blue_vision
bash scripts/deploy-blue-vision.sh
bash scripts/run-blue-vision-browser.sh
~~~

如果只需要打开交互式 RDK 终端，可直接执行 ssh sunrise@192.168.1.50；仓库也提供 [等待有线连接后打开终端的脚本](scripts/open-rdk-terminal.sh)，其 RDK_INTERFACE 和 RDK_HOST 可覆盖。

## 6. 运行参数与画面检查

参数文件是 [config/blue_vision_ros.yaml](config/blue_vision_ros.yaml)。相机参数、检测阈值在启动时读取，修改后重新启动。

蓝色分割使用 `gray_min=110`，红色使用 `red_gray_min=60` 与两段红色色相。红蓝共用 `close_kernel_height=15`、灯条面积下限45、旋转框长边下限16px、短边下限3px、长短边比1.8，并约束方向、错位、边界和片段合并。调试区的 `detection_diagnostics` 显示候选及剔除计数，断流时清空。参数针对当前设备，其他相机/灯条需结合画面调整；角度阈值不等于实拍识别角度承诺。

| 环境变量 | 默认值 | 含义 |
|---|---:|---|
| ARMOR_COLOR | blue | 目标颜色：blue / red / both；红色入口固定 red |
| EXPOSURE_US | 3500 | 曝光，微秒 |
| CAMERA_GAIN | 2 | 相机增益 |
| CAMERA_FPS | 30 | 请求的采集帧率 |
| PROCESSING_WIDTH | 960 | 检测宽度，保持完整视场比例 |
| JPEG_QUALITY | 75 | JPEG 质量 |
| PREDICTION_MS | 100 | 预测时间，毫秒 |
| POSE_CALIBRATION | 空 | RDK 内的标定配置路径 |
| BROWSER_PORT | 8876 | 电脑上的转发端口 |
| ROS_DOMAIN_ID | 42 | 此模块 ROS 域 |

蓝色灯条日常检测使用默认曝光。室内拍棋盘格较暗时，可以在电脑用以下设置启动：

~~~bash
EXPOSURE_US=60000 CAMERA_GAIN=16 CAMERA_FPS=15 \
  bash scripts/run-blue-vision-browser.sh
~~~

这是当前室内标定的临时设置。60 ms 曝光需要相机和棋盘停稳，补光后可缩短曝光。当前相机实际增益上限约 16.98，设置 18 会被 SDK 拒绝；其他相机应读取自己的范围。

电脑检查服务：

~~~bash
curl --noproxy '*' http://127.0.0.1:8876/status
~~~

正常应有 stream_alive=true、runtime=ros2。detected 表示当前帧是否检测到装甲；拍棋盘时该值可以为 false。调试模式的“保存标定照片”保存 RDK 原始 PNG，与棋盘是否已经检出无关。

## 7. 标定、筛选和启用 PnP

摆放和保存照片可先看 [棋盘格标定拍照指南](docs/棋盘格标定拍照指南.md)，含位置示意和 30 张拍摄清单。
详细计算步骤见 [CALIBRATION.md](CALIBRATION.md)，包括打印棋盘、测量实际格子边长、拍摄、隔离照片及实测灯条尺寸。

在 RDK 先查看筛选报告，再将选中的照片复制到 accepted、未选中的原图移到 quarantine：

Git 历史中已有的 `calibration/` 照片是开发历史素材，不是当前新批次。采新照片时请用独立目录，避免混入旧镜头状态。下面所有输入和输出目录都以同一新批次命名。

~~~bash
cd /home/sunrise/rdk_blue_vision

# 采集前，在电脑启动相机时设置独立目录：
# ARMOR_CAPTURE_DIR=calibration/session-new/captures bash scripts/run-blue-vision-browser.sh

# 在 RDK：只检查，生成报告
python3 scripts/review_calibration.py \
  --images calibration/session-new/captures \
  --report calibration/session-new/review-preview.json

# 检查报告后应用；输入、accepted、quarantine 为三个独立目录
python3 scripts/review_calibration.py --apply \
  --images calibration/session-new/captures \
  --accepted calibration/session-new/accepted \
  --quarantine calibration/session-new/quarantine \
  --report calibration/session-new/review-applied.json

# 只读取本批入选图
python3 scripts/calibrate_camera.py \
  --images calibration/session-new/accepted \
  --columns 9 --rows 6 --square-mm 19 \
  --output config/pose_camera.measured.yml
~~~

本机打印格子实测为 19 mm。模板设计为 20 mm，其他打印板必须实际测量后更改参数。已有 accepted 集或输出配置时脚本会要求使用新目录 / 文件名，避免覆盖旧结果。

PnP 所需物理尺寸是两灯条中心线间距和单根灯条有效发光长度，单位为米。将实测尺寸填入自己的配置，核对内参、分辨率和报告后开启调试，再通过已知距离验证结果：

~~~bash
# 在 RDK：从自己的标定结果创建运行配置
cp config/pose_camera.measured.yml config/pose_camera.yml
# 编辑 pose_camera.yml，填写 armor_width_m、armor_height_m；
# 填好实测尺寸、核对本机内参后，保留 calibration_verified: 0，
# 添加 allow_unverified_pnp: 1，进入显式未核验调试。

# 在电脑：结束旧会话后启动
POSE_CALIBRATION=config/pose_camera.yml bash scripts/run-blue-vision-browser.sh
~~~

[pose_camera.example.yml](config/pose_camera.example.yml) 是保持禁用状态的模板。只有完成独立实测核验后，才考虑将自己的 `calibration_verified` 标记设为 1。

复现**原开发相机与固定镜头状态**的当前初版，可在电脑部署后显式选择随仓库保存的[未核验设备快照](config/pose_camera.rdk-hikrobot-debug.yml)：

~~~bash
POSE_CALIBRATION=config/pose_camera.rdk-hikrobot-debug.yml \
  bash scripts/run-blue-vision-browser.sh
~~~

打开 `http://127.0.0.1:8876/#debug`，应显示“PnP 调试 · 距离未核验”，`/status` 的 `geometry.reason` 为 `debug_unverified`，检测成功时包含 `geometry.poses`。界面区分直线距离 R、前向深度 Z、XYZ 和重投影误差；简洁模式也保留未核验提示。该快照不是其他相机的通用标定，更换镜头状态后应重新标定。

在电脑保存一组静态数值：

~~~bash
# 先用卷尺量镜头前端到两灯条之间、灯条高度中间的中心点，并保持静止无遮挡。
python3 scripts/collect_static_pnp.py \
  --output "$HOME/rdk-pnp-checks/session-2m" \
  --reference-m 2.00 --uncertainty-m 0.01 \
  --calibration config/pose_camera.rdk-hikrobot-debug.yml
~~~

输出目录必须是新的，默认采样 10 秒、50 次；这是 HTTP 状态抽样，不能当作逐帧检出率。`--calibration` 保存电脑上的同名配置副本，需与 RDK 实际启用文件一致；它不会改变运行参数。详细测量基准和数值限制见 [CALIBRATION.md](CALIBRATION.md)。

距离和速度是相机相对量；目前未接 IMU / 里程计补偿、整车旋转模型或控制输出。

## 8. RDK 本机直接构建与运行

也可直接在已安装依赖的 RDK 克隆 main：

~~~bash
git clone --branch main https://github.com/zhuge-Tom/RDK.git /home/sunrise/rdk_blue_vision
cd /home/sunrise/rdk_blue_vision
source /opt/ros/humble/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build --clean-first --parallel 4
ctest --test-dir build --output-on-failure
cmake --install build
bash scripts/run-rdk-ros2.sh
~~~

然后在电脑另开终端转发端口，并打开浏览器：

~~~bash
ssh -N -L 8876:127.0.0.1:8876 sunrise@192.168.1.50
~~~

安装MVS并成功生成真实相机目标时，应有七组 CTest 通过；ROS 测试使用独立域 91 和合成图像，不打开真实相机。类型化消息、话题和诊断命令见 [ROS2.md](ROS2.md)。

本轮红蓝更新检查：RDK原生完整清理重编译，七组CTest全部通过（ROS覆盖八种红/蓝/双模式阶段）。实机红色启动20次HTTP抽样均在线且检出装甲，约30FPS。合成倾斜/LED间隙回归、五张保存实拍的旧新版回放和浏览器JS诊断检查通过。用户随后反馈现场初测可用；未测出精确的角度/近距极限，三维精度与动态预测仍需独立验证。

**升级已有部署**：本次`Armors`和`VisionResult`新增`diagnostics_json`字段。先结束旧相机会话，再完整重编译并安装全部消息及节点；上面的`--clean-first`也避免电脑/板端时钟不一致造成旧目标被误复用。相机SDK、标定文件及新照片仍在设备本地维护。

前一版蓝色静态测距发布检查（2026-11-07）：核对板端核心源码及检测参数与待发布版本哈希一致，复跑板端现有构建的六组 CTest，全部通过。另在 x86_64 / ROS 2 Jazzy / OpenCV 4.6 容器中以 `BUILD_MVS_DIRECT=OFF` 独立构建，三组 C++ 回归通过；用本次发布的设备配置复算 250 组保存角点，距离与原记录最大差约 4.8e-10 m。浏览器调试提示、R/Z 显示、目标丢失/断流清空和便携采样脚本的 HTTP 记录流程也已检查。容器检查不代替 RDK 的 ARM64 MVS 取流，合成/回放检查不代替动态实测。

2026-10-28 在上述 RDK 环境中，从独立源码目录重新构建、安装，并通过全部六组测试；部署脚本和安装后的 ROS 2 启动入口也完成验证。另用 37 张原图的副本复测筛选流程，得到 19 张入选、18 张隔离，重算 RMS 约 0.393 px。合成输入测试与拟合误差不替代真实相机的距离、运动和预测精度核验。

## 9. 常见问题

| 现象 | 检查 |
|---|---|
| 网线连接不上 | 电源、carrier、有线 IP、绑定网卡的 ping；VPN 是否改了路由 |
| 编译找不到 MVS / 启动缺少 blue_vision_ros | ARM64 SDK 是否安装在 /opt/MVS，依赖是否完整 |
| 相机打开失败 | 相机是否出现在 lsusb，USB 权限、SDK 返回码、是否被其他程序占用 |
| Browser / 8876 already in use | 先结束旧会话；不要再次打开同一相机 |
| 浏览器无法打开 | 使用 HTTP；核对 SSH 转发；curl 加 --noproxy '*'；检查本机代理 |
| 画面暗 | 拍棋盘时使用补光或较长曝光；蓝灯条检测与棋盘拍摄的曝光需求不同 |
| 保存照片失败 | 看页面具体错误：服务发现、响应超时、相机断流或写入错误；检查 RDK 磁盘空间 |
| 只有二维结果 | 检查配置路径、真实灯条尺寸，以及是否显式启用未核验调试或使用已核验标定 |
| CLI 看不到 ROS 节点 | source ROS 环境，匹配 ROS 域与本机设置，使用 --no-daemon --spin-time 5 |

MVS 的 XOpenDisplay Fail 在当前无桌面取流测试中未影响视频；仍应以相机是否实际出图为准。

## 10. 源码结构

~~~text
src/armor_detector.hpp           灯条过滤、配对和二维角点
src/image_tracking.hpp           二维像素跟踪和预测
src/pose_tracking.hpp            标定、IPPE PnP、三维跟踪和预测
src/mvs_camera.hpp               MVS USB 相机封装
src/vision_output.hpp            绘图、状态 JSON、原始照片保存
src/ros_nodes.cpp                四个 C++ ROS 2 节点
src/blue_vision_ros.cpp           同进程节点执行器
msg/                            类型化装甲、姿态和轨迹消息
scripts/rdk_ros_browser.py       ROS 订阅、采集服务客户端
scripts/rdk_mjpeg_browser.py     HTTP 与双模式页面
scripts/review_calibration.py    照片筛选、清单与隔离
scripts/calibrate_camera.py      相机内参计算
scripts/collect_static_pnp.py    静态 PnP 数值采样
config/blue_vision_ros.yaml      当前节点配置
config/pose_camera.rdk-hikrobot-debug.yml  原开发设备的未核验 PnP 快照
~~~

相机 SDK 由本机独立安装，新的照片与生成的标定文件默认不纳入 Git。本次发布保存源码、启动工具、设备调试快照和数值测量记录；历史提交中的照片仅作开发档案。原始直采版本保留作对照，停止 ROS 2 会话后可用 VISION_BACKEND=direct 启动同一个蓝色浏览器入口；当前灯条门槛调参针对默认 ROS 2 路径。
