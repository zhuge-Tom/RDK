# RDK 蓝色视觉 ROS 2 运行结构

本模块使用 RDK 上现有 ROS 2 Humble。电脑只通过 SSH 转发查看浏览器，
不要求电脑安装 ROS 2 或海康 SDK。海康相机仍通过 RDK 的 MVS SDK 打开。
main 分支提供这套 RDK 流程，完整依赖安装与部署步骤见 [README.md](README.md)。

## 节点与话题

| 节点 | 输入 | 输出 |
|---|---|---|
| `/hik_camera`，C++ | MVS USB3 相机 | `/camera/image_raw`、`/camera/camera_info` |
| `/armor_detector`，C++ | `/camera/image_raw` | `/armor/detections` |
| `/armor_tracker`，C++ | `/armor/detections` | `/armor/tracks` |
| `/armor_visualizer`，C++ | 同一时间戳的原图与 `/armor/tracks` | `/armor/visualization/compressed`、`/armor/status` |
| `/armor_browser_bridge`，Python | 可视化压缩图、状态话题 | HTTP 视频、状态及页面；调用相机标定照片服务 |

标定照片通过 ROS 2 服务 `/camera/save_calibration_frame` (`std_srvs/srv/Trigger`)
请求，由相机节点保存当前未缩放、未绘图的原始 PNG。
浏览器显示图片来自可视化话题，预测数据来自跟踪节点。
MVS 取图定时器使用独立的互斥回调组，采集服务使用默认回调组；
两者通过短时间持锁的只读原始帧快照交接，PNG 保存期间继续取图。
这样可避免阻塞式取图占住同一回调组，导致采集及相机参数服务长时间无响应。

| 话题 | 类型 | 主要字段 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 完整视场缩放后的 BGR8、采集时间戳、相机坐标系 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 对应图像分辨率的内参和畸变；未标定时 K[0]=0 |
| `/armor/detections` | `rdk_s100_blue_armor_detector/msg/Armors` | 原图时间戳、颜色、四角点、处理耗时和检测帧率 |
| `/armor/tracks` | `rdk_s100_blue_armor_detector/msg/VisionResult` | 二维轨迹、预测中心、标定状态及可用的三维结果 |
| `/armor/visualization/compressed` | `sensor_msgs/msg/CompressedImage` | 已绘制装甲框和预测点的 JPEG，保留原图时间戳 |
| `/armor/status` | `std_msgs/msg/String` | 面向浏览器的 JSON，包括 runtime=ros2 和阶段耗时 |

四角点顺序是左上、右上、右下、左下。二维坐标/速度单位为 px、px/s；
三维坐标/速度单位为 m、m/s，相机坐标 X 向右、Y 向下、Z 向前。
无有效标定时，三维结果为空，二维跟踪与预测继续运行。

## 性能与时间同步

四个 C++ 节点在同一个进程内运行，启用 rclcpp 同进程通信，并共享只读图像。
检测和跟踪通过自定义 ROS 消息连接。可视化按采集时间戳匹配图像与结果，
仅为绘图复制图像，最终进行一次 JPEG 编码。
图像/检测/跟踪队列深度为 1，匹配缓存有固定上限，超过 250 ms 的旧结果不显示。
Python 桥接节点只接收压缩 JPEG 和状态，不解码或重新编码视频。
OpenCV 单线程配合三线程 ROS 执行器，避免各节点重复扩展内部线程。

默认 `ROS_LOCALHOST_ONLY=1`、`ROS_DOMAIN_ID=42`，所有 ROS 数据都留在 RDK。
电脑通过现有 HTTP/SSH 链路看视频，不通过网线传输 ROS 原始图像。
`/status` 的 `capture_to_encode_ms` 是 RDK 内采集完成到 JPEG 编码完成的时间，
不包含传输、浏览器渲染和相机曝光时间，不能当作完整端到端延迟。

## 构建、测试与启动

在 RDK 源码目录：

```bash
cd /home/sunrise/rdk_blue_vision
source /opt/ros/humble/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
cmake --install build
bash scripts/run-rdk-ros2.sh
```

第六组回归测试使用独立 ROS 域 91，不打开真实相机。
它用真实节点验证未标定/合成已标定两条路径、类型化消息、CameraInfo、浏览器视频、
标定采集服务、断流清空和正常退出。合成标定通过不等于真实相机已标定。

电脑常用入口不变：

```bash
cd ~/RDK
bash scripts/deploy-blue-vision.sh
bash scripts/run-blue-vision-browser.sh
```

当前独立蓝色模块入口默认选择 ROS 2。原来的直采程序仍可用于对照：

```bash
VISION_BACKEND=direct bash scripts/run-blue-vision-browser.sh
```

相机打开具有独占性。切换前先 Ctrl+C 结束原会话，确认相机进程和服务已退出；
不要同时启动两个后端。启动脚本在其中一个子进程退出时会停止同会话的另一个进程。

## 配置与诊断

节点参数集中在 `config/blue_vision_ros.yaml`，在启动时读取并设为只读；
修改阈值/相机参数后重启。常用环境覆盖仍保留：
`EXPOSURE_US`、`CAMERA_GAIN`、`CAMERA_FPS`、`PROCESSING_WIDTH`、
`JPEG_QUALITY`、`PREDICTION_MS`、`POSE_CALIBRATION`、`ARMOR_CAPTURE_DIR`。
默认曝光 3500 us、增益 2、采集 30 FPS、检测宽度 960、预测 100 ms。
真实标定步骤参见 `CALIBRATION.md`。

另开 RDK 终端诊断，必须使用相同的 ROS 域：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/rdk_blue_vision/install/share/rdk_s100_blue_armor_detector/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42
ros2 node list --no-daemon --spin-time 5
ros2 topic list -t --no-daemon --spin-time 5
ros2 topic hz /armor/tracks
ros2 topic echo /armor/tracks --once --qos-reliability best_effort
ros2 param list /armor_detector
```

首次发现 ROS 图需要几秒钟。使用 `--no-daemon` 避免已有 ROS CLI 后台进程沿用
其他域或网络设置。诊断终端的 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY` 必须与
运行进程一致；自定义启动环境时也要同步修改诊断命令。

浏览器调试入口：`http://127.0.0.1:8876/#debug`。
HTTP 的 `/status`、`/stream`、`/capture` 保持兼容；断流后清空旧预测。
如需录制回放，可使用 rosbag，注意录制原始图像会增加磁盘和通信负担。

四个 C++ 类也注册为 rclcpp 组件，后续可以在标准组件容器中加载，
或与原 RMCS 工程对接；当前消息接口是本模块新增的定义，尚未宣称完全兼容原 RMCS 接口。

## 2026-10-27 板端验证记录

RDK 原生重新配置和编译成功，六组 CTest 全部通过，耗时 12.30 s。
五个节点及文档列出的六个视觉话题、原始照片采集服务均已发现；
真实相机经 ROS 2 输出 960×720 画面，浏览器状态为 `runtime=ros2`。
连续 30 次采样全部在线，检测/显示帧率中位数均约 30 FPS，
采集完成到编码完成中位数约 32.9 ms，检测阶段约 7.9 ms。
这些耗时不包含电脑浏览器的完整显示延迟。

本轮实机画面没有点亮的蓝色装甲灯条，检测数为零；目标角点和预测的
ROS 消息链路由合成输入测试覆盖，仍需放入真实蓝色装甲验证实机效果。
真实相机状态为 `calibration_missing`，完成棋盘格标定和灯条尺寸测量前，
三维定位与三维预测保持禁用。

同日修复了实机取图阻塞导致采集服务超时的问题：相机取图与默认服务
使用不同回调组，持锁只用于取得原始帧快照，PNG 编码在锁外执行。
六组回归再次全部通过；当前 60 ms / 增益 16 / 15 FPS 实机连续三次
保存 1440×1080 原始 PNG，HTTP 响应约 0.12 s，相机参数查询也恢复响应。
验证照片单独保存在 RDK 的 `calibration/service-check-20261027/`，
未放入用于正式标定的 `calibration/captures/`。
