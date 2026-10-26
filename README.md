# RDK S100 蓝色装甲板识别

面向 RoboMaster 视觉开发的蓝色装甲灯条检测项目。`main` 分支要求海康相机连接在 RDK S100 上，提供两条可复现流程：

1. 相机采集和 C++ 检测都在 RDK 上运行，电脑只显示浏览器画面。
2. RDK 采集相机并通过 SSH 发送画面，电脑运行 ROS 2 C++ 检测节点和浏览器。

程序只输出检测可视化、二维装甲框和四个像素角点，不包含射击、云台或运动控制。海康相机直接连接电脑的独立流程位于 [`pc-direct-hikrobot`](https://github.com/zhuge-Tom/RDK/tree/pc-direct-hikrobot) 分支。

## 检测算法

- HSV 蓝色分割与灰度亮度联合筛选
- 竖向闭运算，连接灯条中过曝发白造成的断裂区域
- 面积、尺寸、长宽比和蓝红通道比例过滤
- 灯条高度、纵向偏差、间距及装甲比例配对
- 输出二维框和四角点，顺序为左上、右上、右下、左下
- 浏览器显示识别状态、角点坐标、检测 FPS 和显示 FPS

实机环境中，RDK 直连模式以 960×720 处理时约为 30 FPS。

## 文件结构

```text
src/blue_armor_mvs_detector.cpp   RDK 直接调用 MVS 的 C++ 检测程序
src/blue_armor_detector_node.cpp  电脑端 ROS 2 图像检测节点
config/blue_armor_detector.yaml   ROS 2 检测参数
scripts/deploy-to-rdk.sh          从电脑上传源码并在 RDK 编译
scripts/run-rdk-local.sh          RDK 本机检测和网页服务
scripts/run-rdk-browser.sh        电脑启动 RDK 检测、SSH 转发并打开浏览器
scripts/mvs_raw_mjpeg.py          RDK 相机原始画面输出
scripts/mjpeg_ros_bridge.py       电脑 ROS 2 图像发布和浏览器服务
scripts/run-pc-via-rdk.sh         RDK 采集、电脑检测的完整启动脚本
```

## 已验证环境

- RDK S100：Ubuntu 22.04、ROS 2 Humble、aarch64
- 海康相机：MV-CS016-10UC，USB3
- RDK MVS SDK：安装在 `/opt/MVS`，包含 ARM64 动态库
- 电脑：Ubuntu、ROS 2 Humble 或 Jazzy、OpenCV 4
- RDK 默认网口地址：`192.168.1.50`

MVS SDK 由海康提供，不包含在本仓库中。RDK 必须先能通过 MVS 打开相机。该工业相机通常不会显示为 `/dev/video0`。

## 开始前检查

电脑连接 RDK 网线后执行：

```bash
ping -c 2 192.168.1.50
ssh sunrise@192.168.1.50
```

在 RDK 上检查相机和 MVS：

```bash
lsusb | grep -i Hikrobot
test -f /opt/MVS/include/MvCameraControl.h && echo "MVS headers OK"
test -f /opt/MVS/lib/aarch64/libMvCameraControl.so && echo "MVS library OK"
```

同一时刻只能有一个程序独占海康相机。启动前请关闭 MVS 客户端或其他相机进程。

## 流程一：RDK 运行检测，电脑浏览器查看

这是延迟更低的推荐流程。电脑只接收已经画好检测框的 JPEG。

在电脑执行：

```bash
git clone --branch main https://github.com/zhuge-Tom/RDK.git
cd RDK

RDK_HOST=sunrise@192.168.1.50 bash scripts/deploy-to-rdk.sh
RDK_HOST=sunrise@192.168.1.50 bash scripts/run-rdk-browser.sh
```

`deploy-to-rdk.sh` 会把源码上传到 `/home/sunrise/rdk-s100-blue-armor-detector`，使用 RDK 自己的 GCC、OpenCV、ROS 2 和 MVS SDK 编译。`run-rdk-browser.sh` 会启动 RDK 检测、建立 SSH 端口转发并自动打开：

```text
http://127.0.0.1:8876/
```

调整相机参数时，可以先 SSH 到 RDK，再执行：

```bash
cd /home/sunrise/rdk-s100-blue-armor-detector
EXPOSURE_US=3000 CAMERA_GAIN=2 CAMERA_FPS=30 \
PROCESSING_WIDTH=960 JPEG_QUALITY=75 \
bash scripts/run-rdk-local.sh
```

按 `Ctrl+C` 会停止检测并释放相机。

## 流程二：RDK 采集相机，电脑运行 ROS 2 检测

电脑需要安装 ROS 2、`cv_bridge` 和 OpenCV。RDK 需要 Python 3、NumPy、OpenCV Python 和 MVS Python binding。

在电脑建立 ROS 2 工作区：

```bash
mkdir -p ~/blue_armor_ws/src
cd ~/blue_armor_ws/src
git clone --branch main https://github.com/zhuge-Tom/RDK.git

cd ~/blue_armor_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rdk_s100_blue_armor_detector \
  --cmake-args -DBUILD_MVS_DIRECT=OFF
source install/setup.bash
```

运行完整链路：

```bash
cd ~/blue_armor_ws/src/RDK
RDK_HOST=sunrise@192.168.1.50 \
RDK_MVS_PYTHON_PATH=/home/RM_Vision \
bash scripts/run-pc-via-rdk.sh
```

脚本会执行以下流程：

```text
RDK 海康相机 → MVS → JPEG/SSH → 电脑 /image_raw
→ ROS 2 C++ 检测节点 → 浏览器 http://127.0.0.1:8876/
```

如果 RDK 的 `MvImport` 不在 `/home/RM_Vision`，把 `RDK_MVS_PYTHON_PATH` 改为包含 `MvImport` 目录的上一级路径。

## ROS 2 接口

输入：

- `/image_raw`：`sensor_msgs/msg/Image`，编码为 BGR8

输出：

- `/blue_armor_detector/visualization`：带灯条、装甲框和角点的图像
- `/blue_armor_detector/mask`：蓝色亮区二值掩膜
- `/blue_armor_detector/armors`：`sensor_msgs/msg/PointCloud`

`armors` 每四个点表示一个装甲板，顺序为 TL、TR、BR、BL。channels 分别为 `armor_id`、`corner_id` 和 `geometry_score`。

## 验证

浏览器打开后检查状态接口：

```bash
curl --noproxy '*' http://127.0.0.1:8876/status
```

识别到一个装甲板时会返回类似：

```json
{
  "detected": true,
  "armors": 1,
  "corners": [[131.3, 139.5], [747.8, 213.1], [715.1, 474.5], [101.7, 398.0]],
  "detect_fps": 30.0,
  "web_fps": 30.0
}
```

绿色框为装甲四边形，红点为四角点，紫色框为二维包围框。连续移动相机或装甲板，确认画面、坐标和 FPS 持续更新。

## 参数调整

电脑 ROS 2 模式使用 `config/blue_armor_detector.yaml`。RDK 直连程序的同名阈值位于 `src/blue_armor_mvs_detector.cpp` 的 `Config`。

灯条中心大面积发白时先降低曝光。每次只调整一类参数，并在移动、倾斜、远近距离及连续多帧条件下检查稳定性。

## 常见问题

- 浏览器无法连接：确认启动终端仍在运行，并检查 `curl --noproxy '*' http://127.0.0.1:8876/status`。
- 相机打开失败：关闭其他相机程序，检查 USB3、MVS 安装和设备权限。
- SSH 失败：确认电脑和 RDK 位于同一网段，并检查 `RDK_HOST`。
- ROS 节点找不到：重新执行 `source /opt/ros/humble/setup.bash` 和工作区的 `source install/setup.bash`。

## License

[MIT](LICENSE)
