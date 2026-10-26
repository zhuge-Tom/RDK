# 电脑直连海康摄像头：蓝色装甲板识别

这是 `pc-direct-hikrobot` 分支。海康 MV-CS016-10UC 通过 USB3 直接连接 Ubuntu 电脑，MVS SDK、C++ 检测算法和浏览器服务全部运行在电脑上。

本分支不需要 ROS 2，也没有远程采集或 SSH 转发。程序只输出检测可视化、二维装甲框和四个像素角点，不包含射击、云台或运动控制。

```mermaid
flowchart LR
  A[海康 MV-CS016-10UC] -->|USB3| B[电脑 MVS SDK]
  B --> C[电脑 C++ 蓝色装甲检测]
  C --> D[二维框与四角点]
  C --> E[本机 MJPEG 服务]
  E --> F[电脑浏览器]
```

## 检测算法

- HSV 蓝色分割与灰度亮度联合筛选
- 竖向闭运算，连接灯条中过曝发白造成的断裂区域
- 面积、尺寸、长宽比和蓝红通道比例过滤
- 灯条高度、纵向偏差、间距及装甲比例配对
- 装甲角点顺序为左上、右上、右下、左下
- 浏览器显示识别状态、角点坐标、检测 FPS 和显示 FPS

## 文件结构

```text
src/blue_armor_mvs_detector.cpp   MVS 取流和 C++ 蓝色装甲检测
scripts/mjpeg_browser.py          本机 MJPEG 浏览器服务
scripts/check-pc-environment.sh   编译前环境和相机检查
scripts/run-pc-direct-hikrobot.sh 检测、网页服务和浏览器启动入口
```

## 电脑要求

- x86_64 Ubuntu 22.04
- 海康机器人 Linux x86_64 MVS SDK，默认安装到 `/opt/MVS`
- 海康相机通过 USB3 直接连接电脑
- OpenCV 4 开发包
- GCC/G++、CMake、Python 3、curl 和 xdg-utils

安装开源依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev \
  python3 curl xdg-utils usbutils
```

MVS SDK 由海康提供，不包含在本仓库中。安装时请选择 Linux x86_64 版本。程序需要以下文件：

```text
/opt/MVS/include/MvCameraControl.h
/opt/MVS/lib/64/libMvCameraControl.so
```

该相机属于工业相机，通常不会显示为 `/dev/video0`；程序通过 MVS SDK 打开设备。

## 下载与检查

```bash
git clone --branch pc-direct-hikrobot https://github.com/zhuge-Tom/RDK.git
cd RDK

bash scripts/check-pc-environment.sh
```

检查脚本会确认电脑架构、编译工具、OpenCV、MVS 头文件、MVS 动态库和 USB 相机。全部显示 `OK` 后再编译。

如果 MVS 安装在其他目录：

```bash
MVS_ROOT=/path/to/MVS bash scripts/check-pc-environment.sh
```

## 编译

默认 MVS 路径：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

自定义 MVS 路径：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMVS_ROOT=/path/to/MVS
cmake --build build --parallel
```

编译完成后应生成：

```text
build/blue_armor_mvs_detector
```

## 启动识别和浏览器

确认海康 MVS 客户端及其他相机程序已经关闭，然后执行：

```bash
bash scripts/run-pc-direct-hikrobot.sh
```

脚本会依次完成：

1. 通过电脑上的 MVS SDK 独占打开 USB 相机。
2. 在电脑运行 C++ 蓝色灯条与装甲配对。
3. 在 `127.0.0.1:8876` 启动 MJPEG 服务。
4. 等待服务就绪并自动打开电脑浏览器。

浏览器地址：

```text
http://127.0.0.1:8876/
```

按 `Ctrl+C` 停止程序并释放相机。

## 调整相机和显示参数

```bash
EXPOSURE_US=3000 \
CAMERA_GAIN=2 \
CAMERA_FPS=30 \
PROCESSING_WIDTH=960 \
JPEG_QUALITY=75 \
BROWSER_PORT=8876 \
bash scripts/run-pc-direct-hikrobot.sh
```

默认值：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `EXPOSURE_US` | `3500` | 相机曝光时间，微秒 |
| `CAMERA_GAIN` | `2` | 相机增益 |
| `CAMERA_FPS` | `30` | 采集帧率 |
| `PROCESSING_WIDTH` | `960` | 检测图像宽度 |
| `JPEG_QUALITY` | `75` | 浏览器 JPEG 质量 |
| `BROWSER_PORT` | `8876` | 本机浏览器端口 |
| `MVS_ROOT` | `/opt/MVS` | MVS SDK 安装目录 |

灯条中心大面积发白时先降低曝光。每次只调整一类参数，并在移动、倾斜、远近距离和连续多帧条件下检查稳定性。

## 验证输出

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

画面中的绿色四边形为装甲区域，红点为四角点，紫色矩形为二维包围框。移动相机或装甲板时，画面、角点坐标和 FPS 应持续更新。

## 常见问题

- `MVS SDK not found`：确认安装的是 x86_64 Linux MVS，并检查 `MVS_ROOT`。
- `no Hikrobot USB camera found`：检查 USB3 线、相机供电、`lsusb` 输出和 MVS 驱动。
- `Device busy`：退出 MVS 客户端和其他占用相机的程序后重试。
- 浏览器没有自动打开：手动访问 `http://127.0.0.1:8876/`。
- 灯条发白或识别不稳定：降低 `EXPOSURE_US`，再调整增益和颜色阈值。

## License

[MIT](LICENSE)
