# 红色装甲识别与模式切换

日期：2026-11-07。红色适配完成后，用户现场初测反馈可用并要求发布至main。沿用`/home/sunrise/rdk_blue_vision`模块；包名和目录名保留蓝色命名，内部支持红/蓝/双颜色。

## 接入范围

- 红色灯条、配对、2D框和四角点。
- 红色二维跟踪、像素速度、100ms预测。
- 红色PnP的XYZ、姿态、R/Z、重投影误差。
- 红色三维跟踪与短时预测。
- 红蓝同框时按颜色匹配轨迹，避免两种颜色交换ID。
- ROS 2红色阈值参数，以及浏览器当前颜色标题/等待提示。

红色实拍已检出，用户反馈优化后的现场初测可用；精确倾角/近距范围、测距精度和动态效果仍需独立验证。蓝色历史测距记录单独保留。

## 启动与切换

以下电脑命令假设仓库克隆到`$HOME/RDK`；若目录不同，改为自己的路径。先结束旧相机/浏览器启动会话，使相机和转发端口可用，再启动需要的模式。

```bash
cd "$HOME/RDK"

# 红色，复用原开发相机的未核验调试配置
POSE_CALIBRATION=config/pose_camera.rdk-hikrobot-debug.yml \
  bash scripts/run-red-vision-browser.sh

# 蓝色（默认颜色仍为blue）
POSE_CALIBRATION=config/pose_camera.rdk-hikrobot-debug.yml \
  bash scripts/run-blue-vision-browser.sh

# 调试时同时检测两种颜色
ARMOR_COLOR=both POSE_CALIBRATION=config/pose_camera.rdk-hikrobot-debug.yml \
  bash scripts/run-blue-vision-browser.sh
```

一次只运行其中一个命令。浏览器为`http://127.0.0.1:8876/#debug`，标题分别显示红色、蓝色、红蓝装甲，`/status`的`mode`为`red`、`blue`或`both`。

如果电脑已有单独的SSH端口转发，可在RDK终端切换颜色；电脑转发继续复用：

```bash
cd /home/sunrise/rdk_blue_vision
ARMOR_COLOR=red POSE_CALIBRATION=config/pose_camera.rdk-hikrobot-debug.yml \
  bash scripts/run-rdk-ros2.sh
```

Ctrl+C结束后，把`ARMOR_COLOR`改为`blue`或`both`再启动。颜色在启动时选择，网页简洁/调试按钮仅切换显示内容。

## 参数和标定

`config/blue_vision_ros.yaml`仍为共用配置。红色HSV色相使用0–10及170–179两段，`red_gray_min=60`、`min_red_blue_ratio=1.05`；可通过`red_hue_low_max`、`red_hue_high_min`分别调整色相边界。蓝色原阈值保留。尺寸门槛当前共用：面积45、旋转框长边16px、短边3px、长短边比1.8，基础闭运算高度15。倾斜和近距离的共用改动见[几何优化记录](armor-geometry.md)。

本轮曝光3500us、增益2、30FPS起步。实际红灯成像若有过曝、断裂或漏检，应根据原始画面调整红色参数并复查角点，而不是用距离反调W/H。

同一相机和固定镜头不因切换灯光颜色而重新拍棋盘；使用同一几何尺寸的装甲时继续沿用K/D、W约130mm、H59mm。不同尺寸装甲需要对应W/H，当前共用一套几何配置，不把不同尺寸混入同一次测试。`calibration_verified=0`、`distance_validation_verified=0`，显式`allow_unverified_pnp=1`，浏览器继续显示未核验提示。

## 现场第一轮

把同一块装甲点亮红色，在约1.5–2米处完整入画，先保持静止无遮挡。检查标题为红色、出现框和四角点、调试区有R/Z/XYZ及重投影误差，随后记录检测连续性。确认静态画面后，再做缓慢左右移动、停止及短时遮挡。

用户已明确现有尺寸无法更精量，本轮使用现有近似尺寸开展初版测试，不把精量尺寸作为启动阻塞条件。真实红色检测和距离数据另建记录，不改写蓝色历史数据。

## 本轮检查结果

红色接入阶段RDK原生清理重编译后，六组CTest全部通过；后续几何优化新增回归，最新为七组全部通过。ROS管线测试含八种阶段：蓝色/红色分别无标定、已核验合成标定、显式未核验调试，以及红蓝同框调试、红色色相参数覆盖。测试覆盖红色角点、PnP、确认后的跟踪预测、红蓝轨迹关联隔离、浏览器、采集和断流清空。测试图为合成输入，不代表红色实际测距已验收。

浏览器脚本检查通过红/蓝/双模式标题、红色等待提示、PnP数值、未核验提示与丢失/断流清空。红色适配初次启动时20次状态均有画面、约30FPS；随后用户报告红灯可识别，但大角度和近距离漏检。本轮已继续优化公共几何，最新部署及测试结果见[几何优化记录](armor-geometry.md)，不能把初次启动记录当成全部场景验收。

红色接入阶段的构建与启动检查归档在原开发电脑`/home/zhuge/auto/rdk_camera_checks/20261107-red-adaptation/`。随后公共几何优化通过七组CTest；用户反馈现场初测可用。完整结果见几何优化记录。下述备份路径为原开发设备本地档案，不是克隆仓库后的必要文件。

## 修改前备份

电脑：`/home/zhuge/auto/rdk_backups/20261107-before-red/rdk-blue-before-red-20261107.tar.gz`。
RDK：`/home/sunrise/rdk_backups/rdk-blue-before-red-20261107.tar.gz`。
备份含修改前源码、配置、测试、安装目录和主要二进制；现有相机标定与标定照片继续保留。
