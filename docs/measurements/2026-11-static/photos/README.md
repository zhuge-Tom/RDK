# 蓝色装甲板静态照片与逐张数值

2026-11-22整理。这里仅包含蓝色装甲板，未添加棋盘格照片。共13张：10张1440×1080原图和3张960×720采样后可视化截图。

## 逐张索引

点击照片编号查看照片、四角点与完整数值。`离线R/Z`是2026-11-22在RDK上用当前C++ ROS 2和本目录配置，对该张原图重新检测及解算的结果。截图未作为解算输入，相关字段留空。`关联组均值`仅为历史整组统计，不是该照片的同步读数；所有照片都未声称与某一次历史HTTP状态精确同步。

| 照片与详情 | 类型 | 离线R / m | 离线Z / m | 离线重投影 / px | 关联组均值R / m |
|---|---|---:|---:|---:|---:|
| [near-1m-diagnostic-01](records/near-1m-diagnostic-01.md) | 原图 | 1.017135 | 1.014452 | 1.733600 | — |
| [near-1m-diagnostic-02](records/near-1m-diagnostic-02.md) | 原图 | 1.017135 | 1.014452 | 1.733600 | — |
| [near-1m-diagnostic-03](records/near-1m-diagnostic-03.md) | 原图 | 1.017135 | 1.014452 | 1.733600 | — |
| [near-1m-diagnostic-04](records/near-1m-diagnostic-04.md) | 原图 | 1.017135 | 1.014452 | 1.733600 | — |
| [near-1m-diagnostic-05](records/near-1m-diagnostic-05.md) | 原图 | 1.017135 | 1.014452 | 1.733600 | — |
| [near-1p5m-diagnostic](records/near-1p5m-diagnostic.md) | 原图 | 1.481333 | 1.457639 | 0.737473 | 1.480138 |
| [near-3m-diagnostic-00](records/near-3m-diagnostic-00.md) | 原图 | 3.059278 | 3.041816 | 0.297086 | — |
| [near-3m-diagnostic-01](records/near-3m-diagnostic-01.md) | 原图 | 3.068029 | 3.036170 | 0.161540 | — |
| [near-3m-diagnostic-02](records/near-3m-diagnostic-02.md) | 原图 | 3.082956 | 3.042330 | 0.154672 | — |
| [2p5m-after-sampling-raw](records/2p5m-after-sampling-raw.md) | 原图 | 2.485245 | 2.483364 | 0.375849 | 2.478795 |
| [1m-after-sampling-view](records/1m-after-sampling-view.md) | 带框截图 | — | — | — | 1.001988 |
| [2m-after-sampling-view](records/2m-after-sampling-view.md) | 带框截图 | — | — | — | 1.993074 |
| [2p5m-after-sampling-view](records/2p5m-after-sampling-view.md) | 带框截图 | — | — | — | 2.478795 |

批量文件：[values.csv](values.csv)、[values.json](values.json)、[照片来源和校验值](catalog.json)。每张的独立JSON在`records/`，原照片在`images/`。原始文件逐字节复制，未裁切、改亮度或缩放。

## 对应关系与限制

- 5张约1米诊断原图属于早期静止排查，不是后来最终1.00m/H59采样组的原图；不能把最终组1.001988m均值套给它们。
- 1.5米原图属于像素门槛诊断；关联1.5米组只作场景背景，不断言是采样窗口内的某一帧。
- 3张约3米诊断原图拍摄过程中位置曾变化，不对应后续3.10m侧面参考组；不拿3.10m计算这些图的误差。
- 2.5米原图在50次采样结束后另拍；1m/2m/2.5m截图也在采样后读取。后采照片与该组均值可能不同。
- 原来每组50次HTTP读数并未保存50张同步原图，无法事后补成一一配对的原始数据集。正式组的全部读数仍在[上一级](../README.md)，原始数值没有被回放结果替换。
- 已被用户要求弃用的遮挡/移动试采、缩放合成图、二值掩码、红色照片未混入本目录。最终3m侧面参考组没有可确认对应的保留照片；只保留既有数值档案。

W约130mm、H取59mm，K/D为原固定镜头设备候选。`calibration_verified=0`、`distance_validation_verified=0`，回放仍为`debug_unverified`；这些输出不认证相机或测距精度。坐标为相机光学系X右/Y下/Z前，米；角点位于960×720处理图像，按左上、右上、右下、左下。

## 在RDK复算

先按仓库README构建、安装ROS 2项目。脚本使用`test_image`模式，默认隔离ROS域96，不打开真实相机或浏览器。该域应没有其他项目发布同名状态话题。

```bash
cd /home/sunrise/rdk_blue_vision
source /opt/ros/humble/setup.bash
source install/share/rdk_s100_blue_armor_detector/local_setup.bash
export LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:${LD_LIBRARY_PATH:-}
python3 scripts/replay_armor_photos.py \
  --manifest docs/measurements/2026-11-static/photos/catalog.json \
  --params-file docs/measurements/2026-11-static/photos/replay/blue_vision_ros.yaml \
  --calibration docs/measurements/2026-11-static/photos/replay/pose_camera.yml \
  --output "$HOME/armor-photo-replay-new"
```

输出目录必须不存在。照片SHA256在回放前核对；10张原图各生成一份JSON，截图跳过。源代码变化后复算可能不同，应保留新旧结果并对比版本。回放完整状态和核心源码SHA256见[replay/results.json](replay/results.json)，其中时间戳、检测耗时与轨迹属于本次回放运行，不是原始拍摄事件。
