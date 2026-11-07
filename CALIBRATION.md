# 相机标定、灯条尺寸和PnP核验

适用于相机接RDK、电脑看浏览器的main流程。当前进度和四组数值见[蓝色装甲板进度](docs/blue-armor-progress.md)；摆放、不同距离和角度的拍照清单见[棋盘格标定拍照指南](docs/棋盘格标定拍照指南.md)。

## 1. 固定镜头并实测棋盘

当前棋盘9×6内角点（10×7方格），打印板横纵格子均实测为19mm。`config/chessboard-9x6-20mm.svg`是设计20mm的A4横向模板，100%打印后仍需实测，计算填实际边长。其他棋盘不能照搬19mm。

先确定焦距、对焦和光圈，再固定镜头。相机ROI和采样方式一致；不同对焦或变焦状态另分批次。轻微光圈调节不自动等同于内参失效，但应记录时点并检查成像。标定需要不同位置、距离和前后倾角，不要求每张都在2米，也不需要精确测每张拍摄距离。

## 2. 单独采集新一批原图

以下命令在电脑执行；需先按[README](README.md)部署：

```bash
cd "$HOME/RDK"
ARMOR_CAPTURE_DIR=calibration/session-new/captures \
EXPOSURE_US=60000 CAMERA_GAIN=16 CAMERA_FPS=15 \
  bash scripts/run-blue-vision-browser.sh
```

打开`http://127.0.0.1:8876/#debug`，在棋盘和相机停稳后点击“保存标定照片”。这是1440×1080原图PNG，无检测绘图；浏览器960×720截图不代替原图。60ms/增益16是当前室内静态棋盘亮度设置，应根据照明缩短曝光、避免拖影和过曝。日常蓝灯条检测恢复3500us、增益2、30FPS。

Git历史中保留的`calibration/`照片只是开发档案，不能当成本次新拍素材。使用独立`session-new`目录；新的一批换目录名。拍约20–30张、完整清晰、覆盖中央四角与倾角的照片，最终数量以合格筛选和姿态覆盖为准。

## 3. 筛选、隔离和计算

以下在RDK终端执行：

```bash
cd /home/sunrise/rdk_blue_vision
python3 scripts/review_calibration.py \
  --images calibration/session-new/captures \
  --report calibration/session-new/review-preview.json

# 查看报告后应用；三个图片目录相互独立
python3 scripts/review_calibration.py --apply \
  --images calibration/session-new/captures \
  --accepted calibration/session-new/accepted \
  --quarantine calibration/session-new/quarantine \
  --report calibration/session-new/review-applied.json

python3 scripts/calibrate_camera.py \
  --images calibration/session-new/accepted \
  --columns 9 --rows 6 --square-mm 19 \
  --output config/pose_camera.measured-session-new.yml
```

筛选默认要求完整54个内角点、估计棋盘边界完整、相同分辨率、对比度≥60、内角点凸包覆盖≥3%、横纵边缘过渡宽度≤6px。6px只是初版宽松筛选线，更清晰照片优先；可用`--max-edge-px 3`收紧。自动筛选不判断所有姿态覆盖，应检查分布。

`--apply`复制入选图到accepted，移动不合格原图到quarantine，保留原因和哈希。报告和目标同名文件不会覆盖。标定默认至少15张有效照片；输出K/D、真实分辨率、逐图误差和`.report.json`，保持`calibration_verified: 0`及零装甲尺寸。拟合RMS小不代表实际测距准确。

## 4. 尺寸对应检测角点

四角点取左右灯条长轴的上/下端点，顺序为左上、右上、右下、左下。

- W：两根有效发光灯条中心线之间的间距。
- H：单根有效发光灯条沿长轴的长度，左右分别确认。

量发光区域，不用外壳宽高代替。模型假定等长、平行、共面灯条，明显过曝会影响端点。当前初版W约130mm、H实测约58–60mm，运行取59mm；这是近似尺寸，不用已知距离反向调尺寸凑数。其他装甲应填自己的实测值，毫米除以1000写入米单位配置。

## 5. 先启用显式未核验调试

在RDK从自己的结果复制：

```bash
cp config/pose_camera.measured-session-new.yml config/pose_camera.yml
# 编辑pose_camera.yml：填写自己的armor_width_m、armor_height_m；
# 保留calibration_verified: 0，添加allow_unverified_pnp: 1。
```

结束旧会话后，在电脑启动：

```bash
POSE_CALIBRATION=config/pose_camera.yml bash scripts/run-blue-vision-browser.sh
```

只有内参、畸变、正数尺寸及分辨率合法，才会输出三维结果。`geometry.reason=debug_unverified`，验证标志仍为0，浏览器简洁模式也保留“PnP调试·距离未核验”。未显式允许调试时保持`calibration_unverified`，只输出二维结果。

同一原开发相机、镜头状态未变时，可直接在电脑选择当前设备快照：

```bash
POSE_CALIBRATION=config/pose_camera.rdk-hikrobot-debug.yml \
  bash scripts/run-blue-vision-browser.sh
```

快照来自16张筛选照片、19mm棋盘、1440×1080，RMS约0.189px，W130/H59。它是该设备的未核验候选，不用于其他相机或改变镜头状态后的设备。`pose_camera.example.yml`模板则保持调试禁用、尺寸为零。

## 6. 独立距离核验

固定相机、固定点亮的装甲，完整入画。卷尺一端在镜头前端，另一端在两根灯条之间、灯条高度中间的中心点，直接连接两端拉直。记录实际距离及参考偏差；镜头前端与光心的偏移尚未知。量到装甲侧面、地面投影或板平面垂距不能直接当作中心R参考。

相机光学系X向右、Y向下、Z向前。浏览器R=`sqrt(X²+Y²+Z²)`是直线距离，Z是前向深度，离轴时两者不同；目标偏轴或板有倾角本身不要求重新标定。画面或镜头运动后，应重新记录摆放参考；转动对焦/变焦环需要重新评估内参。

在电脑固定摆放、等待检测稳定后采集：

```bash
python3 scripts/collect_static_pnp.py \
  --output "$HOME/rdk-pnp-checks/session-2m" \
  --reference-m 2.00 --uncertainty-m 0.01 \
  --calibration config/pose_camera.rdk-hikrobot-debug.yml
```

`--calibration`用于保存电脑上的同一配置副本，应与RDK正在使用的配置一致，不会切换现场参数。只知道约数时用`--approximate-reference`替代`--uncertainty-m`，不编造±1cm。`--url`可覆盖浏览器地址，`--reference-origin`可记录不同端点。

采样默认10秒50次，输出CSV、完整状态JSON、汇总和可用的配置快照。它要求持续可读服务和单个装甲，但不自动判断物理静止；请在`summary.json`核对角点中心变化及原始状态，移动或遮挡的窗口单独留档，重新摆放的卷尺数值不能套到旧组。HTTP抽样不是逐帧检出率。

多距离、多位置对比均值、波动和名义误差；不单凭一处距离、低重投影误差或不变读数认定精度通过。当前1.5m/2.5m仍有约2cm名义负偏差，3m参考端点不同，故全距离核验未通过。[已保存记录](docs/measurements/2026-11-static/README.md)可复核。

## 7. 核验后及下一阶段

只有完成本机参数来源、尺寸和独立距离核验，才将自己的`calibration_verified`设为1；关闭调试许可并保存验收依据。当前发布快照继续保留0，已有结果不代表全场景验收。

现阶段用户无法进一步精量尺寸，先固定近似W/H进行动态跟踪测试。约2米左右缓慢平移30–50cm、停2秒、重复3次，先无遮挡，再测遮挡和预测；动态精度尚未实测。原始照片和本机生成文件自行备份，仓库保存通用流程、未核验快照和数值记录。
