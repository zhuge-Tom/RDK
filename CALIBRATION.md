# 当前海康相机标定与灯条尺寸测量

相机始终接 RDK。电脑浏览器只负责查看和点击采集。
标定需要真实棋盘格照片和实物测量；尚未采集时不能声称已经标定完成。

## 1. 准备棋盘格和量具

本仓库提供 `config/chessboard-9x6-20mm.svg`，A4 横向，按 **100% 原始尺寸** 打印。
它有 **10 列 × 7 行方格，9 列 × 6 行内角点**，设计方格边长 20 mm。
也可以使用现有的平整棋盘格，命令填写它的实际内角点数量。

**当前这张打印板的横向、纵向方格边长均已实测为 19 mm（2026-10-27）。**
本次标定采用 **19 mm = 0.019 m**；脚本默认值和下面的命令均已按此设置。
SVG 文件名中的 20 mm 是原始设计尺寸，计算以实物测量值为准。

将纸平整贴在硬板上，保持清晰、无翘曲。用尺子测量实际印出的方格边长；
建议横纵各跨多个方格测量后除以格数。若打印发生不同比例的横纵缩放，应重新打印。
准备直尺/游标卡尺，测量目标灯条；另用卷尺做后续距离核验。

## 2. 保持相机配置，采集 20–30 张原始照片

固定当前镜头焦距、对焦和相机 ROI/采样设置。保持以后检测使用的同一成像配置。
增加环境照明，让黑白格在浏览器中清楚可见；若必须调曝光，保持无明显拖影和过曝。
蓝灯条检测默认的 3500 us 曝光未必足以拍清室内棋盘格。
当前室内使用曝光 **60000 us（60 ms）**、增益 **16**、帧率 **15 FPS**；
增益从 12 上调以增加亮度。此前在增益 12 的实时预览中抽帧成功找到了
9×6 共 54 个内角点；这只是角点可检测性检查，
不代表照片无拖影或已完成标定。此曝光下必须固定相机，棋盘停稳后再保存；
如果有拖影，应增加照明并缩短曝光。

需要重新启动这个亮度设置时，先结束当前相机会话，再在电脑执行：

```bash
cd ~/RDK
EXPOSURE_US=60000 CAMERA_GAIN=16 CAMERA_FPS=15 bash scripts/run-blue-vision-browser.sh
```

标定照片采集完毕后，普通检测入口恢复默认的 3500 us、增益 2、30 FPS。

1. 打开 `http://127.0.0.1:8876/`，切换到“调试模式”。
2. 让棋盘格完整进入画面，停稳，再点击“保存标定照片”。
3. 看到保存成功后移动标定板，再保存下一张。
4. 覆盖画面中央、四角和边缘，改变距离及左右/上下倾斜角度，保持所有内角点可见。
   不要只在一个位置连拍，也不要只拍与相机平行的棋盘格。

按钮保存的是 **原始分辨率、未缩放、无检测绘图的 PNG**，而不是浏览器截图。
默认目录：`/home/sunrise/rdk_blue_vision/calibration/captures/`。
实际原始分辨率以成功提示和照片为准；浏览器的 960×720 是检测显示分辨率。

每次标定使用独立的图片目录，不要把不同镜头、对焦、ROI、分辨率的照片混用。
启动时可用 `ARMOR_CAPTURE_DIR` 在 RDK 指定新目录。

点击保存时不要求画面已经检测到棋盘；照片是否能用于标定由后续角点检查决定。
若出现采集错误，当前网页会具体区分服务尚未发现、保存响应超时、
无新鲜原始帧或文件写入失败。超时后先检查已保存数量，避免重复点击。

## 3. 在 RDK 筛选照片并计算相机参数

以下命令均在 RDK 执行。新克隆的仓库不包含实拍照片，应先完成上一节采集。
先运行只生成报告的检查，查看每张照片的清晰度、覆盖率和拒收原因：

```bash
cd /home/sunrise/rdk_blue_vision
python3 scripts/review_calibration.py \
  --columns 9 --rows 6 --report calibration/review-preview.json
```

默认扫描 `calibration/captures/*.png`，此时不会移动照片。
检查报告后应用相同规则，复制合格照片到 accepted，移动不合格原图到 quarantine：

```bash
python3 scripts/review_calibration.py --apply \
  --columns 9 --rows 6 --report calibration/review-applied.json
```

默认要求完整 54 个内角点、估计的整块棋盘边界在画面内、统一分辨率、
黑白对比度至少 60、内角点凸包占画面至少 3%，横纵边缘过渡宽度均不超过 6 px。
6 px 是 PnP 初版的宽松筛选线，优先补拍更清晰的照片；可用 `--max-edge-px 3`
收紧筛选。自动筛选不判断姿态覆盖是否充分，应人工检查不同位置、距离和倾角。

报告已有同名文件、accepted 已有图片或隔离目录存在同名原图时，脚本会拒绝覆盖。
新批次用独立目录，不要把未复查照片直接添加到已选集。例如：

```bash
python3 scripts/review_calibration.py --apply \
  --images calibration/captures-next \
  --accepted calibration/accepted-next \
  --quarantine calibration/quarantine-next \
  --report calibration/review-next.json
python3 scripts/calibrate_camera.py \
  --images calibration/accepted-next \
  --columns 9 --rows 6 --square-mm 19 \
  --output config/pose_camera.measured-next.yml
```

该例要求先把新批次采集到 captures-next；启动时在电脑设置
`ARMOR_CAPTURE_DIR=calibration/captures-next`。
重新使用旧批次时，应准备其全部原图的副本，再用新目录重新筛选。

### 当前设备的开发记录

当前已复查原编号 1–37，按用户要求将不适合初版的原图隔离：

- 初版入选 **19 张**，独立目录 `calibration/accepted/`。
- 隔离 **18 张**（含早先移出的第 1 张），目录 `calibration/quarantine-20261027-200714/`。
- 入选清单及哈希在 `calibration/accepted/selection.json`，隔离原因在隔离目录的 `reasons.json`。
- 全部原图及本轮报告已另行备份到开发电脑；实拍照片不随源码仓库发布。

后续标定默认只读取 `calibration/accepted/`。浏览器新拍照片先进入 `calibration/captures/`，
必须复查后才能加入入选集，避免混入未检查素材。
上述筛选规则用于这次初版；更高精度仍需更清晰、覆盖充分的照片与实测核验。

已用这 19 张和实测 19 mm 计算 `config/pose_camera.measured.yml`。
拟合 RMS 为 **0.393 px**，逐张误差 **0.108–0.674 px**；报告是
`config/pose_camera.measured.report.json`。程序归一化不同 OpenCV 版本的角点形状，
并核对逐张误差与总体 RMS 一致。当前配置保持未验证，仍需实测灯条尺寸和独立距离核验。

### 用合格照片计算内参

在 RDK 终端执行（9×6 个内角点，当前打印板实测格子边长 19 mm）：

```bash
cd /home/sunrise/rdk_blue_vision
python3 scripts/calibrate_camera.py \
  --images calibration/accepted \
  --columns 9 --rows 6 --square-mm 19 \
  --output config/pose_camera.measured.yml
```

更换或重新打印棋盘后，重新实测并通过 `--square-mm` 填写实际边长。
`--columns`、`--rows` 数的是内角点。
本轮结果已经生成；重算时更换输出文件名以保存旧结果。补拍后先复查并更新入选集。
需要 RDK 已有的 Python OpenCV 与 NumPy，不要求电脑安装 MVS SDK。

程序会排除未找到完整棋盘格的照片，默认至少需要 15 张有效照片，
输出内参、畸变、真实原始分辨率和 `.report.json` 中的逐张重投影误差。
已有结果不会覆盖；重做时换一个输出文件名。
检查整体与逐张误差、照片清晰程度、姿态覆盖，以及去畸变后的直线效果。
可先把 RMS 小于约 0.5–1 px 当作初步检查目标，最终仍以独立距离核验为准。
较小的拟合误差不能证明参数一定准确。

## 4. 实测灯条几何尺寸

检测器的四角点位于左右灯条各自长轴的上下端点，按左上、右上、右下、左下排列。
因此需要测量：

- 宽 W：两根灯条**中心线之间**的实际间距。
- 高 H：单根有效发光灯条沿长轴的**上端到下端**长度；左右分别测量确认一致。

量发光区域，不量装甲外壳宽高，不量两灯条的外沿间距。
尺寸必须与图像实际拟合到的发光区域对应；明显过曝会改变这个对应关系。
本模型假定两灯条等长、平行、共面。不同尺寸装甲不能混用一套 W/H。

将毫米除以 1000 填入配置。例：仅当实测为 135 mm 和 56 mm 时，才填
`armor_width_m: 0.135`、`armor_height_m: 0.056`，不要直接套用例子。

## 5. 接入并核验三维定位

生成结果始终保留 `calibration_verified: 0`、尺寸为零。
确认相机参数来源、报告和实测灯条尺寸后：

```bash
cp config/pose_camera.measured.yml config/pose_camera.yml
# 编辑 pose_camera.yml：填实测 armor_width_m / armor_height_m，确认后将 calibration_verified 改为 1。
```

关闭旧启动会话，再在电脑启动：

```bash
cd ~/RDK
POSE_CALIBRATION=config/pose_camera.yml bash scripts/run-blue-vision-browser.sh
```

先固定相机、静止装甲，在多个已知距离和画面位置记录 XYZ 与误差。
测量基准应尽量对应相机光心到装甲中心；相机外壳基准带来的偏移需要记录。
注意 `Z` 是相机向前深度，面板“距离”是 `sqrt(X²+Y²+Z²)`，离轴时二者不同。
完成静态核验后再测试运动、短时遮挡和预测误差。

将本次原始照片、配置和误差报告另备份到电脑。源码仓库提供通用脚本和未验证示例；
实际照片与生成的本机标定文件由 `.gitignore` 排除，应自行保存并用于自己的相机。

依据：[OpenCV 相机标定教程](https://docs.opencv.org/4.5.4/d4/d94/tutorial_camera_calibration.html)、
[OpenCV 棋盘格角点与标定示例](https://docs.opencv.org/4.4.0/dc/dbb/tutorial_py_calibration.html)。
