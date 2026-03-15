# 2026-03-14 晚间进展与明日计划

## 一、范围说明

这份记录只总结 **GitHub 推送之后** 到现在新增完成的工作，不重复前面已经提交并推送过的 Local BA、covisibility、culling、`CreateNewKeyFrame` 优化等内容。

## 二、今晚新增完成的工作

### 1. 生成离线 GT 语义对象数据

目标：

- 不先用 StreamPETR / Stream tracking 结果
- 先直接用 CARLA 生成的 nuScenes 风格 GT 3D box
- 构建一套稳定的离线语义测试输入

完成内容：

- 新增了 GT 转换脚本：
  - `/data/code/carla_nuscenes/generate_gt_objecttrack_bins.py`
- 脚本可以把：
  - `/data/code/carla_nuscenes/dataset_nusc_0307_1704`
  下的 nuScenes 元数据和 GT 框
  转成：
  - `/data/code/carla/dataset_nusc_0307_1704_carla/ObjectTrackTruth/000000.bin`
  - `/data/code/carla/dataset_nusc_0307_1704_carla/ObjectTrackTruth/000001.bin`
  - ...
  - `/data/code/carla/dataset_nusc_0307_1704_carla/object_track_meta.json`

### 2. 统一把对象观测转换到车体坐标系

讨论结论：

- 直接存 `LIDAR_TOP` 坐标系不太适合后续语义 SLAM 接入
- 更适合存成 `ego/body frame`

已执行：

- 重新运行脚本，使用：

```bash
python3 /data/code/carla_nuscenes/generate_gt_objecttrack_bins.py --coordinate-frame ego
```

结果：

- 当前 `ObjectTrackTruth/*.bin` 已经是 **车体坐标系（ego/body frame）** 下的对象观测
- `object_track_meta.json` 中：
  - `coordinate_frame = ego`
  - `reference_channel = LIDAR_TOP`

### 3. 接通离线对象输入读取链路

目标：

- 在当前 SLAM 主程序中按帧读取 `ObjectTrackTruth/*.bin`
- 暂时不进优化，先保证输入链稳定

完成内容：

- 新增轻量数据结构：
  - [`include/ObjectObservation.h`](/data/code/LL_SLAM_MultiCamera/include/ObjectObservation.h)
- 在 [`Examples/stereo_carla_multi.cc`](/data/code/LL_SLAM_MultiCamera/Examples/stereo_carla_multi.cc) 中增加了：
  - `LoadObjectTrackPaths(...)`
  - `getObjectTrack(...)`
  - `LoadObjectTrackMultiThread(...)`
  - `ObjectTrackTruth` 队列
  - 与图像/特征同步打包进主循环

当前行为：

- 运行时会自动优先读取：
  - `SequencePath/ObjectTrackTruth/*.bin`
- 每帧会打印：
  - `ObjectTrack count : ...`
  - `static : ...`

### 4. 将对象观测一路传到 Frame / System / Tracking

目标：

- 不只在主程序中读到对象数据
- 而是把对象观测真正挂到当前帧数据结构上，给后面 viewer / map object / 优化留入口

完成内容：

- 修改了以下接口，把 `ObjectObservation` 从主程序一路传下去：
  - [`include/System.h`](/data/code/LL_SLAM_MultiCamera/include/System.h)
  - [`include/Tracking.h`](/data/code/LL_SLAM_MultiCamera/include/Tracking.h)
  - [`include/Frame.h`](/data/code/LL_SLAM_MultiCamera/include/Frame.h)
  - [`src/System.cc`](/data/code/LL_SLAM_MultiCamera/src/System.cc)
  - [`src/Tracking.cc`](/data/code/LL_SLAM_MultiCamera/src/Tracking.cc)
  - [`src/Frame.cc`](/data/code/LL_SLAM_MultiCamera/src/Frame.cc)

当前结果：

- `Frame` 现在已经持有：
  - `mvObjectObservations`

这意味着后面做 viewer 可视化、对象地图类、静态/动态分流、对象优化时，已经有了一条干净的数据入口。

### 5. Viewer 中增加对象观测面板

目标：

- 先在你当前 MP4 viewer 里把对象观测显示出来
- 不急着做复杂的 3D bbox 投影或对象地图层

完成内容：

- 在 [`src/Viewer.cc`](/data/code/LL_SLAM_MultiCamera/src/Viewer.cc) 中新增了一个轻量 BEV 对象面板：
  - 以当前帧 `ego` 为中心
  - 显示对象点
  - 静态/动态颜色区分
  - `track_id`
  - 对象总数 / 静态对象数

### 6. 定位到“为什么视频里看不到对象”

问题原因已经确认：

- 对象面板原本是先画在 `img_map` 上
- 但后面 8 路预览图再贴回画布时，把面板覆盖掉了

已修正：

- 把对象面板移动到 **预览图贴回之后** 再画
- 同时把位置改到画布中央区域，避免被周围 8 路预览图遮挡

注意：

- 这最后一步修改已经写进代码，但**还没有在今天结束前重新跑一次视频做最终肉眼确认**
- 代码层面修改已完成，明天第一件事就是重新跑短视频验证对象面板是否真的可见

## 三、当前代码状态

当前代码已经具备：

- 离线 GT 对象观测数据生成能力
- 统一的 `ego/body frame` 对象观测格式
- 主程序逐帧离线对象读入
- `Frame` 内对象观测存储
- `Viewer` 的对象面板可视化代码

但**还没有**做：

- 对象进入地图的 `MapObject` 类
- 静态/动态对象分层管理
- 对象在世界地图中的持久化
- 对象参与优化
- 图像侧 3D bbox 投影显示

## 四、明天建议优先做的事

### 1. 先验证 Viewer 中对象面板是否可见

这是明天最先要做的事情。

具体检查：

- MP4 里是否能看到对象面板
- 对象数量是否和日志一致
- 静态/动态颜色是否正确
- 面板是否会遮挡关键信息

如果 still 不明显：

- 再调整面板位置 / 尺寸 / 颜色 / 文本样式

### 2. 决定下一步是“图像侧可视化”还是“地图侧对象类”

推荐优先级：

- 先做 **图像侧可视化**
  - 在相机预览图上叠加对象 ID / 位置提示
  - 暂时不一定画完整 3D 框

然后再做：

- `MapObject` 类
  - 为静态车辆语义地图做准备

原因：

- 先把观测可视化做完整，更容易确认数据和坐标系完全没问题
- 再进地图层，风险更低

### 3. 进入 Phase 3：对象地图层设计

如果可视化验证通过，下一步建议直接做：

- `MapObject` 数据结构
- `Frame` 到 `MapObject` 的观测关联
- 静态 / 动态 / unknown 三态管理

### 4. 再之后才开始考虑对象优化

顺序建议：

1. 先读入
2. 再可视化
3. 再对象地图
4. 最后才进优化

这个节奏最稳。

## 五、今晚新增涉及的主要文件

仓库内新增/修改：

- [`include/ObjectObservation.h`](/data/code/LL_SLAM_MultiCamera/include/ObjectObservation.h)
- [`include/Frame.h`](/data/code/LL_SLAM_MultiCamera/include/Frame.h)
- [`include/System.h`](/data/code/LL_SLAM_MultiCamera/include/System.h)
- [`include/Tracking.h`](/data/code/LL_SLAM_MultiCamera/include/Tracking.h)
- [`src/Frame.cc`](/data/code/LL_SLAM_MultiCamera/src/Frame.cc)
- [`src/System.cc`](/data/code/LL_SLAM_MultiCamera/src/System.cc)
- [`src/Tracking.cc`](/data/code/LL_SLAM_MultiCamera/src/Tracking.cc)
- [`src/Viewer.cc`](/data/code/LL_SLAM_MultiCamera/src/Viewer.cc)
- [`Examples/stereo_carla_multi.cc`](/data/code/LL_SLAM_MultiCamera/Examples/stereo_carla_multi.cc)

仓库外新增：

- `/data/code/carla_nuscenes/generate_gt_objecttrack_bins.py`

生成数据：

- `/data/code/carla/dataset_nusc_0307_1704_carla/ObjectTrackTruth/*.bin`
- `/data/code/carla/dataset_nusc_0307_1704_carla/object_track_meta.json`
