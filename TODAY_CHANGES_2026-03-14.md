# 2026-03-14 代码改动记录

## 今日主要目标

今天的主要工作是把当前仓库从“多目跟踪 + 简化局部建图原型”，往更接近 ORB-SLAM2 的局部 SLAM 结构推进，重点放在以下几件事上：

- 补齐 `KeyFrame / MapPoint` 的观测关系管理
- 增加共视图 `covisibility graph`
- 接入 `Local Bundle Adjustment`
- 增加基础版 `MapPoint / KeyFrame culling`
- 优化 `CreateNewKeyFrame use time`
- 排查并缓解 `Local BA` 后位姿大跳变问题

## 当前分支

- 当前工作分支：`cx/add_3d_det`

## 1. 补齐 KeyFrame / MapPoint 双向观测关系

原始代码里虽然有 `KeyFrame` 和 `MapPoint`，但缺少比较完整的双向观测维护逻辑。

今天补进去的内容：

- `KeyFrame` 新增：
  - `AddObservation(...)`
  - `EraseObservation(...)`
  - `SetBadFlag()`
  - `isBad()`
- `MapPoint` 新增：
  - 总观测数统计
  - 不同关键帧观测数统计
  - `EraseObservation(...)`
  - `GetObservations()`
  - `SetBadFlag()`
  - `isBad()`
- 给 `MapPoint` 的 descriptor / pose 访问增加了 mutex 保护
- 删除点或删除关键帧时，会同步拆掉双方观测关系

涉及文件：

- `include/KeyFrame.h`
- `src/KeyFrame.cc`
- `include/MapPoint.h`
- `src/MapPoint.cc`

## 2. 增加共视图结构

原始实现里，局部地图选择基本还是“最近几帧”这种简单策略。今天增加了基础版共视图支持。

新增内容：

- `KeyFrame` 内增加共视连接存储：
  - 与其他关键帧的连接权重
  - 按权重排序后的共视关键帧列表
- 新增接口：
  - `AddConnection(...)`
  - `EraseConnection(...)`
  - `UpdateConnections()`
  - `GetConnectedKeyFrames()`
  - `GetBestCovisibilityKeyFrames(...)`
  - `GetWeight(...)`
- `Map::UpdateLocalMap(...)` 改成支持“按参考关键帧刷新局部地图”
- 局部地图优先由：
  - 参考关键帧
  - top 共视邻居
  - 不足时再回填最近关键帧
 组成

涉及文件：

- `include/KeyFrame.h`
- `src/KeyFrame.cc`
- `include/Map.h`
- `src/Map.cc`

## 3. 接入 Local Bundle Adjustment

原始仓库只有 `PoseOptimization(Frame*)`，本质是当前帧位姿优化，地图点固定，不属于真正的局部 BA。

今天增加了：

- `Optimizer::LocalBundleAdjustment(KeyFrame*, Map*)`

当前 `Local BA` 图中包含：

- 局部关键帧位姿顶点
- 边界关键帧位姿顶点
- 局部地图点顶点
- 多相机重投影误差边

实现上直接复用了现有的多相机重投影边，没有另起一套后端。

涉及文件：

- `include/Optimizer.h`
- `src/Optimizer.cc`

## 4. Local BA 窗口策略逐步调整

今天 `Local BA` 的窗口策略不是一次定好的，而是边跑边调出来的。

### 早期问题

一开始 Local BA 里把“刚插入的新关键帧 `pKF`”也一起优化了。  
这样会导致：

- 建图线程刚插完关键帧
- 立刻又把这个关键帧位姿强行拉到新位置
- 前端 tracking 下一帧接着跑时，就会表现成位姿大跳

### 当前整理后的策略

现在的 `Local BA` 窗口被整理成三类关键帧：

- 当前新插入关键帧 `pKF`
  - 固定，不参与优化
  - 作为当前局部 BA 的锚点
- 邻居关键帧
  - 来自共视邻居
  - 允许优化
- 边界关键帧
  - 固定
  - 只提供约束

这样做之后，`Local BA` 后的位姿大跳明显减轻。

涉及文件：

- `src/Optimizer.cc`

## 5. Tracking 和 LocalMapping 的状态同步修正

为了定位 BA 后位姿跳变，今天还查了 tracking 和建图线程之间的状态同步问题，并做了修正。

补的同步逻辑包括：

- `Local BA` 优化后，把 `KeyFrame` 的新位姿同步回它对应的 `Frame`
- 把 BA 引入的位姿修正传播到 tracking 当前还在使用的 `mpCurrentFrame / pPreFrame`
- `TrackWithMotionModel()` 改成优先使用真正的局部地图点：
  - `mpMap->GetLocalMapPoint()`
  - 不再只盯着 `mpReferenceKF` 自己那一小撮地图点

涉及文件：

- `src/Tracking.cc`
- `src/LocalMapping.cc`
- `src/Optimizer.cc`

## 6. 增加基础版 MapPoint / KeyFrame Culling

为了防止局部图和 BA 规模越跑越大，今天增加了基础版剔除逻辑。

### MapPointCulling

大致策略：

- 没有有效关键帧观测的点，删除
- 太老但关键帧观测数仍然太少的点，删除
- 成熟失败的弱点，删除

### KeyFrameCulling

大致策略：

- 如果某个局部关键帧中的大多数地图点，在其他局部关键帧里也已经有足够冗余观测
- 那么这个关键帧会被判为冗余关键帧并删除

### 注意

这版 culling 还是基础版，主要目标是先控制图规模和 Local BA 规模，不是最终版策略。

涉及文件：

- `include/LocalMapping.h`
- `src/LocalMapping.cc`
- `src/KeyFrame.cc`
- `src/MapPoint.cc`

## 7. 开发过程中定位并修掉的关键 bug

### 7.1 第一帧初始化卡死

原因：

- `MapPoint::AddObservation()` 持有 `mMutexFeatures`
- 内部又直接调用 `UpdateDescriptor()`
- `UpdateDescriptor()` 再次锁同一个 mutex

结果：

- 第一帧初始化时直接自锁卡死

修复：

- 把 descriptor 更新移到锁外执行

涉及文件：

- `src/MapPoint.cc`

### 7.2 Tracking 到 Map 的递归锁问题

原因：

- `Tracking::Track()` 外层先锁了 `mpMap->mMutexUpdate`
- 初始化/建图里又调用 `Map` 成员函数继续锁同一个 mutex

结果：

- 首帧或建图路径出现递归自锁

修复：

- 去掉 `Tracking::Track()` 外层那把 map 锁

涉及文件：

- `src/Tracking.cc`

### 7.3 descriptor 比较崩溃

原因：

- `FeatureMatcher::DescriptorDistance()` 里可能拿到空 descriptor 或尺寸不一致 descriptor
- 最终在 `cv::Mat::dot(...)` 里触发 OpenCV 断言

修复：

- 加了 descriptor 空值和尺寸检查
- 不合法时直接返回大距离

涉及文件：

- `src/FeatureMatcher.cc`

## 8. 关键帧插入频率实验

今天做了一个阶段性实验，把关键帧插入频率从：

- 每 `30` 帧插一次

改成了：

- 每 `10` 帧插一次

目的是先观察：

- Local BA 约束是否更合理
- 共视关系是否更稳定
- 建图线程会不会被压垮

实验结论：

- 更高频率插关键帧确实让局部约束变密
- 但也让 `CreateNewKeyFrame` 更容易成为瓶颈
- 所以下一步更合理的是改成“条件触发插关键帧”，而不是长期固定 `10` 帧一次

涉及文件：

- `src/Tracking.cc`

## 9. 优化 CreateNewKeyFrame 的主要耗时

今天专门针对 `CreateNewKeyFrame use time` 做了优化。

### 主要瓶颈定位

日志显示最重的一块是：

- `SearchByEpipolarGeometryBetweenImage(...)`

原始逻辑里，它会：

- 对很多局部关键帧都做
- 对全部 `12` 个相机都做

对多目系统来说，这个开销非常大。

### 当前优化思路

现在把它裁成：

- 少量共视关键帧
- 少量代表性相机

#### 关键帧选择

- 只保留少量局部关键帧参与极线匹配

#### 相机选择

不是简单写死相机编号，而是做了一个基础版动态选择：

- 先按方位分组
  - front
  - right
  - back
  - left
- 再在每组里挑当前特征支持最强的相机
- 最终大致保留 `4` 个代表性相机

日志里可以看到类似：

- `Epipolar selected KFs : 1 selected cams : 0 3 6 10`
- `Epipolar selected KFs : 2 selected cams : 1 4 6 9`
- `Epipolar selected KFs : 2 selected cams : 1 3 6 9`

### 实际效果

优化前：

- `SearchByEpipolarGeometryBetweenImage use time` 经常是几百毫秒
- `CreateNewKeyFrame use time` 常见 `0.8s ~ 1.0s+`

优化后：

- `SearchByEpipolarGeometryBetweenImage use time` 下降到大约 `40ms ~ 120ms`
- `CreateNewKeyFrame use time` 明显下降到大约：
  - `148ms`
  - `250ms`
  - `310ms`
  - `465ms`

说明这块优化是有效的。

涉及文件：

- `src/LocalMapping.cc`

## 10. 今天最终跑出来的主要结论

到今天结束时，系统的状态可以概括为：

- 仓库可以稳定编译
- 程序可以完整跑完 `0-99` 帧
- Local BA 已经接上并真正执行
- covisibility graph 已经有基础版支持
- KeyFrame / MapPoint 观测关系已经完整很多
- 基础 culling 已经接入
- `CreateNewKeyFrame` 主要耗时已经明显下降
- BA 后位姿大跳的问题已经明显缓解

但仍然还有后续工作要做：

- 关键帧插入还没有改成条件触发
- `CreateNewKeyFrame` 仍然不算轻
- culling 还只是第一版
- 轨迹仍有慢性漂移和中等幅度修正

## 11. 建议的下一步

比较推荐继续做这几项：

1. 把关键帧插入从固定 `10` 帧改成条件触发
2. 继续收紧 `CreateNewKeyFrame` 的计算量
3. 继续优化 culling 阈值
4. 如果后面要加语义对象，把当前这条局部建图 + Local BA 主干先彻底稳定住

## 今日改动过的主要源码文件

- `include/KeyFrame.h`
- `include/LocalMapping.h`
- `include/Map.h`
- `include/MapPoint.h`
- `include/Optimizer.h`
- `src/FeatureMatcher.cc`
- `src/KeyFrame.cc`
- `src/LocalMapping.cc`
- `src/Map.cc`
- `src/MapPoint.cc`
- `src/Optimizer.cc`
- `src/Tracking.cc`
