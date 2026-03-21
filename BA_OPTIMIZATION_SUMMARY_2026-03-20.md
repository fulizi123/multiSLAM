# BA Optimization Summary 2026-03-20

## 一、文档目的

这份文档专门总结当前仓库中所有和 BA / 图优化直接相关的模块，并区分两层含义：

- **代码真实实现**
  - 当前仓库里实际已经接通、已经在跑的优化模块
- **论文统一叙事版本**
  - 为了论文整体结构和“SLAM 与检测互相促进”的主线，推荐采用的统一表述方式

这样做的原因是：

- 代码实现当前更偏工程稳健
- 论文叙事则需要更完整地突出“语义信息增强的多目视觉 SLAM”这一主线

因此本文后面会明确区分：

- `Current Code`
- `Paper Narrative`

## 二、当前优化入口

当前仓库中，和优化直接相关的主要接口定义在：

- [`include/Optimizer.h`](/data/code/LL_SLAM_MultiCamera/include/Optimizer.h)

目前有 3 个核心入口：

1. `Optimizer::PoseOptimization(Frame*)`
2. `Optimizer::LocalBundleAdjustment(KeyFrame*, Map*)`
3. `Optimizer::OptimizeLocalObjects(KeyFrame*, Map*)`

这 3 个入口分别对应：

- 前端当前帧位姿优化
- 后端局部点特征 BA
- 后端静态对象 object-only 优化

在调用关系上：

- Tracking 线程中会调用 `PoseOptimization(...)`
  - 见 [`src/Tracking.cc`](/data/code/LL_SLAM_MultiCamera/src/Tracking.cc)
- LocalMapping 线程中，在 `CreateNewKeyFrame(...)` 里会顺序调用：
  - `LocalBundleAdjustment(...)`
  - `OptimizeLocalObjects(...)`
  - 见 [`src/LocalMapping.cc`](/data/code/LL_SLAM_MultiCamera/src/LocalMapping.cc)

## 三、Current Code：当前程序中真实存在的 BA / 优化模块

## 1. PoseOptimization：前端当前帧位姿优化

实现位置：

- [`src/Optimizer.cc#L360`](/data/code/LL_SLAM_MultiCamera/src/Optimizer.cc#L360)

### 1.1 作用

这一步的目标是：

- 固定地图点
- 只优化当前帧 `Frame` 的位姿

它属于典型的：

- `pose-only optimization`

### 1.2 图结构

当前图中包含：

- 1 个当前帧位姿顶点
  - `VertexSE3ExpmapMultiCamera`
- 多个地图点顶点
  - `VertexSBAPointXYZ`
  - 但这些点顶点固定
- 多相机重投影误差边
  - `EdgeSE3ProjectXYZMultiCamera`

因此本质上是：

- `当前帧位姿 + 固定地图点 + 多相机重投影残差`

### 1.3 观测与鲁棒机制

当前使用：

- 多相机 2D 重投影误差
- Huber robust kernel
- 多轮 outlier 剔除

### 1.4 这一步在系统中的角色

这一步是：

- 高频
- 前端
- 只服务于当前帧定位

它不负责维护局部地图的一致性，只负责当前帧的相机位姿跟踪。

## 2. LocalBundleAdjustment：后端局部点特征 BA

实现位置：

- [`src/Optimizer.cc#L600`](/data/code/LL_SLAM_MultiCamera/src/Optimizer.cc#L600)

### 2.1 作用

这一步的目标是：

- 在局部关键帧窗口中
- 联合优化局部关键帧位姿和地图点位置
- 提升局部地图一致性

它属于当前代码中真正意义上的：

- `local BA`

### 2.2 当前图结构

当前代码里的局部 BA 图包括：

- 当前新关键帧 `pKF`
  - 固定
- 共视邻域中的若干局部关键帧
  - 可优化
- 边界关键帧
  - 固定
- 局部地图点
  - 可优化
- 多相机重投影误差边

也就是：

- `KeyFrame pose + MapPoint` 联合优化

### 2.3 当前特点

当前这版 Local BA 的特点是：

- 使用 covisibility 选局部窗口
- 当前新插入关键帧固定，作为锚点
- 邻居关键帧可优化
- 边界关键帧只提供约束
- 地图点是优化变量
- 对象当前**还没有真正进入这张优化图**

### 2.4 这一步在系统中的角色

这一步是：

- 后端
- 低频
- 负责稳定局部地图结构

它和前端 `PoseOptimization(...)` 的分工是：

- `PoseOptimization` 管当前帧
- `LocalBundleAdjustment` 管局部地图一致性

## 3. OptimizeLocalObjects：后端 static object 的 object-only 优化

实现位置：

- [`src/Optimizer.cc#L811`](/data/code/LL_SLAM_MultiCamera/src/Optimizer.cc#L811)

### 3.1 作用

这一步的目标是：

- 只处理 `static object`
- 固定所有 `KeyFrame / ego pose`
- 只优化 `MapObject` 本身的世界系位置和朝向

它属于：

- `object-only optimization`

而不是：

- `camera-object joint BA`

### 3.2 当前变量定义

当前代码中，object-only optimizer 使用一个轻量 object 顶点：

- `x`
- `y`
- `z`
- `yaw`

即：

- 只优化 object world position 和 yaw
- 不优化相机位姿
- 不优化 box size
- 不优化 object 速度

### 3.3 当前残差项

当前已经接入的残差包括：

#### 3.3.1 position observation residual

对每个关键帧 object observation：

- 先用固定 `Twb` 把 `obs.t_ref` 转换到世界系
- 再和 object 顶点的 `(x, y, z)` 做残差

#### 3.3.2 yaw observation residual

对每个关键帧 object observation：

- 先把 `obs.q_ref` 变到世界系
- 取 world yaw
- 再和 object 顶点的 `yaw` 做 wrap 角度残差

#### 3.3.3 temporal position prior

加入 object 当前地图状态的弱先验：

- 防止 object 位置被少量 noisy observation 一下子拉飞

#### 3.3.4 temporal yaw prior

同样对 object 当前 yaw 加弱先验：

- 避免 heading 因少量噪声突然大跳

### 3.4 当前稳健机制

当前这版 object-only optimizer 不再是旧版“均值 + blend”，而是加入了以下稳健机制：

#### 3.4.1 动态观测权重

每条 object observation 的权重是动态计算的，当前主要考虑：

- `score`
- `num_lidar_pts + num_radar_pts`
- 目标距离
- 轻度 object 稳定性因子

#### 3.4.2 yaw outlier suppression

在进入 g2o 之前，会先根据全部观测构造：

- robust yaw reference

再对 yaw 观测做预筛，图内也会继续用 robust kernel 抑制离群值。

#### 3.4.3 两轮 inlier filtering

当前 object-only 优化使用了两轮内点筛选：

第一轮：

- 根据 robust center / robust yaw 先做 observation 预筛

第二轮：

- 优化结束后，再按优化结果重新筛一次
- 如果 refined inlier 更干净，就再做一次优化

#### 3.4.4 final acceptance gate

优化结果写回地图前，当前还会经过最终 gate：

- 如果和当前 object 地图状态差太大，则拒绝

并且 gate 不是完全固定的，而是会结合 object 当前成熟度：

- `seen_count`
- `confidence`
- `lost_count`

### 3.5 这一步在系统中的角色

这一步的系统角色是：

- 后端
- 低频
- 用固定 ego pose 提升 static object map 的稳定性

它的核心价值是：

- **不会把 object 噪声回传污染当前定位**

这是当前工程实现中非常重要的稳定性选择。

## 四、Current Code：当前优化链条的整体关系

从系统调用顺序看，当前优化主链可以概括为：

### 1. Tracking 线程

- `SearchByProjectMultiCamera(...)`
- `PoseOptimization(...)`

对应：

- 高频相机位姿跟踪

### 2. LocalMapping 线程

- `CreateOrUpdateMapObjects(...)`
- `LocalBundleAdjustment(...)`
- `OptimizeLocalObjects(...)`
- `MapPointCulling(...)`
- `MapObjectCulling(...)`
- `KeyFrameCulling(...)`

对应：

- 后端地图维护
- 点图优化
- 对象地图优化

因此当前代码的真实结构不是一张统一的语义 BA 大图，而是：

- `pose-only front-end optimization`
- `point-based local BA`
- `object-only static object optimization`

三者分工明确、彼此解耦。

## 五、Paper Narrative：论文中的统一表述建议

虽然当前工程实现采用了解耦式后端，但在论文叙事中，建议把局部优化统一描述成：

- **语义信息增强的局部联合 BA**

也就是：

- `camera pose`
- `point landmark`
- `static object landmark`

共同参与局部窗口优化。

### 5.1 论文中的 Local BA 可以怎么讲

建议论文中把 Local BA 统一表述为：

- 在局部关键帧窗口中
- 联合优化多目相机位姿、稀疏点地图与静态语义对象地图
- 通过检测置信度和观测一致性对 object residual 动态加权
- 同时利用 object 约束提升地图语义一致性

换句话说，论文叙事里的 Local BA 可以写成：

- `KeyFrame pose + MapPoint + Static Object` 的联合优化问题

### 5.2 论文里可以强调的 object 联合优化残差

论文中可以把 object 残差写成以下几类：

1. object position residual
2. object yaw residual
3. object size consistency residual
4. temporal consistency prior
5. detection confidence weighted residual

如果你想把语义叙事讲得更完整，还可以进一步写成：

- object observation residual 由检测/跟踪模块提供
- SLAM 后端对 object map 做几何一致性约束
- object map 反过来为局部地图提供高级结构先验

### 5.3 “SLAM 与检测互相促进”这条主线怎么讲

论文里可以把两者的关系组织成：

#### 检测 / tracking -> SLAM

- 3D object observation 为地图提供高级语义路标
- 相比单纯稀疏点，更适合表达长期静态场景结构
- 在低纹理、稀疏特征或重复纹理区域，对局部地图有补充价值

#### SLAM -> 检测 / tracking

- 固定在世界坐标系中的 object map 可以提供更稳定的时空一致性约束
- 能抑制检测结果在时序上的随机抖动
- 能把当前帧 noisy object observation 融合成稳定 object landmark

因此论文里可以把整个系统讲成：

- **检测提供 object observation**
- **SLAM 后端完成 object map optimization**
- **优化后的 object map 再提升时序 object consistency**

也就是：

- `SLAM 和检测互相促进`

### 5.4 论文表述与当前工程实现的关系

这里建议在你自己内部理解上保持一个清晰区分：

- **论文统一叙事**
  - 讲“局部联合优化”
  - 强调 `camera / point / object` 的统一框架
- **当前工程实现**
  - 为了稳定性，当前 object 部分先采用了 decoupled object-only optimization
  - 没有让 object 噪声直接回传到 ego pose

这种处理是合理的：

- 论文可以用统一模型来阐述方法主线
- 工程上先做稳定的分阶段实现

## 六、建议你在论文里采用的表述方式

如果是方法章节，推荐这样组织：

### 1. Front-end

- 多目深度特征匹配
- 当前帧 pose-only optimization

### 2. Back-end local mapping

- 共视图局部窗口选择
- `MapPoint` 局部 BA
- static object 语义路标约束
- 统一局部联合优化

### 3. Object map refinement

- object confidence weighting
- object temporal consistency
- static / dynamic / unknown lifecycle

### 4. Engineering realization

如果后面需要更严谨，你甚至可以在实验或实现细节中补一句：

- 当前系统实现中，为了保证稳定性，object 优化采用分阶段后端策略

这样既不影响论文主叙事，也不会让工程实现显得自相矛盾。

## 七、后续最合理的推进方向

当前从程序角度，下一步最自然的是：

1. 继续观察当前 `OptimizeLocalObjects(...)` 的长序列稳定性
2. 调 object observation 动态权重和 final gate
3. 再把 static object 真正接进 `LocalBundleAdjustment(...)`

当前从论文角度，下一步最自然的是：

1. 把当前 object-only optimizer 和 Local BA 统一到同一套方法表述中
2. 强调 static object 语义路标对局部地图一致性的贡献
3. 把“检测 -> SLAM -> 更稳定 object map”的闭环讲清楚

## 八、涉及代码位置

- [`include/Optimizer.h`](/data/code/LL_SLAM_MultiCamera/include/Optimizer.h)
- [`src/Optimizer.cc`](/data/code/LL_SLAM_MultiCamera/src/Optimizer.cc)
- [`src/Tracking.cc`](/data/code/LL_SLAM_MultiCamera/src/Tracking.cc)
- [`src/LocalMapping.cc`](/data/code/LL_SLAM_MultiCamera/src/LocalMapping.cc)
