# 2026-03-20 对象生命周期管理与可视化修正记录

## 一、范围说明

今天这轮工作，重点不在继续推进 `Scene Graph / VLM`，而是先把当前仓库中的对象地图层补完整，具体范围包括：

- 把 `MapObject` 从“几何缓存”升级成“可维护的地图实体”
- 在 `LocalMapping` 中增加对象生命周期管理与 `MapObjectCulling`
- 正式引入 `static / dynamic / unknown` 三态对象管理
- 修正由本轮改动带来的 mp4 可视化问题

这次工作主要围绕以下文件展开：

- `include/MapObject.h`
- `src/MapObject.cc`
- `include/LocalMapping.h`
- `src/LocalMapping.cc`
- `src/Viewer.cc`

## 二、工作前的基线状态

在开始做对象生命周期之前，当前 GT 对象链路已经完成一轮稳定性验证：

- 1200 帧长序列可以稳定跑通
- 当前对象输入链路工作正常
- mp4 基线可视化原本是正常的

因此今天的主要目标，不是重新处理输入，而是在此基础上继续补对象地图层和生命周期逻辑。

## 三、今天完成的主要改动

## 1. 给 `MapObject` 增加生命周期字段与状态机

之前的 `MapObject` 主要保存：

- `track_id`
- `class_id`
- `score`
- `is_static`
- 世界系位置、姿态、速度、尺寸

它更像“对象几何缓存”，还不是完整地图实体。

今天补进去的内容包括：

- `ObjectState`
  - `OBJECT_STATE_STATIC`
  - `OBJECT_STATE_DYNAMIC`
  - `OBJECT_STATE_UNKNOWN`
- `SourceType`
  - `OBJECT_SOURCE_UNKNOWN`
  - `OBJECT_SOURCE_OBJECT_BIN`
  - `OBJECT_SOURCE_STREAMPETR`
- 生命周期统计字段
  - `mFirstObservedTime`
  - `mLastObservedTime`
  - `mnFirstObservedKFId`
  - `mnLastObservedKFId`
  - `mnLastLifecycleKFId`
  - `mnSeenCount`
  - `mnLostCount`
  - `mnAge`
  - `mnStaticObservationCount`
  - `mnDynamicObservationCount`
  - `mfConfidence`

同时新增接口：

- `MarkMissed(...)`
- `GetConfidence()`
- `GetSeenCount()`
- `GetLostCount()`
- `GetAge()`
- `GetStaticObservationCount()`
- `GetDynamicObservationCount()`
- `GetFirstObservedKFId()`
- `GetLastObservedKFId()`
- `GetState()`
- `GetSourceType()`
- `IsDynamic()`
- `IsUnknown()`

状态更新逻辑上，当前采用的是保守版投票规则：

- 只见过静态观测：`static`
- 只见过动态观测：`dynamic`
- 同一个对象既出现过静态观测、也出现过动态观测：`unknown`

这样可以覆盖当前最关心的情况：

- 某些车第一次被看成静态
- 后续又开始移动
- 这类对象后续不再当纯静态对象处理，而是归入 `unknown`

## 2. 在 `LocalMapping` 里补上对象生命周期管理

今天把 `CreateOrUpdateMapObjects(...)` 从“按 `track_id` 直接更新对象几何”扩成了“按关键帧驱动的对象生命周期更新”。

新增逻辑包括：

- 当前关键帧对象按 `track_id` 去重
- 新对象创建时写入时间戳、关键帧 id、来源类型
- 已有对象更新时同步刷新：
  - `seen_count`
  - `lost_count`
  - `age`
  - `confidence`
  - 静态/动态观测计数
  - 三态对象状态
- 对当前关键帧没有再次观测到的对象：
  - 调用 `MarkMissed(...)`
  - 累计对象丢失计数

同时，为了观察当前地图对象状态，今天把 `MapObjectUpdateStats` 也扩展成支持输出：

- `current_static_obs`
- `current_dynamic_obs`
- `new_mapobject`
- `updated_mapobject`
- `global_static_mapobject`
- `global_dynamic_mapobject`
- `global_unknown_mapobject`

## 3. 增加 `MapObjectCulling`

之前仓库里已经有：

- `MapPointCulling`
- `KeyFrameCulling`

但没有对象级剔除。

今天新增了：

- `LocalMapping::MapObjectCulling(...)`

当前采用的是第一版偏保守的对象剔除规则，主要目标是控制对象地图规模和坏对象积累，而不是一步做成最终版。

当前 culling 规则包括：

- 观测太少且年龄已经增长的对象，删除
- 动态对象长期未再观测到，删除
- 静态或 `unknown` 对象长期未再观测到，删除
- 年龄增长后仍然很弱、置信度很低的对象，删除
- 对于几何位置、尺寸、朝向都极其接近的非动态对象：
  - 只对“弱且新”的疑似重复对象做保守删除

这里特意没有做成激进 merge，原因是：

- 同类静态车在停车场/路边本来就可能彼此很近
- 如果 duplicate 判定过猛，会直接误删真实目标

所以当前 duplicate 处理的策略是：

- 先做极严格的几何相似度筛选
- 再要求被删对象本身“弱、年轻、或已经掉观测”
- 只有这样才执行删除

## 4. 正式引入三态对象可视化

今天把对象颜色从原本的二态：

- 静态
- 动态

扩展成三态：

- `static`：绿色
- `dynamic`：橙蓝色系
- `unknown`：粉色

实现上不仅改了颜色，还改了可视化的状态解析逻辑。

新增内容包括：

- 通过 `track_id` 主动回查地图中的 `MapObject`
- 当前帧 3D 框和 BEV 中的对象颜色优先跟随地图对象状态
- `unknown` 对象的相机预览框和 BEV 框都可以显示成粉色

## 四、今天新增后暴露出来的可视化 bug

今天这轮改动并不是一次就稳定了。对象生命周期和三态显示加进去后，最先暴露出来的是 mp4 中两个明显问题：

### 1. 动态车辆在粉色和橙色之间来回跳

根因定位结果：

- 当前帧 Viewer 原本优先从 `pCurrentFrame->mvMapObjects` 取对象状态
- 但普通 `Frame` 并没有真正维护好这条映射
- 于是很多对象在当前帧显示时会退回到原始 `obj.is_static`
- 导致已经进入 `unknown` 的对象，在可视化层又被按动态颜色画出来

结果表现为：

- 同一辆车在粉色和橙色之间来回切

### 2. 地图整体抽动，所有绿色静态车一起抖

根因定位结果：

- 这轮修改一度把 BEV 中“当前帧对象观测”也画了出来
- 但地图层本来已经在画静态 `MapObject`
- 对于静态对象，结果变成：
  - 地图层画一份
  - 当前观测层又画一份

这两层对象位置并不完全重合，于是表现成：

- 所有静态车一起抽动
- 整张对象地图像在轻微抖动

## 五、今天针对 bug 做的修正

针对上述问题，今天又做了一轮可视化修复：

### 1. 引入 `ResolveTrackedMapObject(...)`

Viewer 现在不再只依赖：

- `pCurrentFrame->mvMapObjects`

而是会在必要时按：

- `track_id -> MapObject`

主动回查地图对象。

这样做之后：

- 当前帧对象颜色可以真正继承地图状态
- `unknown` 对象不会再因为当前帧 fallback 到 `obj.is_static` 而跳回橙色

### 2. 恢复 BEV 分层规则

最终整理后的规则是：

- 地图层：
  - 只画真正的 `static map object`
- 当前帧观测层：
  - 只画 `dynamic`
  - 以及 `unknown`
- 不再把 `static` 当前观测重复叠加到地图层上

这样做之后：

- 绿色静态车不会再双层叠加
- 整张地图的抽动问题消失

### 3. 对 reviewer 子 agent 提出的 3 个问题做了修正

为了避免今天这轮状态机逻辑带来潜在隐患，专门拉了 reviewer 子 agent 做检查。它指出的 3 个关键问题分别是：

#### 3.1 `lost_count` 会重复累加

问题：

- `MarkMissed(...)` 一开始按 `current_keyframe_id - last_observed_id` 直接累加
- 这样对象连续缺失多个关键帧时，会把同一个 gap 重复加进去

修正：

- 新增 `mnLastLifecycleKFId`
- `lost_count` 现在按生命周期推进只累计一次

#### 3.2 同一帧重复 `track_id` 会污染状态统计

问题：

- 如果同一帧里出现重复 `track_id`
- 生命周期统计会被重复更新
- 可能把对象错误推成 `unknown`

修正：

- 在 `CreateOrUpdateMapObjects(...)` 中对当前关键帧的 `track_id` 做去重
- 同一帧重复 `track_id` 直接跳过

#### 3.3 duplicate culling 太激进

问题：

- 第一版 duplicate 逻辑对近邻同类对象过于敏感
- 有误删真实静态车的风险

修正：

- duplicate 清理改成保守策略
- 只有当对象“弱、年轻、置信度低、或已经掉观测”时，才允许作为可疑重复对象删除

## 六、验证结果

今天这轮改动完成后，已经做了以下验证：

- `LL_SLAM_MultiCamera` 重新编译通过
- `stereo_carla_multi` 重新编译通过
- 做了短时真实运行 sanity check
- 用户已经重新检查 mp4，确认：
  - 动态车不再在粉色和橙色之间来回跳
  - 地图整体抽动问题已经没有明显问题

也就是说，到今天结束时，这一轮对象生命周期和三态显示改动，已经从：

- “功能接入但带来明显可视化回归”

修到了：

- “生命周期逻辑已接通，可视化也已恢复稳定”

## 七、继续推进对象后端：把 `OptimizeLocalObjects(...)` 升级成 g2o 版 object-only optimizer

在对象生命周期与可视化稳定下来之后，今天又继续推进了对象后端优化本身。

在此之前，仓库中的 `OptimizeLocalObjects(...)` 本质上还是：

- 多关键帧世界系对象观测的简单平均
- 对 yaw 做 circular mean
- 再做 jump gate
- 最后做固定比例 blend

这一版虽然已经满足：

- 固定 `ego / KeyFrame pose`
- 只优化 `MapObject`
- 不把对象噪声回传到定位

但它更接近“工程平滑器”，还不是更稳健的对象优化器。

因此今天把它升级成了：

- **g2o 版 object-only optimizer**

并且整体风格保持和仓库当前的 `PoseOptimization / LocalBundleAdjustment` 一致：

- 仍然放在 `src/Optimizer.cc`
- 仍然使用 g2o
- 不改 `LocalBundleAdjustment(...)` 的主图
- 只在 `OptimizeLocalObjects(...)` 内部新建一个轻量 object-only 图

### 1. 当前 g2o object optimizer 的变量定义

今天新加的对象顶点是一个轻量的 `VertexObjectPose`，状态维度为：

- `x`
- `y`
- `z`
- `yaw`

也就是：

- 只优化世界系位置和朝向
- 不优化 `ego pose`
- 不优化 `size`
- 不优化速度

这样做的原因是：

- 风险低
- 不影响当前定位精度
- 和现有 `MapObject` 数据结构兼容

### 2. 当前加入的残差项

今天 object-only 图中加入了以下残差：

#### 2.1 位置观测残差

对每个关键帧对象观测，先利用固定 `Twb` 转到世界系：

- `world position observation`

然后对 object 顶点的位置做残差约束。

#### 2.2 yaw 观测残差

同样先把关键帧中的对象朝向变到世界系：

- `world yaw observation`

然后对 object 顶点的 `yaw` 做 wrap 过的角度残差。

#### 2.3 temporal position prior

为了避免 object 在观测较少时被少量 noisy observation 拉飞，加入了一个弱的时间一致性先验：

- object 新状态不要一下子偏离当前地图状态太多

#### 2.4 temporal yaw prior

同样对 `yaw` 加了弱的时间一致性先验。

这一步相当于把旧版“固定比例 blend”的思想，改成了更明确的优化图 prior edge。

### 3. 当前加入的 4 类稳健机制

这次 object optimizer 里，一起补了 4 件对实际效果更关键的东西：

#### 3.1 观测权重改成动态计算

现在的每条 object observation，不再等权。

当前动态权重主要考虑：

- `score`
- `num_lidar_pts + num_radar_pts`
- 目标距离
- 轻度的 object 稳定性权重

并且这次特意做了调整：

- 不再把 `lost_count` 直接统一乘到所有观测上
- 避免一个 stale object 把当前最新好观测也一起压低

#### 3.2 yaw outlier suppression

这次对 yaw 做了两层抑制：

- 先根据全部观测求一个 robust yaw reference
- 再对 yaw 观测做预筛
- 在 g2o 图中，yaw edge 也挂了 Huber kernel

这样做是为了避免少量朝向异常观测把 object heading 拉偏。

#### 3.3 两轮 inlier filtering

这次不是直接拿所有 observation 进图，而是做了两轮内点筛选：

第一轮：

- 根据 robust center / robust yaw 先做预筛

第二轮：

- 优化完成后，再按优化结果重新筛一次更干净的 inlier 集合
- 如果 refined inlier 比第一轮更收敛，就再做一次优化

#### 3.4 用 temporal prior 替代旧版固定 blend

旧版 object 优化最后是：

- `jump gate`
- `固定比例 blend`

今天改成：

- object-only 图中加入弱的 temporal prior edge

并且这次又根据 reviewer 反馈做了修正：

- temporal prior 的强度会随 inlier 数量增多而衰减
- 避免在观测较多时，prior 反过来过强地把 object 锁死在旧位置

### 4. 最终 acceptance gate 也做了加强

这次 object-only 图虽然已经更稳健，但最后一步仍然保留了 object 地图更新前的 safety gate。

并且相比之前只用固定阈值：

- `2.0 m`
- `35 deg`

今天改成了会结合 object 当前成熟度来动态调整 gate：

- `seen_count`
- `confidence`
- `lost_count`

这样做之后：

- 稳定老对象的接受阈值更严格
- 新对象或 recently unstable 对象相对更宽松

### 5. 对 reviewer 子 agent 新指出的问题做了修正

在 g2o 版 object optimizer 接上之后，又专门拉了 reviewer 子 agent 检查当前建模。

它指出的主要问题包括：

#### 5.1 temporal prior 过强

问题：

- 一开始 temporal prior 只和 `averageWeight` 成比例
- 在 observation 数少时，容易把 object 锁得太死

修正：

- temporal prior 强度现在会按 inlier 数量归一化衰减

#### 5.2 观测权重可能过度放大 support 点数

问题：

- support point 权重一开始偏大
- 可能让低分观测因为点数多而反超高分观测

修正：

- support 权重缩弱
- score 权重抬高
- 更偏向单观测质量本身

#### 5.3 yaw 二次筛选过紧

问题：

- 第二轮 refined yaw gate 一开始比第一轮明显更紧
- 容易把合法 observation 也过早筛掉

修正：

- refined yaw gate 改成不再明显收紧
- 保持和第一轮 gate 同量级

#### 5.4 final acceptance gate 没看 object 成熟度

问题：

- 最后一层更新 gate 原本仍然是全局固定阈值

修正：

- final acceptance gate 现在会结合：
  - `seen_count`
  - `confidence`
  - `lost_count`

#### 5.5 数值 Jacobian 开销

问题：

- 一开始用的是 unary edge 默认数值差分 Jacobian
- object 数量上来后有额外开销风险

修正：

- 位置 edge 和 yaw edge 都改成了解析 Jacobian

## 八、验证结果补充

除了前面生命周期与可视化层面的验证，这次 object optimizer 升级后又补做了以下检查：

- `Optimizer.cc` 改动后重新编译通过
- `LL_SLAM_MultiCamera` 重新编译通过
- `stereo_carla_multi` 重新编译通过
- 做了短时真实运行 sanity check
- 程序能够正常跑过多个 `CreateNewKeyFrame`
- `OptimizeLocalObjects count` 在后续关键帧上正常触发
- 没有出现新的 runtime 崩溃

当前可以认为：

- g2o 版 object-only optimizer 已经接通
- 当前对象后端已经不再只是“均值 + blend”

但也要明确：

- 这还是 object-only optimization
- 还没有把 static object 接进 `LocalBundleAdjustment(...)`
- 也还没有让 object 约束前端 pose

## 九、当前阶段结论

今天这轮工作的阶段性结果可以概括为：

- `MapObject` 已经不再只是简单几何缓存
- 对象地图层已经具备基础生命周期管理能力
- `static / dynamic / unknown` 三态已经正式进入代码主链
- `MapObjectCulling` 已经接入
- `unknown` 对象的粉色可视化已经接通
- 本轮引入的可视化 bug 已定位并修正
- `OptimizeLocalObjects(...)` 已经升级成 g2o 版 object-only optimizer
- object observation 权重、yaw 抑制、两轮 inlier filtering、temporal prior 都已经接通

但也要明确：

- 这一步还没有开始改 `camera + object` 联合优化图
- 当前对象管理还是“类无关、状态优先”的通用策略
- 还没有按不同语义类别（例如车、人、锥桶等）做差异化生命周期策略

## 十、下一步建议

比较合理的下一步是继续推进对象后端，而不是再回头改输入或高层语义。

建议优先级：

1. 继续观察当前 g2o object-only optimizer 的长序列效果和稳定性
2. 如果需要，继续调 object observation 的动态权重和 final acceptance gate
3. 再之后，把 static object 正式接进 `LocalBundleAdjustment(...)`
4. 如果后面有明确的 `class_id -> semantic class` 定义，再补按类别差异化管理策略

## 十一、今日改动涉及的主要文件

- `include/MapObject.h`
- `src/MapObject.cc`
- `include/LocalMapping.h`
- `src/LocalMapping.cc`
- `src/Viewer.cc`
- `src/Optimizer.cc`
