# 2026-03-21 语义场景图与 VLM 脑暴方案记录

## 一、文档目的

这份文档用于记录当前关于“语义场景图 + VLM”部分的阶段性方案脑暴结果，主要服务于两个目的：

- 为后续代码实现提供一个清晰的高层路线图
- 为论文中“高层语义认知 / Semantic Scene Graph / VLM-enhanced mapping”部分提供可用的叙事框架

当前这份文档不是实现记录，而是：

- 方案目的
- 整体思路
- 技术路径
- 预期效果

的整理稿。

## 二、当前判断：VLM 最适合做什么

当前讨论后的核心共识是：

- **VLM 最适合承担高层场景语义推理的角色**
- **不适合直接参与几何建图和位姿优化**

更具体地说，VLM 在当前系统里最合适的用途是：

- 判断当前是否处于路口区域
- 判断当前是否进入新的道路语义区域
- 辅助决定是否新建一个拓扑节点
- 辅助推断道路之间的连通关系类型
- 给当前道路 / 路口 / 区域打高层语义标签

而不建议让 VLM 直接做：

- `ego pose` 更新
- 几何坐标修正
- object 几何优化
- per-frame 高频低层推理

一句话概括就是：

- **VLM 是 topological semantic reasoner，不是 geometry source**

## 三、总体目标

在当前多目 SLAM + object map 的基础上，后续要构建的是一个：

- **以道路拓扑为骨架**
- **以静态物体为语义锚点**
- **以动态轨迹为时态补充层**
- **由 VLM 提供高层场景语义**

的分层语义场景图。

最终希望形成的不是单纯 object graph，而是：

- **road-centric hierarchical scene graph**

## 四、总体思路

## 1. 以道路连通结构为主骨架

目前最容易实现、也最适合论文讲故事的方式，不是直接以 object 为中心建图，而是：

- 先做 **道路拓扑图**
- 再把静态物体和动态轨迹挂到这张图上

也就是说，scene graph 的第一层不应该是：

- `vehicle / object`

而应该是：

- `RoadSegment`
- `Intersection`

你之前提出的想法：

- 对每条道路按方向建立图
- 在相邻道路之间建立连通性

这个方向是正确的，而且是最适合做第一版的。

## 2. 先做 road-centric graph，再挂 object 和 VLM

推荐的系统演化顺序是：

1. 先建立道路分向拓扑图
2. 把 static object 挂到对应道路 / 路口节点上
3. 动态车作为独立时态层叠加上去
4. VLM 只对稳定节点补高层语义

这样做的优点是：

- 图结构清晰
- 工程实现简单
- 不会把动态目标直接污染静态地图
- VLM 也不需要碰几何优化主链

## 五、推荐的分层场景图结构

建议把整个 scene graph 分成 4 层。

## 1. Geometry Layer

这是底层几何层，由当前 SLAM 系统直接维护，包括：

- `KeyFrame`
- `Pose graph`
- `MapPoint`
- `Static MapObject`

这层的作用是：

- 提供稳定的世界坐标基准
- 提供 scene graph 其余层的几何支撑

这层本身不需要交给 VLM。

## 2. Topology Layer

这是整个 scene graph 的主骨架，建议第一版只包含：

- `RoadSegment`
- `Intersection`

其中：

- `RoadSegment`
  - 有方向
  - 表示一段连续可通行道路片段
- `Intersection`
  - 表示道路连接区域

这层是后续“道路分向图”的核心。

## 3. Static Semantic Layer

这一层挂接长期稳定存在的静态语义实体，包括：

- `StaticObject`

例如：

- 静态车辆
- 停靠车辆
- 交通灯
- 交通牌
- 路锥
- 施工设施
- 公交站等

这些节点不作为道路拓扑骨架，而是附着在：

- `RoadSegment`
- `Intersection`

之上。

## 4. Dynamic Layer

这一层专门处理动态目标，不应并入静态主图。

建议表示为：

- `DynamicTrack`
- `TrajectoryState`

也就是：

- 一个 `DynamicTrack` 对应一个时态代理
- 每个关键时刻挂一个 `TrajectoryState(t)`

这层是：

- 时态层
- overlay 层
- 不要求永久持久化到静态地图主图中

## 六、推荐的最小可实现版本（MVP）

第一版场景图不建议一开始就追求精细车道线级别，而建议先做以下最小集合。

## 1. 节点类型

- `RoadSegment`
- `Intersection`
- `StaticObject`
- `DynamicTrack`

## 2. 边类型

第一版边类型建议只做：

- `connects_to`
- `left_turn_to`
- `right_turn_to`
- `merge_into`
- `belongs_to_intersection`
- `located_on_segment`
- `near_segment`
- `observed_from`
- `moving_along`
- `approaching_intersection`
- `parked_on`

这已经足够表达：

- 道路方向性
- 道路之间的连通性
- 交叉口拓扑
- 物体和道路的空间关系
- 动态车和道路的时态关系

## 七、动态对象的建模建议

## 1. 动态对象不要进静态主图

最重要的原则是：

- 动态对象不要被当成永久静态地图节点

动态车不应该像静态车一样并进静态 scene graph 主骨架，否则：

- 图会不稳定
- 地图会混乱
- 动态噪声会污染长期结构

## 2. 动态对象更适合建成时态代理

推荐表示形式：

- 一个 `DynamicTrack` 节点对应一辆车
- 每个关键时刻挂一个 `TrajectoryState`

每个 `TrajectoryState` 可以存：

- `timestamp`
- `position`
- `yaw`
- `velocity`
- `bbox size`
- `confidence`
- 可选：`lane-relative s/l`
- `motion_state`

`motion_state` 第一版建议只做：

- `moving`
- `stopped`
- `parked`
- `unknown`

## 3. 动态层里的关系

第一版最容易做的关系是：

- `moving_along(segment)`
- `approaching(intersection)`
- `inside_intersection`
- `leaving_intersection`
- `stopped_on(segment)`
- `parked_on(segment)`
- `following(track)`

这些关系都可以先用：

- 几何位置
- 速度
- 航向
- 轨迹连续性

做规则化判断，不必一开始就用复杂学习器。

## 4. 离线统计层

动态信息除了保留短期轨迹，还可以逐步压缩成“交通模式层”：

- 某段路常见车流方向
- 某路口常见左转 / 右转流
- 某些区域常见临停
- 某些交叉口常有拥堵 / 等待

这样动态信息就不只是“单辆车”，而是：

- `traffic pattern graph`

这对论文表达很加分。

## 八、VLM 在这套图中的角色

## 1. VLM 的角色定位

VLM 的最佳定位是：

- **异步高层语义标注器**

即：

- 给稳定节点补语义
- 给候选连通关系补高层判别
- 给区域生成 scene-level summary

而不是：

- 高频推理器
- 几何更新器
- pose 优化器

## 2. VLM 最适合处理的对象

### 2.1 `RoadSegment`

VLM 可为道路节点补充：

- `residential street`
- `main road`
- `service road`
- `parking access lane`
- `bus lane`

### 2.2 `Intersection`

VLM 可为路口节点补充：

- `T-junction`
- `crossroad`
- `signalized intersection`
- `roundabout entry`
- `merge area`

### 2.3 `StaticObject / DynamicTrack`

VLM 可为稳定对象补充开放词汇属性：

- `sedan`
- `SUV`
- `truck`
- `bus`
- `delivery vehicle`
- `emergency vehicle`
- `illegally parked vehicle`

但这里建议排在道路语义之后，不要一开始就把 VLM 的重点放在 object 细粒度类别上。

## 3. VLM 最值得先做的任务

当前最值得先做的不是“给每个 object 打复杂标签”，而是：

### 3.1 路口识别

VLM 判断：

- 当前是否为 `intersection`
- 如果是，属于什么类型路口

### 3.2 区域切分

VLM 判断：

- 当前是否进入新的道路语义区域
- 是否应该新建一个新的 `RoadSegment / PlaceNode`

### 3.3 连通关系辅助判别

当几何规则已经给出候选边后，VLM 再辅助判断：

- `straight`
- `left_turn`
- `right_turn`
- `merge`
- `split`

### 3.4 区域级语义标注

例如：

- `commercial block`
- `residential area`
- `construction area`
- `dense traffic zone`
- `curbside parking`

## 4. VLM 的输入形式建议

VLM 输入不建议是单帧，而建议是：

- 3 到 4 张代表性多相机关键帧图像
- 1 张当前区域的 BEV / object map 快照
- 1 份简短 object inventory

例如：

- `cars=12, buses=1, cones=3`

这样比单帧更稳，也更适合高层语义判断。

## 5. VLM 的输出形式建议

第一版输出可以收敛成：

- `scene_type`
- `need_new_topology_node`
- `node_type`
- `region_semantics`
- `connectivity_hint`
- `confidence`

也就是说，VLM 主要输出的是：

- 高层类别
- 节点建议
- 连通性提示

## 九、技术路径建议

## 阶段 1：先接入真实 object 输入

在语义场景图之前，仍然建议先完成：

- `StreamPETR` 结果接入

因为之后：

- 动态轨迹层
- 场景级 object inventory
- VLM 场景级推理

都需要真实检测 / 跟踪结果来验证。

## 阶段 2：构建道路拓扑骨架图

先实现：

- `RoadSegment`
- `Intersection`

不要求一开始做到车道线级别高精地图，只做：

- 分向道路段
- 路口节点
- 基础连通关系

## 阶段 3：把 static object 挂到拓扑图上

利用当前已经有的：

- static / dynamic / unknown
- object lifecycle
- object-only optimization

把：

- `static object`

挂到最近的：

- `RoadSegment`
- `Intersection`

节点上。

## 阶段 4：加入 dynamic track overlay

动态车先不进静态主图，只做：

- `DynamicTrack`
- `TrajectoryState`
- 和 `RoadSegment / Intersection` 的关系

## 阶段 5：加入 VLM 异步标注

这一步再让 VLM 去处理：

- 路口识别
- 区域切分
- 连通关系语义确认
- 区域语义 summary

## 阶段 6：加入更高级的动态关系和交通模式

后续再做：

- `following`
- `yielding_to`
- `traffic flow pattern`
- `frequent parking / congestion prior`

## 十、预期效果

如果按上述路线推进，预期可以达到以下效果。

## 1. 对系统实现的预期效果

- scene graph 有清晰骨架
- 不会让动态物体污染静态地图
- VLM 与几何主链解耦
- 更容易逐步实现，不会一下子把系统复杂度拉爆

## 2. 对论文叙事的预期效果

- 可以形成一个很顺的三段式故事：
  - SLAM 提供几何结构
  - 检测 / tracking 提供 object observation 与动态代理
  - VLM 提供高层场景拓扑语义

- 可以把最终系统讲成：
  - 带道路连通性
  - 带对象语义
  - 带动态行为上下文

的分层场景图系统。

## 3. 对后续扩展的预期效果

这一路线后续也天然支持：

- 更细粒度 lane-level graph
- 交通模式统计
- offline VLM refinement
- open-vocabulary place retrieval
- 基于高层语义的回环与重定位

## 十一、当前阶段一句话总结

当前最合理、最便于实现的语义场景图路线是：

- **以有方向的道路拓扑图为骨架**
- **静态物体附着其上**
- **动态车辆作为时态轨迹层叠加**
- **VLM 只负责高层拓扑语义推理和区域语义标注**

这比直接做 object-centric open-vocabulary graph 更稳，也更符合当前系统的工程现实和论文主线。
