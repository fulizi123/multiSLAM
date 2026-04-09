# 2026-03-22 VLM 语义场景图阶段记录

## 一、文档目的

这份文档用于记录当前仓库中“VLM 语义场景图”方向的阶段性进展，重点回答 3 个问题：

- 这一块整体想做成什么
- 今天已经实际做了哪些事
- 接下来最值得优先补的部分是什么

本文不是最终方法定稿，而是：

- 当前工程实现进展
- 当前方法理解
- 后续清晰推进方向

的整理记录。

## 二、当前整体目标

当前这条线的核心目标，不是让 VLM 直接参与几何 SLAM，而是让它承担：

- **高层语义推理器**

也就是：

- 利用多相机图像语义
- 判断当前处于什么类型的道路区域
- 判断是否接近 / 进入路口
- 提供道路拓扑节点与边的候选提示
- 为后续语义场景图提供更高层的区域语义

一句话概括：

- **VLM 负责高层语义与拓扑提示，不负责位姿和几何优化**

这条主线和前面的脑暴文档保持一致：

- [`SCENE_GRAPH_VLM_BRAINSTORM_2026-03-21.md`](/data/code/LL_SLAM_MultiCamera/SCENE_GRAPH_VLM_BRAINSTORM_2026-03-21.md)

## 三、当前对“分层语义场景图”的理解

当前更清晰的理解是：

- 现有系统已经具备一个分层语义场景图的雏形

推荐的层次不是两层，而是 **三层**：

### 1. Entity / Object Layer

这一层主要放：

- `MapObject`
- `StaticAnchor`
- `DynamicTrack`

这一层表达的是：

- 物体级语义
- 静态语义锚点
- 动态车辆或轨迹代理

### 2. Topology Layer

这一层是场景图真正的主骨架，主要放：

- `RoadSegment`
- `Intersection`

这一层表达的是：

- 道路段之间的连通关系
- 路口与道路之间的拓扑结构

### 3. Region / Zone Layer

这一层用于表达比单一路段更高层的区域语义，例如：

- `residential_area`
- `commercial_area`
- `urban_core`
- `construction_zone`
- `parking_complex`

这一层表达的是：

- 一片区域整体的风格
- 更大尺度的语义归属

当前判断是：

- `Object Layer` 和 `Topology Layer` 之间尺度差异太大
- 如果没有 `Region Layer`，很多“城市区域级语义”没有合适的位置承载

因此后续更推荐采用：

- **物体层 + 拓扑层 + 区域层**

的三层结构。

## 四、今天实际完成的工作

今天这部分工作，已经从“脑暴”进入到了第一版工程落地。

### 1. 明确了 VLM 的工程定位

今天已经进一步统一了方法主线：

- VLM 不参与几何优化
- VLM 不回传 `ego pose`
- VLM 不直接修正 object 几何
- VLM 当前只产出：
  - `scene_type`
  - `intersection_state`
  - `road_semantics`
  - `topology_node_hint`
  - `outgoing_topology_hints`
  - `traffic_controls`
  - `static_semantic_anchors`

这意味着当前这条线的定位已经比较清晰：

- **VLM 是高层语义观测源，不是低层几何源**

### 2. 接通了离线 VLM 刷库脚本

今天已经在：

- [`/data/code/carla_nuscenes/generate_vlm_scene_graph.py`](/data/code/carla_nuscenes/generate_vlm_scene_graph.py)

中实现了一版离线批处理脚本。

它的目标是：

- 对转换后的 CARLA 数据集做稀疏 VLM 语义标注
- 把结果写回当前 SLAM 实际使用的数据目录

当前输出目录为：

- [`/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph`](/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph)

### 3. 完成了模型与调用配置的工程收敛

今天一开始尝试了更大的模型，但实际调用过程中暴露出：

- 图像请求延迟高
- 视觉返回慢
- 在 reasoning 模式下可能只返回 `reasoning_content`

最后收敛到当前更实用的一套默认配置：

- 模型：
  - `Qwen/Qwen3.5-35B-A3B`
- SiliconFlow OpenAI 兼容接口：
  - `https://api.siliconflow.cn/v1`
- 在调用时显式关闭 thinking：
  - `enable_thinking = false`
- 要求 JSON 输出：
  - `response_format = json_object`

这一步非常关键，因为它解决了前面两个具体问题：

#### 3.1 视觉请求太重

更大的模型在当前多图输入下，延迟太高，不适合先做第一版离线刷库。

#### 3.2 content 为空

在默认 thinking 模式下，模型会把 token 花在：

- `reasoning_content`

从而出现：

- `message.content` 为空

关闭 thinking 后，这个问题已经解决。

### 4. 把 VLM 输入改成了更轻的 sparse 方案

今天非常明确的一点是：

- 没必要每次都把 12 个相机全量喂给 VLM

当前已经改成了更轻的默认策略：

- `front5` 相机子集
  - `Cam09`
  - `Cam01`
  - `Cam05`
  - `Cam10`
  - `Cam04`
- 默认抽帧：
  - `stride = 10`

这样当前 VLM 看到的，不再是 12 路全图，而是：

- 更贴近道路拓扑判断所需的前方与侧向视角

这一步的意义是：

- 降低 token 压力
- 降低请求时延
- 保留对道路与路口语义最有帮助的视角

### 5. 完成了“稀疏语义帧 -> 全帧查询索引”的设计和输出

今天并没有把 VLM 当成“每帧在线调用模块”，而是做成了：

- 稀疏离线语义锚点

同时额外输出了：

- [`frame_query_index.json`](/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph/frame_query_index.json)

这个文件的作用是：

- 对 1200 帧中的每一帧
- 给出最近的稀疏语义帧
- 方便后续 SLAM 运行时按当前帧去查询最近的 VLM 语义结果

这意味着当前系统思路已经从：

- “每帧在线问 VLM”

转成：

- **离线 sparse semantic library + 在线近邻检索**

这是更合理的工程路线。

### 6. 已完成小批量验证，并保留了部分实际结果

今天已经做过：

- 单帧验证
- 小批量多帧验证

当前已经成功写入部分语义帧文件到：

- [`/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph/frame_semantics`](/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph/frame_semantics)

当前已保存的稀疏语义帧已经达到：

- `000000`
- `000010`
- `000020`
- ...
- `000190`

这说明：

- 当前稀疏版 VLM 管线已经能稳定产出结果

### 7. 给当前 SLAM Viewer 增加了“关键帧组合图”导出能力

在继续尝试 VLM 推理之后，一个明确的问题是：

- 单纯喂多相机图像，模型对“当前是否已经进入路口区域”的判断仍然不够稳

因此今天又往前推进了一步：

- 直接利用当前 SLAM 系统已有的 Viewer 可视化大图
- 把“8 相机预览 + 地图点 / 轨迹 / 当前对象面板”组成的组合图按关键帧落盘

这一步的意义是：

- 不只给模型喂图像纹理信息
- 还给它喂当前 SLAM 的可视化空间上下文

涉及的核心代码包括：

- [`include/Viewer.h`](/data/code/LL_SLAM_MultiCamera/include/Viewer.h)
- [`include/Tracking.h`](/data/code/LL_SLAM_MultiCamera/include/Tracking.h)
- [`src/Viewer.cc`](/data/code/LL_SLAM_MultiCamera/src/Viewer.cc)
- [`src/Tracking.cc`](/data/code/LL_SLAM_MultiCamera/src/Tracking.cc)
- [`Carla.yaml`](/data/code/LL_SLAM_MultiCamera/Carla.yaml)

当前行为是：

- 仍然沿用现有关键帧触发逻辑
  - 也就是当前实现中的每 `10` 帧一次
  - 加初始化关键帧
- 在 Viewer 完成组合图渲染后
- 把关键帧组合图保存到：
  - [`/data/code/carla/dataset_nusc_0307_1704_carla/KeyFrameVisualization`](/data/code/carla/dataset_nusc_0307_1704_carla/KeyFrameVisualization)

同时，还增加了 YAML 开关：

- [`Carla.yaml`](/data/code/LL_SLAM_MultiCamera/Carla.yaml)
  - `Viewer.SaveKeyFrameSnapshots: 1`

也就是：

- 后面如果想关掉关键帧可视化导出
- 只需要把配置改成 `0`
- 不需要再改代码

### 8. 新增一条更稳定的“CARLA 真值拓扑状态”最小链路

在继续分析 VLM 结果之后，今天进一步确认了一点：

- 如果当前目标是先尽快接一个可用 demo
- 直接依赖 VLM 推理结果，风险仍然偏高

因此今天额外补了一条：

- **GT pose + CARLA map waypoint 查询**

的最小真值链路。

也就是：

- 不让模型猜当前是不是路口
- 而是直接用当前数据集中的 GT 位姿
- 去 CARLA 的路网地图上查 waypoint / lane / junction 状态

新增脚本为：

- [`/data/code/carla_nuscenes/generate_carla_topology_status.py`](/data/code/carla_nuscenes/generate_carla_topology_status.py)

它读取：

- [`/data/code/carla/dataset_nusc_0307_1704_carla/gt_pose_frames.json`](/data/code/carla/dataset_nusc_0307_1704_carla/gt_pose_frames.json)

然后利用 live CARLA map 查询：

- `road_id`
- `section_id`
- `lane_id`
- `junction_id`
- `is_junction`
- `junction_proximity`
- `next_junction_distance_m`

这一步的意义是：

- 先拿到稳定的“当前在不在路口 / 当前在哪条 lane 上”的结构化标签
- 方便先在 SLAM 里接一个小 demo
- 后面再慢慢把 VLM 场景图那条线补强

### 9. CARLA 真值拓扑状态已经改成逐帧 `.bin` 输出

在这条真值拓扑状态链路上，最开始一版输出包含的字段偏多，更像分析导出，不适合直接进 C++。因此今天又继续收缩了一版。

当前最终输出结构变成：

- 保留可读输出：
  - `json`
  - `csv`
- 增加逐帧 `.bin`：
  - 和现有 `ObjectTrack/*.bin`、`SuperPoint*.bin` 的目录风格保持一致

当前 `.bin` 输出目录为：

- [`/data/code/carla/dataset_nusc_0307_1704_carla/CarlaTopologyStatus`](/data/code/carla/dataset_nusc_0307_1704_carla/CarlaTopologyStatus)

文件格式是：

- `000000.bin`
- `000001.bin`
- ...

并且每个 `.bin` 中现在只保留 7 个最小字段：

1. `road_id`
2. `section_id`
3. `lane_id`
4. `junction_id`
5. `is_junction`
6. `junction_proximity_id`
7. `next_junction_distance_m`

这里的设计原则是：

- 文件名本身就携带 `frame_index`
- demo 阶段只保留当前 SLAM 真正最有可能直接消费的字段
- 字符串字段不直接写进 `.bin`
- 只保留一个必要的枚举映射：
  - `junction_proximity_id_map`

映射与布局说明会写到：

- [`/data/code/carla/dataset_nusc_0307_1704_carla/carla_topology_status_meta.json`](/data/code/carla/dataset_nusc_0307_1704_carla/carla_topology_status_meta.json)

这样做之后，这条“CARLA 真值拓扑状态”链路已经非常适合后面直接接到 C++ / SLAM 里。

### 10. 已把逐帧 `CarlaTopologyStatus` 最小 `.bin` 接进当前 SLAM 主链

在得到更轻的逐帧真值拓扑状态 `.bin` 之后，今天进一步把这条链真正接进了当前 `LL_SLAM_MultiCamera`。

当前接入路径是：

- `Examples/stereo_carla_multi.cc`
- `System`
- `Tracking`
- `Frame`
- `Viewer`

也就是说：

- `Example` 主程序现在会像读取 `ObjectTrack/*.bin` 一样
- 逐帧读取：
  - [`/data/code/carla/dataset_nusc_0307_1704_carla/CarlaTopologyStatus`](/data/code/carla/dataset_nusc_0307_1704_carla/CarlaTopologyStatus)
- 然后一路把当前帧对应的 `CarlaTopologyStatus` 传到 `Frame`
- `Viewer` 再直接读取当前帧的状态做高层语义可视化

这一步的意义是：

- 当前 SLAM 已经不只是“离线外部导出标签”
- 而是已经真正消费这份高层语义状态输入

涉及的核心文件包括：

- [`include/CarlaTopologyStatus.h`](/data/code/LL_SLAM_MultiCamera/include/CarlaTopologyStatus.h)
- [`include/System.h`](/data/code/LL_SLAM_MultiCamera/include/System.h)
- [`include/Tracking.h`](/data/code/LL_SLAM_MultiCamera/include/Tracking.h)
- [`include/Frame.h`](/data/code/LL_SLAM_MultiCamera/include/Frame.h)
- [`src/System.cc`](/data/code/LL_SLAM_MultiCamera/src/System.cc)
- [`src/Tracking.cc`](/data/code/LL_SLAM_MultiCamera/src/Tracking.cc)
- [`src/Frame.cc`](/data/code/LL_SLAM_MultiCamera/src/Frame.cc)
- [`Examples/stereo_carla_multi.cc`](/data/code/LL_SLAM_MultiCamera/Examples/stereo_carla_multi.cc)

当前最小状态结构只保留了 demo 真正需要的字段：

- `road_id`
- `section_id`
- `lane_id`
- `junction_id`
- `is_junction`
- `junction_proximity_id`
- `next_junction_distance_m`

这意味着：

- 当前高层语义输入已经足够支持“道路 / 路口 + lane / road identity”的最小 demo

### 11. Viewer 中已经增加高层语义背景可视化，并升级成历史语义带

在刚把 `CarlaTopologyStatus` 接进来之后，今天没有选择一开始就做复杂几何 lane mesh，而是先做了一个非常实用的最小可见效果：

- **浅色高层语义背景层**

第一版一开始只是画当前 ego 周围的语义块，但这会有一个明显问题：

- 车辆开走之后，语义块也会消失
- 不能像轨迹一样在地图上保留“之前走过的区域类型”

因此今天又继续升级成了：

- **沿历史位姿累计的语义带状区域**

当前行为是：

- `Viewer` 维护历史 `Twbs`
- 同时维护历史 `CarlaTopologyStatus`
- 对相邻位姿段构造一个横向宽度固定的带状四边形
- 按当前段对应的语义状态给这个区域着色
- 持续叠加在地图背景层上

这样做之后：

- 车走过的高层语义会留在地图上
- 更像“语义走廊带”
- 而不是瞬时块

同时，根据今天后续调整，当前可视化还有两个重要特征：

#### 11.1 不再做前后额外延展

当前语义区域不再是“当前车体前后延展的大块矩形”，而是：

- 只沿车辆历史增量方向累计
- 由相邻位姿段自然铺开

这更符合你希望的：

- “像轨迹一样逐步积累区域类型”

#### 11.2 颜色和透明度做了偏保守但可辨认的设置

当前颜色策略是：

- 普通道路：
  - 按 `road_id` 哈希生成不同浅色
- 路口：
  - 统一一个浅橙色系

并且在后续调试中又做了两处调整：

- 语义带宽度：
  - 从左右 `5m` 扩大到左右 `7m`
- alpha 透明度：
  - 从更浅调高到更容易区分的程度

设计原则依然保持不变：

- 这层高层语义只是背景层
- 不允许遮住已有地图点、连线、对象框与相机预览图
- 轨迹原本的颜色保持不变

涉及的主要文件包括：

- [`include/Viewer.h`](/data/code/LL_SLAM_MultiCamera/include/Viewer.h)
- [`src/Viewer.cc`](/data/code/LL_SLAM_MultiCamera/src/Viewer.cc)

### 12. 当前这一阶段的意义

经过今天这一步，系统已经形成了一条非常实用的最小 demo 路线：

1. 用 GT pose + CARLA map 查询得到逐帧拓扑真值状态
2. 把状态导出成逐帧 `.bin`
3. 在当前 SLAM 主程序中逐帧读入
4. 在 Viewer 中把它变成可见的高层语义背景层

这意味着：

- 当前即使不依赖 VLM 推理结果
- 系统也已经能展示一层高层语义地图信息
- 这为后面把 VLM 结果替换或补充到同一接口上提供了非常好的工程落点

## 五、今天这部分最重要的认识

今天最关键的认识，不是“换了一个模型”，而是更明确了两件事：

### 1. VLM 现在还是偏 image-only semantic observation

虽然当前脚本中附带了少量上下文提示，例如：

- `frame_index`
- `timestamp`
- `object count hint`

但本质上当前 VLM 仍然主要依赖：

- 输入图像语义

它还没有真正利用这些更强的上下文：

- SLAM 当前位姿历史
- 当前图中的 `RoadSegment / Intersection`
- 已有对象地图摘要
- 相邻语义帧历史
- 局部轨迹趋势

因此当前更准确地说是：

- **图像主导的高层语义观测**

还不是：

- **图像 + SLAM 上下文联合推理**

### 2. 目前的图结构雏形已经有了，但层次还不够完整

当前已经能看出两个稳定层次：

- 物体级语义
- 道路 / 路口拓扑语义

但如果希望把“城市区域级语义”也纳入系统，例如：

- 商业区
- 居住区
- 城市核心区
- 施工区域

就必须再补一个更高层的：

- `Region / Zone Layer`

因此当前最合理的长期方向，已经比较明确地变成了：

- **三层分层语义场景图**

## 六、后续最值得优先补的两大块

当前这条线后续最该补的，不是简单继续刷更多帧，而是下面这两大块。

### A. VLM 输入与上下文增强

这是第一大块。

当前 VLM 主要问题不是“看不见”，而是：

- 缺上下文
- 缺连续性
- 缺图状态约束

后面应优先补的输入包括：

- 当前 SLAM 的最近语义节点
- 最近若干个稀疏语义帧结果
- 当前局部对象摘要
  - 静态锚点数量
  - 停靠车密度
  - 交通灯 / 交通牌存在性
- 最近若干帧的轨迹趋势
  - 直行
  - 左偏
  - 右偏
  - 接近路口
- 当前已知 `RoadSegment / Intersection / Region` 候选

也就是说，后面更理想的 VLM 输入应从：

- `image-only`

升级成：

- **image + SLAM structured context**

这一块的主要目标是：

- 降低单帧误判
- 提高语义稳定性
- 让 VLM 真正成为“高层语义推理器”

### B. 场景图结构扩展到三层

这是第二大块。

当前最推荐的最终结构是：

#### 1. Object / Entity Layer

- `StaticAnchor`
- `DynamicTrack`
- `MapObject`

#### 2. Topology Layer

- `RoadSegment`
- `Intersection`

#### 3. Region / Zone Layer

- `Region`
- `Zone`

这三层之间需要定义清楚：

- 每层有哪些节点
- 每层有哪些边
- 跨层如何挂接

例如：

- `StaticAnchor -> located_on -> RoadSegment`
- `RoadSegment -> in_region -> Region`
- `Intersection -> in_region -> Region`
- `DynamicTrack -> approaching -> Intersection`

这一块的主要目标是：

- 让高层语义有清晰承载结构
- 避免所有信息都挤在“路段 / 路口”这一层
- 给后续论文叙事和系统扩展留清晰接口

## 七、当前建议的下一步

如果继续推进这条线，最合理的顺序建议是：

1. 继续把 sparse VLM library 跑完
2. 定义三层 scene graph 的节点 / 边 schema
3. 给 VLM 再补一版结构化上下文输入
4. 在 SLAM 主程序里增加“按当前帧查询最近语义帧”的读取逻辑
5. 再考虑从“语义观测”升级到“语义图状态更新”

当前不建议马上做的事是：

- 让 VLM 直接改 pose
- 让 VLM 直接改 object 几何
- 把所有动态对象直接并入静态语义图主骨架

## 八、涉及的当前主要文件

### 仓库内文档

- [`SCENE_GRAPH_VLM_BRAINSTORM_2026-03-21.md`](/data/code/LL_SLAM_MultiCamera/SCENE_GRAPH_VLM_BRAINSTORM_2026-03-21.md)
- [`VLM_SCENE_GRAPH_PROGRESS_2026-03-22.md`](/data/code/LL_SLAM_MultiCamera/VLM_SCENE_GRAPH_PROGRESS_2026-03-22.md)

### 代码与数据输出

- [`/data/code/carla_nuscenes/generate_vlm_scene_graph.py`](/data/code/carla_nuscenes/generate_vlm_scene_graph.py)
- [`/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph`](/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph)
- [`/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph/frame_semantics`](/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph/frame_semantics)
- [`/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph/frame_query_index.json`](/data/code/carla/dataset_nusc_0307_1704_carla/VLMSceneGraph/frame_query_index.json)
