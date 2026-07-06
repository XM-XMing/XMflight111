# XMflight--面向森林环境无人机自主飞行的 Unity 仿真平台

XMflight 基于 Unity 的无人机森林环境仿真工程，面向路径规划、避障控制、强化学习和深度视觉感知算法验证。工程在 Unity 端构建树木障碍物场景，模拟无人机位姿演化、碰撞检测、前向安全距离感知和 D435i 风格深度图采集，并通过 ZMQ + MessagePack 与外部 ROS/Python 客户端交换控制指令、状态帧和深度帧。

---

## 1. 项目定位

| 项目 | 内容 |
|---|---|
| 平台类型 | Unity 仿真环境 |
| 研究对象 | 森林障碍物场景下的无人机自主飞行 |
| 外部接口 | ZMQ + MessagePack |
| 外部客户端 | ROS 节点、Python 脚本、强化学习训练进程 |
| 控制输入 | 速度控制、位置重置、Step 模式速度控制与目标修正 |
| 感知输出 | 碰撞状态、安全距离、D435i 风格 16UC1 深度图 |
| 主要用途 | 规划算法验证、避障策略验证、深度感知算法测试、RL 环境交互 |

工程核心流程为：外部客户端发送控制指令，Unity 端解析指令并更新无人机位姿，同时执行碰撞检测和深度图采集，最后通过独立 ZMQ 通道发布状态帧与深度帧。

---

## 2. 工程指标

| 指标 | 数值 |
|---|---:|
| Unity 版本 | `2022.3.62f2c1` |
| 主场景 | `Assets/Scenes/Forest.unity` |
| 启用构建场景数 | 1 |
| 自定义 `.cs` 文件 | 11 个 |
| 自定义 `.cs` 代码量 | 3337 行 |
| ComputeShader 文件 | 1 个，88 行 |
| 第三方 MessagePack 源码 | 104 个文件 |
| 插件 DLL | 4 个 |
| 树木 Prefab 类型 | 17 类，`T01`~`T17` |
| 主场景树木实例 | 4000 棵 |
| 主场景树木 MeshCollider | 4000 个 |
| 默认物理步长 | `0.02 s` |
| 默认状态频率 | `1 / 0.02 = 50 Hz`，当 `publishStride=1` |
| 默认目标渲染帧率 | `Application.targetFrameRate≈50` |
| 默认深度图分辨率 | `640 × 480` |
| 默认深度图编码 | `16UC1`，小端序 |
| 默认深度 payload | `640 × 480 × 2 = 614400 bytes/frame` |
| 深度 payload 带宽估计 | `614400 × depth_fps` bytes/s；当 `depth_fps=50` 时约 `29.30 MiB/s` |

状态帧和深度帧不是同一个调度源。状态帧由 `FixedUpdate()` 驱动，深度帧由相机渲染和 GPU 读回驱动。因此，二者默认配置下目标频率接近，但实际运行时不必严格相同。

---

## 3. 系统架构

### 3.1 通信与仿真流程

![](/home/xm/XM/XMflight/xmflight_sim_flow.png)

图中包含 3 条关键链路：

| 链路 | 触发模块 | 数据方向 | 端口 |
|---|---|---|---:|
| 控制指令 | 外部 ROS/Python → `XMSimulationManager.PollCommands()` | 客户端到 Unity | `10253` |
| 状态帧 | `XMSimulationManager.FixedUpdate()` → `PublishDynamicsState()` | Unity 到客户端 | `10254` |
| 深度帧 | `XMImageSynthesis.OnRenderImage()` → `AsyncGPUReadback` → `PublishDepthFrame()` | Unity 到客户端 | `11254` |

### 3.2 运行时模块分工

| 模块 | 文件 | 职责 |
|---|---|---|
| 仿真管理器 | `XMSimulationManager.cs` | ZMQ 初始化、控制指令解析、运动学积分、状态帧/深度帧发布 |
| 碰撞传感器 | `XMCollisionSensor.cs` | 机体碰撞检测、OverlapSphere 邻域检测、RaycastCommand 安全距离估计 |
| 深度相机 | `XMImageSynthesis.cs` | Replacement Shader 深度渲染、RFloat 深度缓存、16UC1 深度图转换 |
| 协议定义 | `XMProtocol.cs` | MessagePack 控制指令、状态帧、深度元数据结构定义 |
| 参数配置 | `XMConfig.cs` | ScriptableObject 参数配置项定义 |
| 坐标转换 | `XMConverters.cs` | ROS 坐标系与 Unity 坐标系转换 |
| 森林生成 | `ForestGenerator.cs` | 随机森林生成、瓦片索引、碰撞网格生成、地图导出 |
| GPU 扫描 | `ScannerGPU.cs` | ComputeShader 点云采样调度与导出 |
| GPU 采样核 | `ForestScanner.compute` | 树木三角面随机点采样 |
| 跟随相机 | `CameraFollow.cs` | 主相机跟随无人机 |
| 旋翼显示 | `Spin.cs` | 旋翼视觉旋转 |

---

## 4. 工程目录

```text
XMflight/
├── Assets/
│   ├── Config/                           # 当前文件夹右键─>Creat─>XMflight─>Config生成*.asset 文件
│   │   └── XMFlightConfig.asset          # 统一仿真参数配置
│   ├── Materials/
│   │   ├── Drone.fbx
│   │   └── Tree/T01.fbx ... T17.fbx
│   ├── Plugins/
│   │   ├── Microsoft.NET.StringTools.dll
│   │   ├── System.Runtime.CompilerServices.Unsafe.dll
│   │   └── Zmq/AsyncIO.dll, NetMQ.dll
│   ├── Prefabs/
│   │   ├── Drone.prefab
│   │   └── T01.prefab ... T17.prefab
│   ├── Scenes/
│   │   ├── Forest.unity                  # 主场景
│   │   └── SampleScene.unity
│   ├── Scripts/
│   │   ├── XMSimulationManager.cs        # 主仿真管理器
│   │   ├── XMCollisionSensor.cs          # 碰撞与安全距离检测
│   │   ├── XMImageSynthesis.cs           # D435i 风格深度相机
│   │   ├── XMProtocol.cs                 # MessagePack 协议定义
│   │   ├── XMConfig.cs                   # ScriptableObject 参数定义
│   │   ├── XMConverters.cs               # ROS/Unity 坐标转换
│   │   ├── ForestGenerator.cs            # 森林生成与地图导出
│   │   ├── ScannerGPU.cs                 # GPU 点云扫描
│   │   ├── ForestScanner.compute         # GPU 三角面采样核
│   │   ├── XMConstants.cs                # 常量定义
│   │   ├── CameraFollow.cs               # 跟随相机
│   │   ├── Spin.cs                       # 旋翼可视化
│   │   └── MessagePack/                  # 第三方序列化源码
│   └── Shaders/
│       ├── XM_UberReplacement.shader     # 深度替换 Shader
│       └── XM_DepthVisualize.shader      # 深度可视化 Shader
├── Packages/
│   ├── manifest.json
│   └── packages-lock.json
├── ProjectSettings/
├── README.md
└── xmflight通信与仿真流程图.png
```

---

## 5. 环境要求

| 组件 | 要求 |
|---|---|
| Unity Editor | `2022.3.62f2c1`，建议使用同版本 |
| 渲染管线 | Built-in Render Pipeline；工程使用 Replacement Shader |
| 通信库 | NetMQ / AsyncIO |
| 序列化 | MessagePack |
| 推荐客户端 | Python + `pyzmq` + `msgpack`，或 ROS 节点封装 ZMQ |
| 操作系统 | Windows/Linux 需分别验证 DLL 和路径配置 |

主要 Unity 包依赖：

| 包名 | 版本 |
|---|---:|
| `com.unity.postprocessing` | 3.4.0 |
| `com.unity.textmeshpro` | 3.0.7 |
| `com.unity.timeline` | 1.7.7 |
| `com.unity.toolchain.linux-x86_64` | 2.0.11 |
| `com.unity.visualscripting` | 1.9.4 |
| `com.unity.ugui` | 1.0.0 |

---

## 6. 快速复现

### 6.1 Unity 端启动

1. 使用 Unity Hub 打开工程根目录 `XMflight/`。
2. Unity Editor 版本选择 `2022.3.62f2c1`。
3. 打开主场景：

```text
Assets/Scenes/Forest.unity
```

4. 检查 Layer 配置：

| Layer ID | 名称 | 用途 |
|---:|---|---|
| 3 | `Obstacle` | 树木和障碍物碰撞层 |
| 6 | `Drone` | 无人机对象层 |

5. 检查核心对象挂载：

| 场景对象 | 必要组件 |
|---|---|
| `Manager` | `ForestGenerator`、`XMSimulationManager` |
| `ScanAllMap` | `ScannerGPU` |
| `Drone` Prefab | `XMCollisionSensor`、`XMImageSynthesis` |
| `Main Camera` | `CameraFollow` |

6. 点击 Play。Unity 端启动后应绑定 3 个 ZMQ 通道：

| 通道 | 方向 | 默认端口 |
|---|---|---:|
| 控制指令订阅 | 外部 → Unity | `10253` |
| 状态帧发布 | Unity → 外部 | `10254` |
| 深度帧发布 | Unity → 外部 | `11254` |

### 6.2 外部客户端连接

外部客户端应完成 3 个动作：

| 步骤 | 操作 | 数据格式 |
|---:|---|---|
| 1 | 向 `tcp://<unity_host>:10253` 发布控制指令 | MessagePack |
| 2 | 从 `tcp://<unity_host>:10254` 订阅状态帧 | MessagePack |
| 3 | 从 `tcp://<unity_host>:11254` 订阅深度帧 | multipart：元数据 + 深度 payload |

---

## 7. 参数配置

主要运行参数集中在 `Assets/Config/XMFlightConfig.asset`，对应类型定义在 `XMConfig.cs`。

```
Assets/Config文件夹下右键─>Creat─>XMflight─>Config生成*.asset 文件
```



### 7.1 通信参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `commandPort` | `10253` | 控制指令订阅端口 |
| `statePort` | `10254` | 状态帧发布端口 |
| `depthPort` | `11254` | 深度帧发布端口 |
| `publishStride` | `1` | 状态发布步长；状态频率 = 物理频率 / stride |
| `zmqHighWatermark` | `1` | ZMQ 高水位队列长度 |
| `depthQueueCapacity` | `2` | 深度帧发送队列容量 |

### 7.2 运动学参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `fixedDeltaTime` | `0.02` | s | Unity 物理步长 |
| `maxSpeed` | `5.0` | m/s | 速度指令幅值上限 |
| `timeConstantXY` | `0.06` | s | 水平速度一阶响应时间常数 |
| `timeConstantZ` | `0.06` | s | 垂直速度一阶响应时间常数 |
| `accFilterTime` | `0.15` | s | 加速度估计滤波时间常数 |
| `maxTiltAngleDeg` | `15` | deg | 姿态可视化最大倾角 |
| `maxAngularRateDeg` | `120` | deg/s | 姿态可视化最大角速度 |
| `visualTiltScale` | `0.65` | - | 视觉倾角缩放 |
| `ctrlLatencyFrames` | `0` | frame | 控制延迟帧数 |

### 7.3 碰撞检测参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `bodyRadius` | `0.5` | m | 无人机近似碰撞半径 |
| `frontRayDistance` | `4.0` | m | 前向射线检测距离 |
| `frontConeAngleDeg` | `60` | deg | 前向锥形检测角 |
| `frontRayCount` | `1024` | 条 | 前向射线数量 |
| `sphereRayCount` | `512` | 条 | 全局球面射线数量 |
| `frontRayRatio` | `0.65` | - | 前向射线比例目标 |
| `clearanceWarningDistance` | `1.2` | m | 安全距离预警阈值 |
| `nearCollisionDistance` | `0.65` | m | 近碰撞阈值 |
| `lowRiskRayCount` | `384` | 条 | 低风险射线数 |
| `mediumRiskRayCount` | `768` | 条 | 中风险射线数 |
| `maxRiskRayCount` | `1536` | 条 | 高风险射线数 |

### 7.4 深度相机参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `pixelWidth` | `640` | pixel | 深度图宽度 |
| `pixelHeight` | `480` | pixel | 深度图高度 |
| `minDepth` | `0.1` | m | 最小深度 |
| `maxDepth` | `20.0` | m | 最大深度 |
| `d435HFovDeg` | `86.0` | deg | 水平视场角 |
| `d435VFovDeg` | `57.0` | deg | 垂直视场角 |
| `useAsyncGPUReadback` | `true` | - | 是否使用异步 GPU 读回 |
| `forceSyncReadback` | `false` | - | 是否强制同步读回 |
| `noDepthValueMm` | `0` | mm | 无深度像素值 |

---

## 8. 运行模式

控制指令包含 3 种模式。

| 模式 | 名称 | 功能 |
|---:|---|---|
| 0 | `Velocity` | 速度控制；外部客户端发送期望速度 |
| 1 | `Teleport` | 位置重置；用于 episode reset 或指定初始位姿 |
| 2 | `Step` | 速度控制 + 目标位置修正；用于离散交互或训练步进 |

---

## 9. 运动学模型

当前工程采用运动学近似模型，而非基于刚体受力的六自由度动力学模型。速度响应使用一阶惯性环节，位置更新使用梯形积分。

### 9.1 速度一阶响应

```text
alphaXY = dt / (tauXY + dt)
alphaZ  = dt / (tauZ  + dt)

v_xz(k+1) = Lerp(v_xz(k), v_cmd_xz, alphaXY)
v_y(k+1)  = Lerp(v_y(k),  v_cmd_y,  alphaZ)
```

其中：

| 符号 | 含义 |
|---|---|
| `dt` | Unity 物理步长 |
| `tauXY` | 水平速度响应时间常数 |
| `tauZ` | 垂直速度响应时间常数 |
| `v_cmd` | 外部控制指令速度 |
| `v(k)` | 当前速度 |
| `v(k+1)` | 下一物理步速度 |

### 9.2 位置积分

```text
p(k+1) = p(k) + 0.5 × [v(k) + v(k+1)] × dt
```

该模型适用于算法闭环验证和训练环境交互；若研究对象为飞行动力学辨识、姿态控制或电机控制，需要替换为刚体动力学模型。

---

## 10. 坐标系约定

Unity 使用左手坐标系，ROS 常用坐标系以 `x` 前、`y` 左、`z` 上为约定。工程中通过 `XMConverters.cs` 进行转换。

| 转换 | 公式 |
|---|---|
| ROS 位置 → Unity 位置 | `[x, y, z] → [-y, z, x]` |
| Unity 位置 → ROS 位置 | `[x, y, z] → [z, -x, y]` |
| ROS 机体系速度 → Unity 机体系速度 | `[vx, vy, vz] → [-vy, vz, vx]` |

控制端和评估端必须使用同一坐标转换规则，否则会产生方向符号错误。

---

## 11. ZMQ + MessagePack 数据接口

### 11.1 端口定义

| 端口 | Socket 方向 | Unity 端角色 | 数据 |
|---:|---|---|---|
| `10253` | SUB | 接收 | 控制指令 |
| `10254` | PUB | 发布 | 状态帧 |
| `11254` | PUB | 发布 | 深度帧 |

### 11.2 控制指令

控制指令由外部客户端发送至 Unity。字段数量为 6。

| 序号 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | `SchemaVersion` | int | 协议版本 |
| 1 | `Mode` | int | 控制模式：0/1/2 |
| 2 | `Seq` | long | 指令序号 |
| 3 | `Velocity` | float[3] | 速度指令 |
| 4 | `Position` | float[3] | 目标位置或重置位置 |
| 5 | `YawDeg` | float | 偏航角，单位 deg |

### 11.3 状态帧

状态帧由 `XMSimulationManager` 通过 `tcp://*:10254` 发布。字段数量为 10。

| 序号 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | `SchemaVersion` | int | 协议版本 |
| 1 | `Seq` | long | 状态帧序号 |
| 2 | `SimTimeNs` | long | 仿真时间戳，单位 ns |
| 3 | `Position` | float[3] | 无人机位置 |
| 4 | `Rotation` | float[4] | 无人机四元数 |
| 5 | `Velocity` | float[3] | 当前速度 |
| 6 | `Acceleration` | float[3] | 当前加速度估计 |
| 7 | `MinClearance` | float | 最小安全距离 |
| 8 | `Flags` | int | 碰撞/状态标志位 |
| 9 | `FrontClearances` | float[3] | 左/中/右前向安全距离 |

状态频率计算：

```text
state_rate = 1 / fixedDeltaTime / publishStride
```

默认值：

```text
state_rate = 1 / 0.02 / 1 = 50 Hz
```

### 11.4 深度帧

深度帧由 `XMImageSynthesis` 采集，由 `XMSimulationManager` 通过 `tcp://*:11254` 发布。深度帧采用 ZMQ multipart：

| ZMQ 段 | 内容 | 格式 |
|---:|---|---|
| 0 | 深度元数据 | MessagePack |
| 1 | 深度图 payload | `16UC1` 原始字节流 |

深度元数据字段数量为 13：

| 序号 | 字段 | 说明 |
|---:|---|---|
| 0 | `SchemaVersion` | 协议版本 |
| 1 | `CaptureId` | 深度帧编号 |
| 2 | `SimTimeNs` | 仿真时间戳 |
| 3 | `Flags` | 状态标志 |
| 4 | `CapturePos` | 采集位置 |
| 5 | `CaptureRot` | 采集姿态 |
| 6 | `CaptureVel` | 采集速度 |
| 7 | `CaptureAcc` | 采集加速度 |
| 8 | `CaptureForward` | 前向向量 |
| 9 | `Camera` | 相机内参与深度范围 |
| 10 | `DepthMeta` | 深度图宽高、编码、步长、字节序 |
| 11 | `MinClearance` | 最小安全距离 |
| 12 | `FrontClearances` | 左/中/右前向安全距离 |

深度 payload 大小：

```text
payload_bytes = width × height × 2
              = 640 × 480 × 2
              = 614400 bytes/frame
```

深度 payload 带宽估计：

```text
depth_payload_bandwidth = 614400 × depth_fps bytes/s
```

当深度帧率为 50 Hz：

```text
614400 × 50 = 30,720,000 bytes/s ≈ 29.30 MiB/s
```

该估计不包含 MessagePack 元数据和 ZMQ multipart 帧头开销。元数据相对于 614400 bytes 的深度 payload 占比较小，但严格评估网络负载时应计入。

### 11.5 状态频率与深度频率差异

| 数据 | 触发位置 | 频率来源 | 默认估计 |
|---|---|---|---:|
| 状态帧 | `FixedUpdate()` | `1 / fixedDeltaTime / publishStride` | 50 Hz |
| 深度帧 | `OnRenderImage()` + `AsyncGPUReadback` | 渲染帧率和 GPU 读回性能 | 目标约 50 Hz，不保证 |

因此：

```text
state_rate = 1 / fixedDeltaTime / publishStride
depth_rate ≈ render_fps，但受 GPU、渲染负载和读回队列影响
```

---

## 12. Python 最小通信示例

以下示例仅用于验证 ZMQ 端口和 MessagePack 编解码流程。实际 ROS 封装应增加时间同步、异常重连、序号检查和坐标系转换。

```python
import time
import zmq
import msgpack

UNITY_HOST = "127.0.0.1"
CMD_PORT = 10253
STATE_PORT = 10254
DEPTH_PORT = 11254

ctx = zmq.Context.instance()

cmd_pub = ctx.socket(zmq.PUB)
cmd_pub.connect(f"tcp://{UNITY_HOST}:{CMD_PORT}")

state_sub = ctx.socket(zmq.SUB)
state_sub.connect(f"tcp://{UNITY_HOST}:{STATE_PORT}")
state_sub.setsockopt(zmq.SUBSCRIBE, b"")

depth_sub = ctx.socket(zmq.SUB)
depth_sub.connect(f"tcp://{UNITY_HOST}:{DEPTH_PORT}")
depth_sub.setsockopt(zmq.SUBSCRIBE, b"")

time.sleep(0.5)

schema_version = 2
seq = 1
mode = 0
velocity = [0.0, 0.0, 1.0]
position = [0.0, 1.0, 0.0]
yaw_deg = 0.0

cmd = [schema_version, mode, seq, velocity, position, yaw_deg]
cmd_pub.send(msgpack.packb(cmd, use_bin_type=True))

poller = zmq.Poller()
poller.register(state_sub, zmq.POLLIN)
poller.register(depth_sub, zmq.POLLIN)

while True:
    events = dict(poller.poll(timeout=1000))

    if state_sub in events:
        state = msgpack.unpackb(state_sub.recv(), raw=False)
        print("state:", state)

    if depth_sub in events:
        meta_bytes, depth_bytes = depth_sub.recv_multipart()
        meta = msgpack.unpackb(meta_bytes, raw=False)
        print("depth meta:", meta)
        print("depth payload bytes:", len(depth_bytes))
```

---

## 13. 场景数据

主场景 `Forest.unity` 中实例化 4000 棵树。每棵树包含 MeshRenderer 和 MeshCollider。

| 树木类型 | 数量 |
|---|---:|
| T01 | 220 |
| T02 | 261 |
| T03 | 263 |
| T04 | 226 |
| T05 | 193 |
| T06 | 257 |
| T07 | 235 |
| T08 | 182 |
| T09 | 247 |
| T10 | 238 |
| T11 | 218 |
| T12 | 253 |
| T13 | 246 |
| T14 | 226 |
| T15 | 261 |
| T16 | 234 |
| T17 | 240 |
| 合计 | 4000 |

场景结构适用于森林环境避障任务。若用于大规模训练，需关注 MeshCollider 数量和深度渲染负载对仿真吞吐率的影响。

---

## 14. 森林生成与地图扫描

工程提供两类地图构建能力：

| 功能 | 主要文件 | 输出/用途 |
|---|---|---|
| 森林生成 | `ForestGenerator.cs` | 随机布置树木、构建碰撞体、导出场景数据 |
| GPU 点云扫描 | `ScannerGPU.cs` + `ForestScanner.compute` | 对树木网格进行三角面随机采样，生成点云 |

当前代码中存在 Linux 路径硬编码：

```text
/home/xm/XM/xm_ws/src/planning/data/map_data
```

若在其他机器、Windows 路径或容器环境中运行，应将该路径改为自己路径

| 推荐方式 | 原因 |
|---|---|
| ScriptableObject 配置项 | 可在 Unity Inspector 中修改 |
| 命令行参数 | 便于批量实验 |
| 环境变量 | 便于集群或容器部署 |
| `Application.persistentDataPath` | Unity 跨平台路径兼容 |

---

## 15. 性能估算

### 15.1 状态通道

状态帧数据量远小于深度帧。默认 50 Hz 下，状态通道主要瓶颈不是带宽，而是外部客户端是否稳定消费消息。

### 15.2 深度通道

默认深度 payload：

```text
640 × 480 × 2 = 614400 bytes/frame
```

不同深度帧率下的原始 payload 带宽：

| 深度帧率 | payload 带宽 |
|---:|---:|
| 10 Hz | 5.86 MiB/s |
| 20 Hz | 11.72 MiB/s |
| 30 Hz | 17.58 MiB/s |
| 50 Hz | 29.30 MiB/s |
| 60 Hz | 35.16 MiB/s |

实际带宽高于表中数值，原因是还包含：

| 开销项 | 来源 |
|---|---|
| MessagePack 元数据 | 深度帧第 0 段 |
| ZMQ multipart 帧头 | ZMQ 通信层 |
| TCP/IP 协议栈开销 | 网络层 |
| 序列化/反序列化 CPU 成本 | Unity 和客户端 |

### 15.3 GPU 点云扫描内存

`ScannerGPU.ProcessBatchSafe()` 中单批最大点数为 `20,000,000`。按每个点 `xyz` 三个 float 估算：

```text
20,000,000 × 3 × 4 = 240,000,000 bytes ≈ 228.88 MiB
```

若同时存在 GPU buffer、CPU float 数组和导出 byte 数组，峰值内存可能超过 `3 × 228.88 MiB = 686.64 MiB`。实际值还需叠加 Unity 对象、Mesh、Collider 和渲染资源占用。

---

## 16. 已知问题与维护建议

| 优先级 | 位置 | 现象 | 建议 |
|---|---|---|---|
| 高 | `XMCollisionSensor.BuildBiasedSphere()` / `UpdateRayClearance()` | 低风险射线模式可能导致前向专用射线数量为 0 | 调整射线数组生成顺序，保证前向射线优先保留 |
| 中 | `ScannerGPU.ProcessBatchSafe()` | 点云导出峰值内存较高 | 降低批大小或采用流式写入 |
| 中 | `ForestGenerator.cs` / `ScannerGPU.cs` | 地图输出路径硬编码 | 改为配置项、命令行参数或环境变量 |
| 中 | `MessagePack.asmdef` | 引用 DLL 与工程实际插件可能不完全一致 | 在 Unity Editor 中执行完整编译验证 |
| 低 | `Spin.cs` | namespace 为 `XMFlight`，其他脚本多为 `XMflight` | 统一命名空间大小写 |
| 低 | `XMConfig.cs` | `isRlTrainingMode` 定义后未发现有效读取 | 删除冗余字段或补全使用逻辑 |
| 低 | `XMSimulationManager.cs` | `_prevVelocity` 赋值但未参与核心计算 | 删除冗余变量或接入状态估计流程 |
| 低 | `HandleTeleport()` | reset 后统计计数被清零 | 若需跨 episode 统计，应拆分 episode 计数和全局计数 |

---

## 17. 验证清单

首次运行或迁移平台时，建议按下表验证。

| 序号 | 验证项 | 通过标准 |
|---:|---|---|
| 1 | Unity 工程打开 | 无缺失脚本、无材质丢失、主场景可加载 |
| 2 | DLL 加载 | NetMQ、AsyncIO、MessagePack 相关程序集无编译错误 |
| 3 | ZMQ 端口 | `10253/10254/11254` 可绑定或连接 |
| 4 | 控制指令 | Mode 0 速度指令可改变无人机位姿 |
| 5 | Teleport | Mode 1 可重置无人机位置 |
| 6 | Step | Mode 2 可执行单步控制和目标修正 |
| 7 | 状态帧 | 客户端可稳定接收 `10254` 状态帧 |
| 8 | 深度帧 | 客户端可接收 multipart，payload 长度为 614400 bytes |
| 9 | 碰撞检测 | 接近树木时 `MinClearance` 下降，碰撞标志有效 |
| 10 | 坐标转换 | ROS 与 Unity 端位置、速度方向一致 |
| 11 | 性能 | 目标帧率、CPU、GPU、内存占用满足实验需求 |
| 12 | 日志 | reset、通信异常、深度队列溢出可定位 |

---

## 18. 适用任务

| 任务 | 适用性 | 说明 |
|---|---|---|
| 森林环境局部避障 | 高 | 场景包含 4000 个树木碰撞体 |
| 深度图感知算法验证 | 高 | 输出 16UC1 深度 payload 和相机内参 |
| 强化学习环境交互 | 高 | 支持 reset、step、状态和深度观测 |
| 路径规划闭环验证 | 高 | 支持外部客户端连续速度控制 |
| 真实飞控低层控制验证 | 低 | 当前运动模型不是刚体动力学模型 |
| 电机/姿态控制研究 | 低 | 未建模电机、桨叶、推力矩和气动效应 |

---

## 19. 推荐后续改进

| 方向 | 改进项 | 预期收益 |
|---|---|---|
| 协议稳定性 | 为 MessagePack 协议增加显式版本兼容检查 | 降低客户端升级风险 |
| 数据同步 | 状态帧和深度帧使用统一采样时间戳机制 | 便于传感器融合和训练数据对齐 |
| 性能优化 | 深度帧支持降采样、压缩或共享内存 | 降低 29.30 MiB/s 级别网络负载 |
| 可复现性 | 固定森林随机种子并导出场景配置 | 保证实验可重复 |
| 工程部署 | 去除硬编码路径，增加配置文件和命令行参数 | 便于多机器部署 |
| 单元测试 | 增加坐标转换、协议编解码、状态更新测试 | 降低维护成本 |
| 文档完善 | 增加 ROS 客户端接口文档和数据录制说明 | 便于算法侧接入 |

---

## 20. 引用说明

若该工程用于论文、课题报告或算法实验说明，建议在方法部分明确以下 4 点：

| 项 | 建议表述 |
|---|---|
| 仿真平台 | Unity `2022.3.62f2c1` |
| 场景规模 | 森林场景含 4000 个树木实例和 4000 个 MeshCollider |
| 状态频率 | 默认 `50 Hz`，由 `FixedUpdate` 和 `publishStride` 决定 |
| 深度数据 | `640 × 480`、`16UC1`，频率由渲染帧率和 GPU 读回决定 |

---

## 21. 版本边界

| 项 | 状态 |
|---|---|
| Unity Editor 编译 | 未在当前环境执行 |
| PlayMode 仿真 | 未在当前环境执行 |
| 外部 ROS 客户端 | 当前压缩包未包含 |
| Python 最小示例 | README 内给出接口级示例，未作为独立测试脚本运行 |
| 数据统计来源 | 当前工程文件静态读取 |
