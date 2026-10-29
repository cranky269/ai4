# 函数接口调用关系图（Flowchart 版）

> 依据《无人机项目实施拆解文档》绘制，使用 Mermaid `flowchart`。
>
> 图例约定（**分图说明——图一与图三的线型语义不同，勿混看**）：
> - **图一（模块间全局图）**：实线箭头 `-->` = ROS 话题数据流；虚线箭头 `-.->` = 非 ROS 话题的物理仿真关联（仅 `/target_sim → /radar_sim`）
> - **图二（模块内部图）**：实线箭头 = 函数调用
> - **图三（总图）**：模块**内**实线 = 函数调用；模块**间**虚线 = ROS 话题数据流（⚠️ 与图一虚线的含义不同）
> - **箭头标签**：话题名 或 传递的数据
> - **着色**：图一按支路着色、图三按模块着色，颜色对照见文末表（图一配色见其 `classDef` 注释）
> - **省略说明**：图一为清晰起见，省略了 debug 话题（`/target/pose`、`/gazebo/model_states`），仅保留核心控制闭环数据流（`/mavros/local_position/pose` 为 navigator 悬停的功能输入、属核心闭环，已画出）。

---

## 一、模块间接口关系图（全局）

> 展示 12 个 ROS 节点（另加 1 个「飞控 PX4 SITL」执行体，非 ROS 节点）的订阅/发布串接关系，即整个系统的数据流闭环。

```mermaid
%%{init: {"flowchart": {"nodeSpacing": 40, "rankSpacing": 70, "padding": 25}} }%%
flowchart TB
    subgraph S1["感知 · 雷达支路"]
        direction LR
        target_sim["/target_sim<br/>敌机飞行脚本"]:::cRadar
        radar_sim["/radar_sim<br/>机载 3D Lidar + 噪声<br/>（输出世界坐标）"]:::cRadar
        cluster["/cluster<br/>滤地 + DBSCAN 聚类"]:::cRadar
    end
    subgraph S2["感知 · 视觉支路"]
        direction LR
        camera["/camera_node<br/>相机驱动"]:::cVision
        detector["/detector<br/>YOLOv8/v11 检测"]:::cVision
        vt["/visual_tracker<br/>BoT-SORT 跟踪"]:::cVision
    end
    subgraph S3["制导"]
        direction LR
        tracker["/tracker<br/>航迹关联 + 卡尔曼"]:::cGuide
        navigator["/navigator<br/>航迹→航点解算"]:::cGuide
        pid["/pid_controller<br/>视觉伺服 PID"]:::cGuide
    end
    subgraph S4["决策 · 切换"]
        direction LR
        sm["/state_machine<br/>雷达/视觉切换 + 熔断"]:::cDecide
    end
    subgraph S5["控制"]
        direction LR
        mux["/control_mux<br/>仲裁 · 唯一出口"]:::cControl
    end
    subgraph S6["执行"]
        direction LR
        mavros["/mavros<br/>MAVLink 桥接"]:::cExec
        fcs["飞控<br/>PX4 SITL"]:::cFcs
    end

    %% 雷达主线
    target_sim -.->|"驱动物理模型"| radar_sim
    radar_sim -->|"/radar/points"| cluster
    cluster -->|"/radar/centroid"| tracker
    tracker -->|"/radar/tracks"| navigator

    %% 视觉主线
    camera -->|"/camera/image_raw"| detector
    detector -->|"/detections"| vt
    vt -->|"/visual/tracks"| pid

    %% 状态机订阅（决策输入）
    tracker -->|"/radar/tracks"| sm
    detector -->|"/detections"| sm
    vt -->|"/visual/tracks"| sm

    %% 制导/决策 → 控制
    navigator -->|"/ctrl/radar"| mux
    pid -->|"/ctrl/visual"| mux
    sm -->|"/state"| mux

    %% 执行
    mux -->|"/mavros/setpoint_raw/local"| mavros
    mavros -->|"MAVLink"| fcs

    %% 位姿回传（navigator 悬停的功能输入，非 debug）
    mavros -->|"/mavros/local_position/pose"| navigator

    %% 配色（与图三风格一致；飞控 fcs 用虚线边框表示非 ROS 节点）
    classDef cRadar   fill:#e3f2fd,stroke:#1565c0,color:#0d47a1
    classDef cVision  fill:#f3e5f5,stroke:#7b1fa2,color:#4a148c
    classDef cGuide   fill:#fff3e0,stroke:#ef6c00,color:#e65100
    classDef cDecide  fill:#fff8e1,stroke:#f9a825,color:#f57f17
    classDef cControl fill:#ffebee,stroke:#c62828,color:#b71c1c
    classDef cExec    fill:#e0f7fa,stroke:#00838f,color:#006064
    classDef cFcs     fill:#eceff1,stroke:#455a64,color:#263238,stroke-dasharray:5 5
```

> 图一连线标签只标话题名，各话题的消息类型见《数据类型.md》第五章。

---

## 二、各模块内部函数调用关系图

### 模块 1：点迹聚类（`/cluster`）

```mermaid
flowchart LR
    A["filter_ground<br/>滤地杂波"] -->|"点集"| B["dbscan_cluster<br/>DBSCAN 聚类取质心"]
```

### 模块 2：航迹关联 + 卡尔曼滤波（`/tracker`）

```mermaid
flowchart LR
    A["predict<br/>状态预测"] -->|"预测状态"| B["associate<br/>航迹关联"] -->|"关联对"| C["update<br/>量测更新"]
```

### 模块 3：导航（`/navigator`）

```mermaid
flowchart LR
    T["on_timer<br/>时钟层总领 · 三选一"] -->|"有航迹"| A["waypoints_from_track<br/>航迹→航点"]
    T -->|"短时无（未超时）"| R["重发上一条<br/>last_cmd"]
    T -->|"长时无（超时）"| C["hover_setpoint<br/>位置保持（自判）"]
    A -->|"Waypoint"| B["send_waypoint<br/>航点→位置指令"]
    B -->|"PositionCommand"| P["发布 /ctrl/radar"]
    R -->|"PositionCommand（on_timer 直接发布，不经 send_waypoint）"| P
    C -->|"PositionCommand（on_timer 直接发布，不经 send_waypoint）"| P
```

### 模块 4：视觉检测（`/detector`）

```mermaid
flowchart LR
    A["detect<br/>YOLO 检测"]
```

### 模块 5：视觉跟踪（`/visual_tracker`）

```mermaid
flowchart LR
    A["track<br/>BoT-SORT 跟踪"]
```

### 模块 6：PID 控制（`/pid_controller`）

```mermaid
flowchart LR
    A["__init__<br/>初始化 PID 参数"] -->|"初始化"| T["on_timer<br/>时钟层总领 · 三选一"]
    T -->|"有 track"| B["compute<br/>三通道计算"]
    T -->|"短时无（未超时）"| R["重发上一条"]
    T -->|"长时无（超时）"| C["stop_setpoint<br/>零速度默认"]
```

### 模块 7：控制仲裁（`/control_mux`）

```mermaid
flowchart LR
    A["select_channel<br/>状态→通道仲裁"]
```

### 模块 8：状态机（`/state_machine`）

```mermaid
flowchart LR
    A["evaluate_visual_lock<br/>视觉锁定判定"] -->|"visual_lock"| C["transition<br/>状态转移"]
    B["evaluate_radar_lost<br/>雷达丢失判定"] -->|"radar_lost"| C
```

---

## 三、全部函数接口总图（按模块着色）

> 一张图包含全部 19 个函数，模块内实线为函数调用，模块间虚线为 ROS 话题数据流；每个模块使用独立配色。
> 另含 3 个外部节点（`Gazebo` 数据源、`mavros` 执行端、飞控 `PX4 SITL`），用灰色虚线边框区分；其中 `mavros ==> 飞控` 为 MAVLink 协议连接（非 ROS 话题，粗箭头）。

```mermaid
flowchart TB
    subgraph M1["① 点迹聚类 /cluster"]
        direction LR
        f1["filter_ground"]:::c1 --> f2["dbscan_cluster"]:::c1
    end
    subgraph M2["② 航迹关联 + 卡尔曼 /tracker"]
        direction LR
        f3["predict"]:::c2 --> f4["associate"]:::c2 --> f5["update"]:::c2
    end
    subgraph M3["③ 导航 /navigator"]
        direction LR
        f18["on_timer"]:::c3 --> f6["waypoints_from_track"]:::c3 --> f7["send_waypoint"]:::c3
        f18 --> f16["hover_setpoint"]:::c3
    end
    %% 注：hover_setpoint / 重发 last_cmd 的结果为 PositionCommand，由 on_timer 直接发布，
    %% 不经 send_waypoint（其入参为 Waypoint，类型不符）——与《函数接口》3.4 一致。
    %% PositionCommand 为纯类型，发布前由节点壳转换为 mavros_msgs/PositionTarget。
    subgraph M4["④ 视觉检测 /detector"]
        f8["detect"]:::c4
    end
    subgraph M5["⑤ 视觉跟踪 /visual_tracker"]
        f9["track"]:::c5
    end
    subgraph M6["⑥ PID 控制 /pid_controller"]
        direction LR
        f10["__init__"]:::c6 --> f19["on_timer"]:::c6 --> f11["compute"]:::c6
        f19 --> f17["stop_setpoint"]:::c6
    end
    subgraph M8["⑧ 状态机 /state_machine"]
        direction LR
        f13["evaluate_visual_lock"]:::c8 --> f15["transition"]:::c8
        f14["evaluate_radar_lost"]:::c8 --> f15["transition"]:::c8
    end
    subgraph M7["⑦ 控制仲裁 /control_mux"]
        f12["select_channel"]:::c7
    end

    %% 模块间 ROS 话题数据流
    M1 -.->|"/radar/centroid"| M2
    M2 -.->|"/radar/tracks"| M3
    M4 -.->|"/detections"| M5
    M5 -.->|"/visual/tracks"| M6
    M2 -.->|"/radar/tracks"| M8
    M4 -.->|"/detections"| M8
    M5 -.->|"/visual/tracks"| M8
    M3 -.->|"/ctrl/radar"| M7
    M6 -.->|"/ctrl/visual"| M7
    M8 -.->|"/state"| M7

    %% 外部模块（数据源 / 执行端，非函数模块，灰色虚线边框区分，统一编号 ⑨）
    gazebo["⑨ Gazebo 仿真环境<br/>雷达点云 + 相机图像数据源"]:::ext
    mavros["⑨ /mavros<br/>MAVLink 桥接 · 执行端"]:::ext
    fcs["⑨ 飞控<br/>PX4 SITL"]:::ext

    %% 数据源 → 感知
    gazebo -.->|"/radar/points"| M1
    gazebo -.->|"/camera/image_raw"| M4

    %% 控制 → 执行端
    M7 -.->|"/mavros/setpoint_raw/local"| mavros
    mavros -.->|"/mavros/local_position/pose"| M3

    %% 执行端 → 飞控（MAVLink 协议，非 ROS 话题，用粗箭头区分）
    mavros ==>|"MAVLink"| fcs

    classDef c1 fill:#e3f2fd,stroke:#1565c0,color:#0d47a1
    classDef c2 fill:#e8f5e9,stroke:#2e7d32,color:#1b5e20
    classDef c3 fill:#fff3e0,stroke:#ef6c00,color:#e65100
    classDef c4 fill:#f3e5f5,stroke:#7b1fa2,color:#4a148c
    classDef c5 fill:#fce4ec,stroke:#c2185b,color:#880e4f
    classDef c6 fill:#e0f7fa,stroke:#00838f,color:#006064
    classDef c7 fill:#ffebee,stroke:#c62828,color:#b71c1c
    classDef c8 fill:#fff8e1,stroke:#f9a825,color:#f57f17
    classDef ext fill:#fafafa,stroke:#616161,color:#212121,stroke-dasharray:5 5
```

### 模块着色对照表

| 颜色类 | 模块 | 填充色 | 描边色 |
|---|---|---|---|
| `c1` | ① 点迹聚类 | `#e3f2fd` | `#1565c0` |
| `c2` | ② 航迹关联 + 卡尔曼 | `#e8f5e9` | `#2e7d32` |
| `c3` | ③ 导航 | `#fff3e0` | `#ef6c00` |
| `c4` | ④ 视觉检测 | `#f3e5f5` | `#7b1fa2` |
| `c5` | ⑤ 视觉跟踪 | `#fce4ec` | `#c2185b` |
| `c6` | ⑥ PID 控制 | `#e0f7fa` | `#00838f` |
| `c7` | ⑦ 控制仲裁 | `#ffebee` | `#c62828` |
| `c8` | ⑧ 状态机 | `#fff8e1` | `#f9a825` |
| `ext` | ⑨ 外部（Gazebo / mavros / 飞控） | `#fafafa` | `#616161` |
