# 与 TIER IV CalibrationTools 的对比

参考仓库：[tier4/CalibrationTools](https://github.com/tier4/CalibrationTools)（分支 `tier4/universe`）。  
本工程只借鉴其**分层与工作流**，不复制其代码，也不绑定 Autoware 完整环境。

---

## 1. 设计上对齐的部分

| 方面 | TIER IV | 本工程 |
|------|---------|--------|
| 管理界面 | `sensor_calibration_manager` | `gui/`（可执行文件 `hs_calib_gui`） |
| 标定器与具体车型解耦 | 服务 + launch 参数 | `Calibrate.srv` + `config/*.yaml` |
| 项目与标定器组合 | 注册表 + 各项目 launch | 注册表 + 各项目配置文件（优先 `ros2 run`） |
| 结果坐标系后处理 | optical ↔ link、sensor_kit 等 | 计划中的 `ResultPostProcessor`（P2） |
| 质量反馈 | score、message | `CalibrationResult` 与 `CalibrationScore[]` |

---

## 2. 能力差异与本工程计划

| 能力 | TIER IV | 本工程计划 |
|------|---------|------------|
| 相机–激光外参（标记 / 交互选点 / 建图） | 较强 | P2 起实现同类能力 |
| 激光–激光、地面约束、雷达相关 | 有现成工具 | P3 |
| 相机内参流程与界面 | 有，但相对简单 | **已对齐 Tier4 工作台 + 三张 matplotlib 统计图**（见 [`TIER4_INTRINSICS_STATS.md`](TIER4_INTRINSICS_STATS.md)） |
| 双目标定与校正 | 不是其主线 | **P1 作为完整功能支持** |
| 直角三面单帧标定 | 无 | **P1 作为本工程特色** |
| 机械臂手眼标定 | 基本没有 | **P2 自行实现** |
| 算法与 ROS 分离 | 部分逻辑写在节点中 | **`core/` 强制不依赖 ROS** |
| 无 ROS 离线复算 | 较弱 | 界面或命令行直接链接 `core` 库 |
| 实现语言 | C++ 与 Python 混用 | **算法 C++17；统计图渲染用 vendored Python 脚本（无业务 Python）** |
| 包组织 | 多个功能包 | **单一功能包 + 目录分层** |

---

## 3. 明确不做的事

- 不把完整 Autoware 或 `individual_params` 作为编译前提  
- 不复制其 GPL 代码；只对齐架构与操作流程  
- 不做「只能调用 TIER IV 可执行文件」的空壳界面  

---

## 4. 服务结果字段对照

TIER IV 的标定结果与本工程 `hs_calib_suite/CalibrationResult` 大致对应：

- `geometry_msgs/TransformStamped transform_stamped`
- `bool success`
- `float32 score`
- `string message`
- 扩展：`CalibrationScore[] scores`（多项指标）

内参结果可走同一服务的扩展字段，或同时写出内参文件；具体在 P1 定稿。

---

## 5. 小结

> 学习其「管理界面 + 可插拔标定器 + 坐标系后处理」；补齐内参、双目、直角三面、手眼与离线能力；坚持算法库不依赖 ROS。
