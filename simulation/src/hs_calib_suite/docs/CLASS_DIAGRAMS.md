# 架构图与类图

本文给出 **UI 层**与 **标定算法层（core）** 的前期类设计，供 P1 实现对照。  
命名空间：`hs_calib::gui`、`hs_calib::core`、`hs_calib::ros_wrap`。  
实现语言：C++17；界面 Qt5；算法库不依赖 ROS。

相关文档：[ARCHITECTURE.md](ARCHITECTURE.md)、[UI_CONCEPT.md](UI_CONCEPT.md)、[TAXONOMY.md](TAXONOMY.md)。

---

## 1. 总架构图（进程与库）

```mermaid
flowchart LR
  subgraph procGui [进程_hs_calib_gui]
    UI[gui页面与控件]
    SC[SessionController]
  end

  subgraph procRos [进程_标定节点]
    Node[ros_wrap节点]
  end

  subgraph libCore [库_hs_calib_core]
    Core[core算法]
  end

  UI --> SC
  SC -->|离线直接调用| Core
  SC -->|在线_服务与话题| Node
  Node --> Core
```

```mermaid
flowchart TB
  subgraph layers [逻辑分层]
    L1[gui_表现与交互]
    L2[SessionController_会话与状态机]
    L3[RosBridge_或_OfflineLoader]
    L4[core_靶标_检测_求解_导出]
  end
  L1 --> L2 --> L3
  L2 --> L4
  L3 --> L4
```

---

## 2. UI 类图

### 2.1 页面与外壳

```mermaid
classDiagram
  direction TB

  class MainWindow {
    +showPage(PageId)
    -stack_ QStackedWidget
    -session_ SessionController
  }

  class IPage {
    <<interface>>
    +onEnter()
    +onLeave()
    +pageId() PageId
  }

  class HomePage {
    +selectedProject() string
    +selectedCalibratorId() string
  }

  class SetupPage {
    +loadConfig(path)
    +readinessOk() bool
  }

  class WorkbenchHost {
    +attachWorkbench(IWorkbenchPage)
  }

  class ReviewPage {
    +setResult(CalibrationResult)
    +exportYaml(path)
  }

  MainWindow --> IPage : 管理
  IPage <|.. HomePage
  IPage <|.. SetupPage
  IPage <|.. WorkbenchHost
  IPage <|.. ReviewPage
  MainWindow --> SessionController
```

### 2.2 工作台插件与通用控件

```mermaid
classDiagram
  direction TB

  class IWorkbenchPage {
    <<interface>>
    +calibratorId() string
    +onCapture()
    +onSolveRequested()
    +setLiveFrame(frame)
  }

  class IntrinsicsWorkbench
  class ExtrinsicWorkbench
  class HandEyeWorkbench
  class TrihedralWorkbench

  IWorkbenchPage <|.. IntrinsicsWorkbench
  IWorkbenchPage <|.. ExtrinsicWorkbench
  IWorkbenchPage <|.. HandEyeWorkbench
  IWorkbenchPage <|.. TrihedralWorkbench

  class ImageViewWidget {
    +setImage(QImage)
    +setOverlay(detections)
  }

  class ObservationListWidget {
    +addItem(ObservationSummary)
    +removeSelected()
  }

  class TfTreeWidget {
    +setTree(kind, transforms)
  }

  class MetricsPanel {
    +setMetrics(map)
  }

  class ParamForm {
    +bindYaml(path)
    +toMap() map
  }

  class ReadinessChecklist {
    +refresh()
    +allPassed() bool
  }

  IntrinsicsWorkbench --> ImageViewWidget
  IntrinsicsWorkbench --> ObservationListWidget
  ExtrinsicWorkbench --> ImageViewWidget
  ExtrinsicWorkbench --> TfTreeWidget
  SetupPage ..> ParamForm
  SetupPage ..> ReadinessChecklist
  ReviewPage ..> MetricsPanel
  ReviewPage ..> TfTreeWidget
```

### 2.3 会话控制（界面侧，不含求解）

```mermaid
classDiagram
  direction LR

  class SessionController {
    +setMode(Online|Offline)
    +configure(project, calibratorId, configPath)
    +startSession()
    +capture()
    +solve() CalibrationResult
    +state() SessionState
  }

  class RosBridge {
    +waitTopic(topic)
    +callCalibrate()
    +lookupTf(parent, child)
  }

  class OfflineLoader {
    +open_image_dir(path)
    +next_frame()
  }

  class CoreFacade {
    +detect(...)
    +calibrate(...) CalibrationResult
    +export(...)
  }

  SessionController --> RosBridge
  SessionController --> OfflineLoader
  SessionController --> CoreFacade
  CoreFacade --> CalibratorRegistry : 使用 core
```

**SessionState 建议枚举：** `Idle` → `Configuring` → `Ready` → `Collecting` → `Solving` → `Review` → `Exported`（失败回到 `Ready` 或 `Collecting`）。

---

## 3. 标定算法类图（`core/`）

### 3.1 靶标与检测

```mermaid
classDiagram
  direction TB

  class TargetModelBase {
    <<abstract>>
    +target_id() string
    +object_points(ids) MatrixXd
  }

  class ChessboardTarget
  class CharucoTarget
  class ArucoGridTarget
  class ArucoSingleTarget
  class TrihedralTarget {
    +plane_angles_deg() Vector3d
    +plane_models() vector~TargetModelBase~
  }

  TargetModelBase <|-- ChessboardTarget
  TargetModelBase <|-- CharucoTarget
  TargetModelBase <|-- ArucoGridTarget
  TargetModelBase <|-- ArucoSingleTarget
  TargetModelBase <|-- TrihedralTarget
  TrihedralTarget o-- TargetModelBase : 三面

  class DetectorBase {
    <<abstract>>
    +detect(frame, target) vector~Correspondence~
  }

  class ChessboardDetector
  class CharucoDetector
  class ArucoGridDetector
  class CircleGridDetector
  class TrihedralChessDetector
  class TrihedralCharucoDetector

  DetectorBase <|-- ChessboardDetector
  DetectorBase <|-- CharucoDetector
  DetectorBase <|-- ArucoGridDetector
  DetectorBase <|-- CircleGridDetector
  DetectorBase <|-- TrihedralChessDetector
  DetectorBase <|-- TrihedralCharucoDetector
  DetectorBase ..> TargetModelBase : 使用
```

### 3.2 观测与结果

```mermaid
classDiagram
  direction TB

  class Correspondence {
    +image_points
    +object_points
    +ids
  }

  class Observation {
    +timestamp_sec
    +frame_id
    +correspondences
  }

  class ObservationBatch {
    +items vector~Observation~
    +notes string
  }

  class CalibrationResult {
    +success bool
    +score float
    +message string
    +transforms
    +intrinsics_meta
    +metrics
  }

  class CalibratorInfo {
    +calibrator_id
    +display_name
    +category
    +required_frames
    +supported_targets
  }

  ObservationBatch o-- Observation
  Observation o-- Correspondence
```

### 3.3 标定器与注册表

```mermaid
classDiagram
  direction TB

  class CalibratorBase {
    <<abstract>>
    +calibrator_info() CalibratorInfo
    +calibrate(batch, config) CalibrationResult
  }

  class CamIntrinsicsCalibrator
  class EyeInHandCalibrator
  class EyeToHandCalibrator
  class StereoExtrinsicsCalibrator
  class TrihedralOneshotCalibrator
  class CamLidarPnPCalibrator

  note for CamIntrinsicsCalibrator
    已实现：chessboard → calibrateCamera
    注册 ID: cam_intrinsics
  end note
  note for EyeInHandCalibrator
    已实现：PnP + calibrateHandEye
    注册 ID: eye_in_hand
  end note
  note for EyeToHandCalibrator
    已实现：PnP + calibrateHandEye(逆位姿)
    注册 ID: eye_to_hand
  end note

  CalibratorBase <|-- CamIntrinsicsCalibrator
  CalibratorBase <|-- EyeInHandCalibrator
  CalibratorBase <|-- EyeToHandCalibrator
  CalibratorBase <|-- StereoExtrinsicsCalibrator
  CalibratorBase <|-- TrihedralOneshotCalibrator
  CalibratorBase <|-- CamLidarPnPCalibrator

  class CalibratorRegistry {
    +register_calibrator(id, factory)
    +create(id) unique_ptr~CalibratorBase~
    +list_ids() vector
  }

  CalibratorRegistry ..> CalibratorBase : 创建

  class ResultExporter {
    +to_camera_info_yaml(result, path)
    +to_extrinsics_yaml(result, path)
    +to_report(result, path)
  }

  class MetricsEvaluator {
    +reprojection_rmse(...)
    +epipolar_error(...)
  }

  CalibratorBase ..> ObservationBatch : 输入
  CalibratorBase ..> CalibrationResult : 输出
  ResultExporter ..> CalibrationResult
  MetricsEvaluator ..> CalibrationResult
```

### 3.4 典型求解依赖（内参示例）

```mermaid
flowchart LR
  Img[图像帧] --> Det[Detector]
  Tgt[TargetModel] --> Det
  Det --> Corr[Correspondence]
  Corr --> Batch[ObservationBatch]
  Batch --> Cal[CamIntrinsicsCalibrator]
  Cal --> Res[CalibrationResult]
  Res --> Exp[ResultExporter]
  Res --> Met[MetricsEvaluator]
```

---

## 4. ROS 薄封装类图（`ros/`）

```mermaid
classDiagram
  direction TB

  class PlaceholderCalibratorNode {
    +on_calibrate()
    +on_info()
  }

  class CalibratorNodeBase {
    <<abstract>>
    #load_params_from_config()
    #buffer_observation()
    #call_core()
  }

  CalibratorNodeBase <|-- PlaceholderCalibratorNode
  CalibratorNodeBase --> CalibratorRegistry : 仅调用
  CalibratorNodeBase ..> ObservationBatch : 组装后交给 core
```

节点 **不继承** 算法类；只持有配置、缓存观测，并调用 `CalibratorRegistry::create(...)->calibrate(...)`。

---

## 5. 目录落地（与实现对齐）

```text
include/hs_calib_suite/
  core/
    types.hpp
    interfaces.hpp
    registry.hpp
    calibrator_base.hpp
    detector_base.hpp
    target_model_base.hpp
    targets/       # chessboard / charuco / aruco_grid / circle_grid / trihedral
    detectors/     # 对应检测器 + aruco_dict
    calibrators/   # cam_intrinsics / trihedral_oneshot / eye_*
    io/            # board_config_yaml / board_pose / export_camera_yaml
    util/          # cv_bridge_local
  gui/
    ...
  ros/
    ...
```

`src/core/` 镜像上述子目录；冒烟工具放在 `src/core/tools/`。
