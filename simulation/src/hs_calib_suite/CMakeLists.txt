cmake_minimum_required(VERSION 3.8)
project(hs_calib_suite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# AUTOMOC only on gui target — avoid scanning rosidl generated sources
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC OFF)
set(CMAKE_AUTOUIC OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ---------------------------------------------------------------------------
# Vendored AprilRobotics/apriltag（Kalibr 同系 36h11 检测后端）
# ---------------------------------------------------------------------------
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(_hs_calib_saved_shared_libs ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
add_subdirectory(third_party/apriltag EXCLUDE_FROM_ALL)
set(BUILD_SHARED_LIBS ${_hs_calib_saved_shared_libs} CACHE BOOL "Build shared libraries" FORCE)
# apriltag 目录开了 -Werror，放宽以免 Jetson/gcc 细节失败
if(TARGET apriltag)
  target_compile_options(apriltag PRIVATE -Wno-error)
endif()

# ---------------------------------------------------------------------------
# Vendored Tier4 ceres_intrinsic_camera_calibrator (Apache-2.0)
# ---------------------------------------------------------------------------
find_package(Ceres REQUIRED)
add_subdirectory(third_party/ceres_intrinsic_camera_calibrator EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------------------
# Messages / services
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibrationResult.msg"
  "msg/CalibrationScore.msg"
  "srv/Calibrate.srv"
  "srv/GetCalibratorInfo.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
)

# ---------------------------------------------------------------------------
# core/ — ROS-free algorithm library
# ---------------------------------------------------------------------------
add_library(hs_calib_core SHARED
  src/core/registry/registry.cpp
  src/core/types/types.cpp
  # util/（与解码无关的通用图像工具）
  src/core/util/cv_image_ops.cpp
  src/core/util/camera_models.cpp
  # targets/
  src/core/targets/chessboard_target.cpp
  src/core/targets/charuco_target.cpp
  src/core/targets/aruco_grid_target.cpp
  src/core/targets/aprilgrid_target.cpp
  src/core/targets/circle_grid_target.cpp
  src/core/targets/trihedral_target.cpp
  # detectors/
  src/core/detectors/aruco_dict.cpp
  src/core/detectors/chessboard_detector.cpp
  src/core/detectors/charuco_detector.cpp
  src/core/detectors/aruco_grid_detector.cpp
  src/core/detectors/aprilgrid_detector.cpp
  src/core/detectors/aruco_marker_detector.cpp
  src/core/detectors/circle_grid_detect_impl.cpp
  src/core/detectors/symmetric_circle_grid_detector.cpp
  src/core/detectors/asymmetric_circle_grid_detector.cpp
  src/core/detectors/trihedral_chess_detector.cpp
  src/core/detectors/trihedral_charuco_detector.cpp
  src/core/detectors/board_type_identifier.cpp
  # calibrators/
  src/core/calibrators/cam_intrinsics_calibrator.cpp
  src/core/calibrators/intrinsics/intrinsics_profile.cpp
  src/core/calibrators/intrinsics/intrinsics_reprojection.cpp
  src/core/calibrators/intrinsics/intrinsics_pipeline.cpp
  src/core/calibrators/intrinsics/intrinsics_plot_statistics.cpp
  src/core/calibrators/intrinsics/intrinsics_capture_filter.cpp
  src/core/calibrators/intrinsics/intrinsics_collector_params.cpp
  src/core/calibrators/intrinsics/intrinsics_config_params.cpp
  src/core/calibrators/intrinsics/board_frame_metrics.cpp
  src/core/calibrators/intrinsics/intrinsics_data_collector.cpp
  src/core/calibrators/intrinsics/intrinsics_session_state.cpp
  src/core/calibrators/stereo_intrinsics_calibrator.cpp
  src/core/calibrators/stereo_extrinsics_calibrator.cpp
  src/core/calibrators/trihedral_oneshot_calibrator.cpp
  src/core/calibrators/eye_in_hand_calibrator.cpp
  src/core/calibrators/eye_to_hand_calibrator.cpp
  # io/
  src/core/io/board_pose.cpp
  src/core/io/board_config_yaml.cpp
  src/core/io/export_camera_yaml.cpp
  src/core/review/review_diagnostics.cpp
)
target_include_directories(hs_calib_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(hs_calib_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
target_link_libraries(hs_calib_core PRIVATE apriltag tier4_ceres_intrinsic)
target_compile_definitions(hs_calib_core PUBLIC HS_CALIB_HAS_CERES=1)
# omnidir (CMei) lives in opencv_ccalib; may be missing from OpenCV_LIBS depending on find_package
find_library(OPENCV_CCALIB_LIB NAMES opencv_ccalib opencv_ccalib.so.4.5d opencv_ccalib.so.4.5
  PATHS ${OpenCV_LIB_DIRS} /usr/lib/x86_64-linux-gnu /usr/local/lib)
if(OPENCV_CCALIB_LIB)
  target_link_libraries(hs_calib_core PUBLIC ${OPENCV_CCALIB_LIB})
endif()
ament_target_dependencies(hs_calib_core PUBLIC ament_index_cpp)
target_compile_features(hs_calib_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# ros/ — thin rclcpp nodes (I/O only)
# ---------------------------------------------------------------------------
add_executable(placeholder_calibrator
  src/ros/placeholder_calibrator_node.cpp
  src/ros/placeholder_calibrator_main.cpp
)
target_include_directories(placeholder_calibrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(placeholder_calibrator
  rclcpp ament_index_cpp sensor_msgs geometry_msgs std_msgs tf2_ros
)
target_link_libraries(placeholder_calibrator hs_calib_core)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(placeholder_calibrator "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# smoke — offline chessboard intrinsics (no GUI)
# ---------------------------------------------------------------------------
add_executable(smoke_cam_intrinsics
  src/core/tools/smoke_cam_intrinsics.cpp
)
target_include_directories(smoke_cam_intrinsics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(smoke_cam_intrinsics hs_calib_core ${OpenCV_LIBS})

add_executable(smoke_handeye
  src/core/tools/smoke_handeye.cpp
)
target_include_directories(smoke_handeye PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(smoke_handeye hs_calib_core ${OpenCV_LIBS})

add_executable(smoke_trihedral_chess
  src/core/tools/smoke_trihedral_chess.cpp
)
target_include_directories(smoke_trihedral_chess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(smoke_trihedral_chess hs_calib_core ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
# gui/ — Qt5 Manager (no solvers)
# ---------------------------------------------------------------------------
add_executable(hs_calib_gui
  src/gui/app/main.cpp
  src/gui/window/main_window.cpp
  src/gui/window/main_window_pages.cpp
  src/gui/window/main_window_workbench.cpp
  src/gui/window/main_window_detect_lab.cpp
  src/gui/theme/app_style.cpp
  src/gui/log/app_logger.cpp
  src/gui/widgets/image_view_widget.cpp
  src/gui/widgets/review_charts_widget.cpp
  src/gui/panels/launcher_config_panel.cpp
  src/gui/intrinsics/intrinsics_workbench_panels.cpp
  src/gui/intrinsics/intrinsics_dialog_ui.cpp
  src/gui/intrinsics/intrinsics_param_tooltips.cpp
  src/gui/intrinsics/intrinsics_preview_overlay.cpp
  src/gui/intrinsics/intrinsics_parameter_dialog.cpp
  src/gui/intrinsics/intrinsics_matplotlib_plotter.cpp
  src/gui/plotting/intrinsics_plot_renderer.cpp
  src/gui/plotting/intrinsics_plot_session.cpp
  src/gui/plotting/intrinsics_async_plot_controller.cpp
  src/gui/plotting/intrinsics_plot_export.cpp
  src/gui/plotting/intrinsics_calibration_bars_dialog.cpp
  src/gui/plotting/intrinsics_calibration_rms_dialog.cpp
  src/gui/session/session_controller.cpp
  src/gui/bridges/ros_image_bridge.cpp
  src/gui/bridges/bag_image_loader.cpp
  src/gui/bridges/ros_bag_frame_reader.cpp
  src/gui/bridges/tf_pose_bridge.cpp
  src/gui/data/pose_csv_store.cpp
  src/gui/projects/project_catalog.cpp
  src/gui/projects/project_workspace.cpp
  include/hs_calib_suite/gui/window/main_window.hpp
  include/hs_calib_suite/gui/theme/app_style.hpp
  include/hs_calib_suite/gui/log/app_logger.hpp
  include/hs_calib_suite/gui/widgets/image_view_widget.hpp
  include/hs_calib_suite/gui/widgets/review_charts_widget.hpp
  include/hs_calib_suite/gui/intrinsics/intrinsics_workbench_panels.hpp
  include/hs_calib_suite/gui/intrinsics/intrinsics_dialog_ui.hpp
  include/hs_calib_suite/gui/intrinsics/intrinsics_param_tooltips.hpp
  include/hs_calib_suite/gui/intrinsics/intrinsics_preview_overlay.hpp
  include/hs_calib_suite/gui/intrinsics/intrinsics_parameter_dialog.hpp
  include/hs_calib_suite/gui/intrinsics/intrinsics_matplotlib_plotter.hpp
  include/hs_calib_suite/gui/plotting/intrinsics_plot_renderer.hpp
  include/hs_calib_suite/gui/plotting/intrinsics_async_plot_controller.hpp
  include/hs_calib_suite/gui/plotting/intrinsics_plot_export.hpp
  include/hs_calib_suite/gui/plotting/intrinsics_calibration_bars_dialog.hpp
  include/hs_calib_suite/gui/plotting/intrinsics_calibration_rms_dialog.hpp
  include/hs_calib_suite/gui/panels/launcher_config_panel.hpp
  include/hs_calib_suite/gui/session/session_controller.hpp
  include/hs_calib_suite/gui/bridges/ros_image_bridge.hpp
  include/hs_calib_suite/gui/bridges/bag_image_loader.hpp
  include/hs_calib_suite/gui/bridges/ros_bag_frame_reader.hpp
  include/hs_calib_suite/gui/bridges/tf_pose_bridge.hpp
  include/hs_calib_suite/gui/data/pose_csv_store.hpp
  include/hs_calib_suite/gui/projects/project_catalog.hpp
  include/hs_calib_suite/gui/projects/project_workspace.hpp
)
set_target_properties(hs_calib_gui PROPERTIES
  AUTOMOC ON
  AUTORCC ON
  AUTOUIC ON
)
target_include_directories(hs_calib_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_include_directories(hs_calib_gui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/window
)
ament_target_dependencies(hs_calib_gui
  rclcpp sensor_msgs cv_bridge tf2_ros ament_index_cpp
  rosbag2_cpp rosbag2_storage
)
target_link_libraries(hs_calib_gui
  Qt5::Widgets
  hs_calib_core
  ${OpenCV_LIBS}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS hs_calib_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS placeholder_calibrator hs_calib_gui smoke_cam_intrinsics smoke_handeye
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config docs scripts
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hs_calib_core)
ament_export_dependencies(
  Eigen3 OpenCV rclcpp geometry_msgs std_msgs sensor_msgs tf2_ros
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
