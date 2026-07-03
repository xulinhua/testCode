# Populates WORKBENCH_SOURCES and WORKBENCH_LINK_LIBS / deps usage via parent scope.

set(WORKBENCH_SOURCES
  src/preferences/app_preferences.cpp
  src/ros_robot_workbench_ui.cpp
  src/ui/lazy_feature_page.cpp
  src/ui/preferences_dialog.cpp
  src/ui/shared_refresh_pool.cpp
  src/ui/shared_ui_executor.cpp
  src/ui/system_status_widget.cpp
  src/module/system_status_module.cpp
  src/manage/system_status_data_manager.cpp
  src/module/calibration_module.cpp
  src/ui/workbench_module_registry.cpp
  src/manage/feature_data_manager_base.cpp
  src/manage/feature_data_manager_hub.cpp
)

macro(workbench_add_module name)
  list(APPEND WORKBENCH_SOURCES
    src/module/${name}_module.cpp
    src/ui/${name}_widget.cpp
    src/manage/${name}_data_manager.cpp)
endmacro()

if(WORKBENCH_KIT_GENERAL)
  list(APPEND WORKBENCH_SOURCES
    src/ui/image_viewer_widget.cpp
    src/module/image_viewer_module.cpp
    src/ui/zoomable_image_widget.cpp
    src/manage/image_viewer_data_manager.cpp)
  workbench_add_module(rosbag_workbench)
  workbench_add_module(topic_lab)
endif()

if(WORKBENCH_KIT_KINEMATICS)
  workbench_add_module(pose_transform)
  workbench_add_module(kinematics_solver)
  workbench_add_module(tf_viewer)
  list(APPEND WORKBENCH_SOURCES
    src/kinematics/mdh_craig1989.cpp
    src/kinematics/mobile_kinematics.cpp)
  if(WORKBENCH_WITH_KDL)
    list(APPEND WORKBENCH_SOURCES src/kinematics/arm_kinematics_kdl.cpp)
  endif()
  if(WORKBENCH_WITH_MOVEIT)
    list(APPEND WORKBENCH_SOURCES src/kinematics/arm_kinematics_moveit.cpp)
  endif()
endif()

if(WORKBENCH_KIT_CALIBRATION)
  if(WORKBENCH_WITH_OPENCV)
    workbench_add_module(board_generator)
  endif()
  workbench_add_module(intrinsic_calibration)
  workbench_add_module(stereo_calibration)
  workbench_add_module(multi_sensor_calibration)
  if(WORKBENCH_WITH_OPENCV)
    workbench_add_module(handeye_calibration)
  endif()
  workbench_add_module(tcp_calibration)
endif()

if(WORKBENCH_KIT_ARM)
  workbench_add_module(joint_monitor)
  if(WORKBENCH_WITH_MOVEIT)
    workbench_add_module(moveit_debug)
  endif()
  workbench_add_module(grasp_pose_gen)
  workbench_add_module(multi_tcp_manager)
endif()

if(WORKBENCH_KIT_MOBILE)
  workbench_add_module(odometry_analyzer)
  workbench_add_module(wheel_calib)
  workbench_add_module(nav2_panel)
endif()

if(WORKBENCH_KIT_LEGGED)
  workbench_add_module(foot_contact_monitor)
  workbench_add_module(legged_imu_panel)
  workbench_add_module(rl_policy_monitor)
endif()

if(WORKBENCH_KIT_HUMANOID)
  workbench_add_module(humanoid_joint_monitor)
  workbench_add_module(balance_panel)
endif()

if(WORKBENCH_KIT_PLANNING)
  workbench_add_module(path_compare)
  workbench_add_module(obstacle_editor)
endif()

if(WORKBENCH_KIT_PERCEPTION3D)
  workbench_add_module(pointcloud_viewer)
  workbench_add_module(depth_analyzer)
  workbench_add_module(lidar_cam_projection)
endif()

if(WORKBENCH_KIT_SIMULATION)
  workbench_add_module(sim_control_panel)
  workbench_add_module(sim_time_monitor)
  workbench_add_module(sim2real_compare)
  workbench_add_module(usd_converter)
endif()

if(WORKBENCH_KIT_DL)
  workbench_add_module(inference_monitor)
  workbench_add_module(detection_overlay)
endif()
