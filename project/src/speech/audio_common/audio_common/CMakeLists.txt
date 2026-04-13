cmake_minimum_required(VERSION 3.8)
project(audio_common)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(custom_msgs_comm REQUIRED)  # 添加custom_msgs_comm依赖
find_library(PORTAUDIO_LIB portaudio REQUIRED)

# 查找espeak-ng库（可选）
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ESPEAK_NG QUIET espeak-ng)
endif()

# 如果找不到pkg-config中的espeak-ng，尝试手动查找
if(NOT ESPEAK_NG_FOUND)
  find_path(ESPEAK_NG_INCLUDE_DIRS NAMES espeak-ng/speak_lib.h PATHS /usr/include /usr/local/include)
  find_library(ESPEAK_NG_LIBRARIES NAMES espeak-ng libespeak-ng PATHS /usr/lib /usr/local/lib)
  if(ESPEAK_NG_INCLUDE_DIRS AND ESPEAK_NG_LIBRARIES)
    set(ESPEAK_NG_FOUND TRUE)
    message(STATUS "Found espeak-ng: ${ESPEAK_NG_LIBRARIES}")
  else()
    message(WARNING "espeak-ng not found, will use command line version")
  endif()
endif()

# 尝试查找WebRTC VAD库
find_package(PkgConfig REQUIRED)
pkg_check_modules(WEBRTC_VAD webrtc-audio-processing)

# 查找Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})

# 定义依赖项
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_srvs
  audio_common_msgs
  custom_msgs_comm  # 添加custom_msgs_comm依赖
  ament_index_cpp
)

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIR}
  ../../cmd_dispatcher/src/cmd_dispatcher/include  # 添加cmd_dispatcher的包含目录
)

# 如果找到了espeak-ng库，则添加相应的编译定义
if(ESPEAK_NG_FOUND)
  add_definitions(-DHAVE_ESPEAK_NG)
  include_directories(${ESPEAK_NG_INCLUDE_DIRS})
endif()

# 如果找到了WebRTC VAD库，则添加相应的编译定义
if(WEBRTC_VAD_FOUND)
  add_definitions(-DHAVE_WEBRTC_VAD)
  include_directories(${WEBRTC_VAD_INCLUDE_DIRS})
endif()

# 音频采集节点
add_executable(audio_capturer_node
  src/audio_common/audio_capturer_node.cpp
  src/audio_common/audio_device_manager.cpp
  src/audio_common/wake_word_detector.cpp
  src/audio_capturer_main.cpp
)
ament_target_dependencies(audio_capturer_node rclcpp audio_common_msgs custom_msgs_comm ament_index_cpp)
target_link_libraries(audio_capturer_node ${PORTAUDIO_LIB} ${Python3_LIBRARIES} ${LIB})
install(TARGETS
  audio_capturer_node
  DESTINATION lib/${PROJECT_NAME}
)

# VAD测试程序
add_executable(test_vad
  test_vad.cpp
  src/audio_common/audio_capturer_node.cpp
  src/audio_common/audio_device_manager.cpp
  src/audio_common/wake_word_detector.cpp
)
ament_target_dependencies(test_vad rclcpp audio_common_msgs custom_msgs_comm ament_index_cpp)
target_link_libraries(test_vad ${PORTAUDIO_LIB} ${Python3_LIBRARIES} ${LIB})
install(TARGETS
  test_vad
  DESTINATION lib/${PROJECT_NAME}
)

# 音频播放节点
add_executable(audio_player_node
  src/audio_common/audio_player_node.cpp
  src/audio_common/audio_device_manager.cpp
  src/audio_player_main.cpp
)
ament_target_dependencies(audio_player_node rclcpp audio_common_msgs custom_msgs_comm ament_index_cpp)
target_link_libraries(audio_player_node ${PORTAUDIO_LIB} ${LIB})
install(TARGETS
  audio_player_node
  DESTINATION lib/${PROJECT_NAME}
)

# 音乐播放节点
add_executable(music_node
  src/audio_common/wave_file.cpp
  src/audio_common/music_node.cpp
  src/music_main.cpp
)
ament_target_dependencies(music_node rclcpp std_srvs audio_common_msgs custom_msgs_comm ament_index_cpp)
target_link_libraries(music_node ${PORTAUDIO_LIB})
install(TARGETS
  music_node
  DESTINATION lib/${PROJECT_NAME}
)

# 文本转语音节点
add_executable(tts_node
  src/audio_common/wave_file.cpp
  src/audio_common/tts_node.cpp
  src/tts_main.cpp
)
ament_target_dependencies(tts_node rclcpp rclcpp_action audio_common_msgs custom_msgs_comm ament_index_cpp)
if(ESPEAK_NG_FOUND)
  target_link_libraries(tts_node ${PORTAUDIO_LIB} ${ESPEAK_NG_LIBRARIES})
else()
  target_link_libraries(tts_node ${PORTAUDIO_LIB})
endif()
install(TARGETS
  tts_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  samples
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  ../config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 注意：launch目录已被移除，此处不再安装

# 安装测试脚本
install(DIRECTORY
  test/
  DESTINATION share/${PROJECT_NAME}/test
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# 创建库
set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/audio_common/wave_file.cpp
  src/audio_common/wake_word_detector.cpp
  src/audio_common/audio_device_manager.cpp
)

# 创建动态库而不是静态库
add_library(${LIB} SHARED ${SOURCES})
# 添加符号导出定义
target_compile_definitions(${LIB} PRIVATE "AUDIO_COMMON_LIB_EXPORT")
ament_target_dependencies(${LIB} ${DEPENDENCIES})
target_link_libraries(${LIB} ${PORTAUDIO_LIB})

install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()