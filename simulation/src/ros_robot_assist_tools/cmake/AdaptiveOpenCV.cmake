# Adaptive OpenCV (same idea as hs_calib_suite):
# - Prefer CUDA OpenCV only when its exact toolkit is installed
# - Otherwise force system CPU OpenCV so find_package(OpenCV) does not
#   pick /usr/local OpenCV built against a missing CUDA minor version
#
# Call this BEFORE find_package(OpenCV).

set(_RRAT_CUDA_OPENCV_CANDIDATES
  "/usr/local/opencv4.10.0/lib/cmake/opencv4"
  "/usr/local/lib/cmake/opencv4"
)
set(_RRAT_SYS_OPENCV_CANDIDATES
  "/usr/lib/cmake/opencv4"
  "/usr/lib/aarch64-linux-gnu/cmake/opencv4"
  "/usr/lib/x86_64-linux-gnu/cmake/opencv4"
)

set(_RRAT_CUDA_OPENCV_DIR "")
set(_RRAT_OPENCV_CUDA_REQ "")
foreach(_dir IN LISTS _RRAT_CUDA_OPENCV_CANDIDATES)
  if(EXISTS "${_dir}/OpenCVConfig.cmake")
    file(STRINGS "${_dir}/OpenCVConfig.cmake" _rrat_ocv_cuda_line
      REGEX "^set\\(OpenCV_CUDA_VERSION")
    if(_rrat_ocv_cuda_line)
      string(REGEX REPLACE ".*\"([0-9]+\\.[0-9]+)\".*" "\\1"
        _RRAT_OPENCV_CUDA_REQ "${_rrat_ocv_cuda_line}")
      set(_RRAT_CUDA_OPENCV_DIR "${_dir}")
      break()
    endif()
  endif()
endforeach()

set(_RRAT_MATCHING_CUDA "")
if(_RRAT_OPENCV_CUDA_REQ AND EXISTS "/usr/local/cuda-${_RRAT_OPENCV_CUDA_REQ}")
  set(_RRAT_MATCHING_CUDA "/usr/local/cuda-${_RRAT_OPENCV_CUDA_REQ}")
elseif(_RRAT_OPENCV_CUDA_REQ AND EXISTS "/usr/local/cuda")
  # Accept /usr/local/cuda symlink if it resolves to the required toolkit
  get_filename_component(_rrat_cuda_real "/usr/local/cuda" REALPATH)
  if(_rrat_cuda_real MATCHES ".*/cuda-${_RRAT_OPENCV_CUDA_REQ}$")
    set(_RRAT_MATCHING_CUDA "/usr/local/cuda")
  endif()
endif()

set(_RRAT_SYS_OPENCV_DIR "")
foreach(_dir IN LISTS _RRAT_SYS_OPENCV_CANDIDATES)
  if(EXISTS "${_dir}/OpenCVConfig.cmake")
    set(_RRAT_SYS_OPENCV_DIR "${_dir}")
    break()
  endif()
endforeach()

if(_RRAT_MATCHING_CUDA AND _RRAT_CUDA_OPENCV_DIR)
  set(OpenCV_DIR "${_RRAT_CUDA_OPENCV_DIR}" CACHE PATH "OpenCV CMake dir" FORCE)
  set(CUDA_TOOLKIT_ROOT_DIR "${_RRAT_MATCHING_CUDA}" CACHE PATH "CUDA toolkit" FORCE)
  set(CUDA_HOME "${_RRAT_MATCHING_CUDA}" CACHE PATH "CUDA home" FORCE)
  set(ENV{CUDA_HOME} "${_RRAT_MATCHING_CUDA}")
  set(ENV{CUDA_PATH} "${_RRAT_MATCHING_CUDA}")
  set(ENV{PATH} "${_RRAT_MATCHING_CUDA}/bin:$ENV{PATH}")
  # FindCUDA honors CUDA_TOOLKIT_ROOT_DIR / CUDA_BIN_PATH
  set(ENV{CUDA_BIN_PATH} "${_RRAT_MATCHING_CUDA}")
  message(STATUS
    "ros_robot_assist_tools: CUDA ${_RRAT_OPENCV_CUDA_REQ} available → "
    "CUDA OpenCV (${OpenCV_DIR}) with ${CUDA_TOOLKIT_ROOT_DIR}")
elseif(_RRAT_SYS_OPENCV_DIR)
  set(OpenCV_DIR "${_RRAT_SYS_OPENCV_DIR}" CACHE PATH "OpenCV CMake dir" FORCE)
  # Clear stale CUDA hints that may point at a mismatched toolkit
  unset(CUDA_TOOLKIT_ROOT_DIR CACHE)
  unset(CUDA_HOME CACHE)
  message(STATUS
    "ros_robot_assist_tools: CUDA OpenCV needs ${_RRAT_OPENCV_CUDA_REQ} "
    "(unavailable/mismatched) → CPU OpenCV (${OpenCV_DIR})")
else()
  message(STATUS
    "ros_robot_assist_tools: OpenCV_DIR left to CMake default search")
endif()
