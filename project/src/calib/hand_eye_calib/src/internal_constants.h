/**
 * @file internal_constants.h
 * @brief 项目内部使用的常量定义
 * 
 * 该文件定义了项目内部使用的各种常量，仅在当前项目范围内生效
 * 此文件为内部使用，不会被导出到其他项目
 */

#ifndef HAND_EYE_CALIB__INTERNAL_CONSTANTS_H_
#define HAND_EYE_CALIB__INTERNAL_CONSTANTS_H_
#include <string>

#define  CAM_TO_BASE_NAME          "camera2base"  // 相机到基座矩阵名称
#define  BASE_TO_CAM_NAME          "base2camera"  // 基座到相机矩阵名称
#define  MATRIX_NAME               "matrix"       // 矩阵名称
#define  HEAD_MOTOR_ANGLES_NAME    "head_motor_angles"  // 头部电机角度名称
#define  QUALITY_METRICS_NAME      "quality_metrics"  // 质量评估指标名称

#endif  // HAND_EYE_CALIB__INTERNAL_CONSTANTS_H_