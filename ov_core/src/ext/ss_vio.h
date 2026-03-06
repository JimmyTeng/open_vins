/*
 * Copyright (c) CompanyNameMagicTag 2024-2024. All rights reserved.
 * Description: vio alg const macros.
 * Author: Software Develop Team
 * Create: 2024-07-22
 */
#ifndef SSVIOEXT_H
#define SSVIOEXT_H

#include "stdbool.h"
#include "../system/ov_core_export.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OT_VIO_MAX_PATH_LEN 128
#define OT_VIO_IMU_DATA_NUM_MAX 1000
typedef struct {
    // char modelPath[OT_VIO_MAX_PATH_LEN];
    char calibParamPath[OT_VIO_MAX_PATH_LEN];
    bool logFlag;
    bool enableLoopClosure;
} OT_VIO_Param;

typedef enum {
    VIO_POSE_VALID_STATUS_NONE = 0,   /* Pose not calculated */
    VIO_POSE_VALID_STATUS_NOT_INIT = 1, /* Pose correctly calculated */
    VIO_POSE_VALID_STATUS_LOST = 2, /* Pose correctly calculated */
    VIO_POSE_VALID_STATUS_SUCCESS = 3, /* Pose correctly calculated */
} OT_VIO_PoseValidStatus;

typedef struct {
    double timestamp;
    double exposureDuration;
    unsigned char* virtAddr;
    unsigned int width;
    unsigned int height;
    unsigned int stride;
} OT_VIO_ImageData;

typedef struct {
    OT_VIO_ImageData leftImage;
} OT_VIO_CameraData;

typedef struct {
    double timestamp; /* second */
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
} OT_VIO_ImuData;

typedef struct {
    unsigned int num;
    OT_VIO_ImuData imuDatas[OT_VIO_IMU_DATA_NUM_MAX];
} OT_VIO_ImuDataInfo;

typedef struct {
    double timestamp;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
    float vx;
    float vy;
    float vz;
    float ox;
    float oy;
    float oz;
    OT_VIO_PoseValidStatus ValidFlag;
} OT_VIO_PoseData;

typedef struct {
    int ids;
    float u;  // 像素坐标系坐标点
    float v;
} OT_VIO_FeaturePoint;

typedef struct {
    double timestamp;
    int featureNum;
    OT_VIO_FeaturePoint* featureSets;
} OT_VIO_FeatureSets;

typedef struct {
    float x;
    float y;
    float z;
    int ids;
} OT_VIO_3DPoint;

typedef struct {
    double timestamp;
    int pointNum;
    OT_VIO_3DPoint* pointSets;
} OT_VIO_3DPointSets;

SS_VIO_API int SS_VIO_Init(const OT_VIO_Param* param);
SS_VIO_API int SS_VIO_GetData(const double timestamp, OT_VIO_PoseData* data);
SS_VIO_API int SS_VIO_DeInit();
SS_VIO_API int SS_VIO_PushImageData(const OT_VIO_CameraData* camData);
SS_VIO_API int SS_VIO_PushImuData(const OT_VIO_ImuDataInfo* imuDataInfo);

#ifdef __cplusplus
}
#endif

#endif