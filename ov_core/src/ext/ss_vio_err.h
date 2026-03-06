/*
 * Copyright (c) CompanyNameMagicTag 2024-2024. All rights reserved.
 * Description: vio tracker api module.
 * Author: Software Develop Team
 * Create: 2024-07-15
 */

#ifndef SS_VIO_ERR_H
#define SS_VIO_ERR_H

#define OT_VIOALG_ERR_INIT                  -2
#define OT_VIOALG_ERR_INIT_CALIBPARAMS      -3
#define OT_VIOALG_ERR_INIT_START            -4
#define OT_VIOALG_ERR_INIT_CHECK_MODELPATH  -5
#define OT_VIOALG_ERR_INIT_CHECK_CALIBPATH  -6
#define OT_VIOALG_ERR_INIT_NULL             -7
#define OT_VIOALG_ERR_DEINIT                -8
#define OT_VIOALG_ERR_PUSH_IMU_GYR          -9
#define OT_VIOALG_ERR_PUSH_IMU_ACC          -10
#define OT_VIOALG_ERR_IMU_NULL              -11
#define OT_VIOALG_ERR_IMU_TIME              -12

#define OT_VIOALG_ERR_IMAGE_NULL            -13
#define OT_VIOALG_ERR_IMAGE_HEIGHT          -14
#define OT_VIOALG_ERR_IMAGE_WIDTH           -15
#define OT_VIOALG_ERR_IMAGE_STRIDE          -16

#define OT_VIOALG_ERR_PUSH_IMAGE            -17
#define OT_VIOALG_ERR_GET_POSE              -18

#define OT_VIOALG_ERR_CALIB_RET_ERROR           -1
#define OT_VIOALG_ERR_CALIB_FILE_OPEN_ERROR     -19
#define OT_VIOALG_ERR_CALIB_FILE_SEEK_ERROR     -20
#define OT_VIOALG_ERR_CALIB_FILE_READ_ERROR     -21
#define OT_VIOALG_ERR_CALIB_MEMORY_MALLOC_ERROR -22
#define OT_VIOALG_ERR_GETDATA_NULL    -23
//#define OT_VIOALG_ERR_CALIB_INSERTED = 1,
//#define OT_VIOALG_ERR_CALIB_UPDATE = 2

#endif