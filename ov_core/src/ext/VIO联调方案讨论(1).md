# 本文档讨论VIO联调事项

# 硬件
    暂定:19A+VIO模组(硬件已经确定能正常跑海思VIO算法。I/O数据正确)
    
# 环境(参考下面虚拟机)
    需下载编译工程
    https://codeup.aliyun.com/68bfd93c9eda9d4e3ee54201/w2q4-hi3519a/w2q4-hi3519a-mediaserver.git
    需要编译

# 方案：(参考下面虚拟机)
    替换海思VIO库，换成我们自研的VIO库,可以修改数据结构
    海思库接口如下:(参考附件.c)
    int SS_VIO_Init(const OT_VIO_Param* param);
    int SS_VIO_GetData(const double timestamp, OT_VIO_PoseData* data);
    int SS_VIO_DeInit();
    int SS_VIO_PushImageData(const OT_VIO_CameraData* camData);
    int SS_VIO_PushImuData(const OT_VIO_ImuDataInfo* imuDataInfo);
    调用参考
    喂图像
    static td_s32 VIOMNG_InputImageData(const VIOMNG_ImageData *imgData)
    {
        if (atomic_load(&g_inputImageStart) == TD_FALSE) {
            return TD_SUCCESS;
        }
        const ot_video_frame_info *srcFrame = &imgData->videoFrame[0];
        td_s32 ret;
        OT_VIO_CameraData vioImageData = { 0 };
        OT_VIO_ImageData *vioImage = &vioImageData.leftImage;

        td_void *virtAddr = srcFrame->video_frame.virt_addr[0];

        vioImage->height = srcFrame->video_frame.height;
        vioImage->width = srcFrame->video_frame.width;
        vioImage->stride = srcFrame->video_frame.stride[0];
        vioImage->virtAddr = (td_u8 *)virtAddr;
        vioImage->timestamp = ((td_double)srcFrame->video_frame.pts) / VIOMNG_US_PER_S;
        vioImage->exposureDuration = ((td_double)imgData->expTime[0]) / VIOMNG_US_PER_S;
        ret = SS_VIO_PushImageData(&vioImageData);
        OT_APPCOMM_LOG_AND_GOTO_IF_EXPR_FALSE(ret == TD_SUCCESS, FAIL, "SS_VIO_PushImageData");
        return TD_SUCCESS;
    FAIL:
        return ret;
    }
    喂IMU
    static td_s32 InputImuData(const VIOMNG_ImuDataBuf *imuData)
    {
        if (atomic_load(&g_inputImuStart) == TD_FALSE) {
            return TD_SUCCESS;
        }

        OT_VIO_ImuDataInfo vioImuDatas = {0};
        for (td_u32 i = 0; i < imuData->num && i < OT_VIO_IMU_DATA_NUM_MAX; ++i) {
            vioImuDatas.imuDatas[i].timestamp = (td_double)imuData->imuData[i].timeStamp / VIOMNG_US_PER_S;
            vioImuDatas.imuDatas[i].accX = imuData->imuData[i].accX;
            vioImuDatas.imuDatas[i].accY = imuData->imuData[i].accY;
            vioImuDatas.imuDatas[i].accZ = imuData->imuData[i].accZ;
            vioImuDatas.imuDatas[i].gyroX = imuData->imuData[i].gyroX;
            vioImuDatas.imuDatas[i].gyroY = imuData->imuData[i].gyroY;
            vioImuDatas.imuDatas[i].gyroZ = imuData->imuData[i].gyroZ;
            vioImuDatas.num++;
        }

        return SS_VIO_PushImuData(&vioImuDatas);
    }
    返回
        SS_VIO_GetData返回VIO数据
#   虚拟机
    密码:空格
    /home/blowfish/ws_project/aliyun_hisi_project
    cd /home/blowfish/ws_project/aliyun_hisi_project/w2q4-hi3519a-mediaserver/HiXuanyuanV100R002C01SPC001
    make reference
    1.编译完成后，文件在目录
        /home/blowfish/ws_project/aliyun_hisi_project/w2q4-hi3519a-mediaserver/HiXuanyuanV100R002C01SPC001/build/image/ss927v100_dronecam_nonescreen_wsb_imx586/spinor
    2.如果已经有系统，不需要全部升级拷贝config和app.tar.gz到tf卡跟目录
    3.开机，手机连接 wifi,运行APP,切换附镜头，可以看到单目图像，点VIO Start，串口会输出如下VIO信息
    [INFO ][13:45:29.322][medialib_demo.cc-Vio_OnVioResult:0312]    [^^^blowfish-debug] medialib_demo timestamp=74.8451 x=0.0244904 y=0.00717578 z=-0.00997899
    [INFO ][13:45:29.323][medialib_demo.cc-Vio_OnVioResult:0314]    [^^^blowfish-debug] medialib_demo yaw= -0.329876 pitch= -3.34565 roll= -91.9796

    [INFO ][13:45:29.333][medialib_demo.cc-Vio_OnVioResult:0312]    [^^^blowfish-debug] medialib_demo timestamp=74.8569 x=0.0244873 y=0.00716381 z=-0.0099675
    [INFO ][13:45:29.333][medialib_demo.cc-Vio_OnVioResult:0314]    [^^^blowfish-debug] medialib_demo yaw= -0.326175 pitch= -3.34559 roll= -91.9747
# 验证指标并分析下一步优化方案
    手持悬停移动精度
    飞行悬停精度
    如何量化验证
    难度：如何量化验证
# 实施
    一起现场联调？