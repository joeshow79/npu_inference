/* Copyright (c) 2018-2019 Sigmastar Technology Corp.
 All rights reserved.

  Unless otherwise stipulated in writing, any and all information contained
 herein regardless in any format shall remain the sole proprietary of
 Sigmastar Technology Corp. and be kept in strict confidence
 (锟斤拷Sigmastar Confidential Information锟斤拷) by the recipient.
 Any unauthorized act including without limitation unauthorized disclosure,
 copying, use, reproduction, sale, distribution, modification, disassembling,
 reverse engineering and compiling of the contents of Sigmastar Confidential
 Information is unlawful and strictly prohibited. Sigmastar hereby reserves the
 rights to any and all damages, losses, costs and expenses resulting therefrom.
*/
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>

#include "st_common.h"
#include "st_vif.h"
#include "st_vpe.h"
#include "st_venc.h"
#include "st_uvc.h"

#include "BasicUsageEnvironment.hh"
#include "liveMedia.hh"
#include "Live555RTSPServer.hh"

#include "mi_rgn.h"
#include "mi_divp.h"

#include "mi_od.h"
#include "mi_md.h"

#include "mi_vdf.h"
#include "mi_ao.h"
#include "mi_aio_datatype.h"
#include "mi_isp.h"
#include "mi_iqserver.h"

#include "linux_list.h"
#include "mi_sys.h"
#include "mi_ipu.h"
#include <string.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <math.h>

#include <string>

#include <sys/time.h>
#include <unistd.h>


#include "mi_common_datatype.h"
#include "mi_sys_datatype.h"
#include "mi_ipu.h"
#include "mi_sys.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <error.h>
#include <errno.h>
#include <pthread.h>

#include <string.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
//#include <vector>

#include <string>
#include "mi_common_datatype.h"
#include "mi_sys_datatype.h"
#include "mi_ipu.h"
#include "mi_sys.h"
#include "mi_rgn.h"
#include <sys/time.h>
#include <unistd.h>

using namespace std;
#define INNER_MOST_ALIGNMENT (8)

#define INOUT_HEAP_SIZE   (6*1000*1000)
#define LABEL_CLASS_COUNT (100)
#define LABEL_NAME_MAX_SIZE (60)

#define RTSP_LISTEN_PORT        554
#define USB_CAMERA0_INDEX          0
#define USB_CAMERA1_INDEX          1
#define USB_CAMERA2_INDEX          2
#define UVC_STREAM0                "uvc_stream0"
#define UVC_STREAM1                "uvc_stream1"
#define UVC_STREAM2                "uvc_stream2"
#define MAX_UVC_DEV_NUM             3
#define PATH_PREFIX                "/mnt"
#define DEBUG_ES_FILE            0

#define RGN_OSD_TIME_START        8
#define RGN_OSD_MAX_NUM         4
#define RGN_OSD_TIME_WIDTH        180
#define RGN_OSD_TIME_HEIGHT        32

#define DOT_FONT_FILE            "/customer/mi_demo/gb2312.hzk"

#define MAX_CAPTURE_NUM            4
#define CAPTURE_PATH            "/mnt/capture"

#define MI_AUDIO_SAMPLE_PER_FRAME 768
#define AO_INPUT_FILE            "8K_16bit_STERO_30s.wav"
#define AO_OUTPUT_FILE            "./tmp.pcm"

#define DIVP_CHN_FOR_OSD        0
#define DIVP_CHN_FOR_DLA        1
#define DIVP_CHN_FOR_VDF        2
#define DIVP_CHN_FOR_ROTATION    3
#define DIVP_CHN_FOR_SCALE        3
#define VENC_CHN_FOR_CAPTURE    12

#define USE_TEST    0
#define USE_STREAM_FILE 0
#define DIVP_CHN_FOR_RESOLUTION 0
#define DIVP_SCALE_IPNUT_FILE    "vpe0_port0_1920x1088_0000.yuv"

#define MAX_CHN_NEED_OSD        4

#define RAW_W                     384
#define RAW_H                     288
#define PANEL_DIVP_ROTATE        0

#define RGB_TO_CRYCB(r, g, b)                                                            \
        (((unsigned int)(( 0.439f * (r) - 0.368f * (g) - 0.071f * (b)) + 128.0f)) << 16) |    \
        (((unsigned int)(( 0.257f * (r) + 0.564f * (g) + 0.098f * (b)) + 16.0f)) << 8) |        \
        (((unsigned int)((-0.148f * (r) - 0.291f * (g) + 0.439f * (b)) + 128.0f)))

#define IQ_FILE_PATH    "/customer/iq_api.bin"
#define IQ_HDR_FILE_PATH    "/customer/hdr.bin"


#define MAX_RGN_COVER_W               8192
#define MAX_RGN_COVER_H               8192

#define RGN_OSD_HANDLE                    0
#define RGN_FOR_VDF_BEGIN                12

#define MAX_FULL_RGN_NULL                3

#ifdef ALIGN_UP
#undef ALIGN_UP
#define ALIGN_UP(x, align) (((x) + ((align) - 1)) & ~((align) - 1))
#else
#define ALIGN_UP(x, align) (((x) + ((align) - 1)) & ~((align) - 1))
#endif
#ifndef ALIGN_DOWN
#define ALIGN_DOWN(val, alignment) (((val)/(alignment))*(alignment))
#endif

struct DetectionBBoxInfo {
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    float score;
    int   classID;

};


struct ST_Stream_Attr_T
{
    MI_BOOL        bEnable;
    ST_Sys_Input_E enInput;
    MI_U32     u32InputChn;
    MI_U32     u32InputPort;
    MI_S32     vencChn;
    MI_VENC_ModType_e eType;
    float      f32Mbps;
    MI_U32     u32Width;
    MI_U32     u32Height;

    MI_U32 enFunc;
    const char    *pszStreamName;
    MI_SYS_BindType_e eBindType;
    MI_U32 u32BindPara;
};

typedef struct
{
    MI_S32 s32UseOnvif;     //0:not use, else use
    MI_S32 s32UseVdf;         // 0: not use, else use
    MI_S32 s32LoadIQ;        // 0: not load, else load IQ file
    MI_S32 s32HDRtype;
    ST_Sensor_Type_T enSensorType;
    MI_SYS_Rotate_e enRotation;
    MI_VPE_3DNR_Level_e en3dNrLevel;
    MI_U8 u8SnrPad;
    MI_S8 s8SnrResIndex;
} ST_Config_S;

typedef struct {
    MI_U32  fcc;
    MI_U32  u32Width;
    MI_U32  u32Height;
    MI_U32  u32FrameRate;
    MI_SYS_ChnPort_t dstChnPort;
} ST_UvcSetting_Attr_T;

typedef struct VENC_STREAMS_s {
    bool used;
    MI_VENC_Stream_t stStream;
} VENC_STREAMS_t;

typedef struct {
    VENC_STREAMS_t *pstuserptr_stream;
} ST_Uvc_Resource_t;

typedef struct {
    char name[20];
    int dev_index;
    ST_UVC_Handle_h handle;
    ST_UvcSetting_Attr_T setting;
    ST_Uvc_Resource_t res;
} ST_UvcDev_t;

typedef struct {
    int devnum;
    ST_UvcDev_t dev[];
} ST_UvcSrc_t;

static MI_BOOL g_bExit = FALSE;
static ST_Config_S g_stConfig;
static ST_UvcSrc_t * g_UvcSrc;
static MI_U8 g_bitrate[MAX_UVC_DEV_NUM] = {0, 0, 0};
static MI_U8 g_maxbuf_cnt = 3;
static MI_U64 g_reqIdr_cnt = 0;
static MI_U8 g_enable_iqserver = 0;
static MI_U8 g_load_iq_bin = 0;
static MI_U32 g_device_num = 1;
static char g_IspBinPath[64] = {0};
static MI_BOOL g_bStartCapture = FALSE;


static struct ST_Stream_Attr_T g_stStreamAttr[] =
{
    [USB_CAMERA0_INDEX] =
    {
        .bEnable = TRUE,
        .enInput = ST_Sys_Input_VPE,
        .u32InputChn = 0,
        .u32InputPort = 0,
        .vencChn = 0,
        .eType = E_MI_VENC_MODTYPE_H264E,
        .f32Mbps = 2.0,
        .u32Width = 1920,
        .u32Height = 1080,
        .enFunc = ST_Sys_Func_UVC,
        .pszStreamName = UVC_STREAM0,
        .eBindType = E_MI_SYS_BIND_TYPE_REALTIME,
        .u32BindPara = 0,
    },
    [USB_CAMERA1_INDEX] =
    {
        .bEnable = TRUE,
        .enInput = ST_Sys_Input_VPE,
        .u32InputChn = 0,
        .u32InputPort = 1,
        .vencChn = 1,
        .eType = E_MI_VENC_MODTYPE_JPEGE,
        .f32Mbps = 2.0,
        .u32Width = 1920,
        .u32Height = 1080,
        .enFunc = ST_Sys_Func_UVC,
        .pszStreamName = UVC_STREAM1,
        .eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE,
        .u32BindPara = 0,
    },
    [USB_CAMERA2_INDEX] =
    {
        .bEnable = FALSE,
        .enInput = ST_Sys_Input_VPE,
        .u32InputChn = 0,
        .u32InputPort = 2,
        .vencChn = 2,
        .eType = E_MI_VENC_MODTYPE_H264E,
        .f32Mbps = 2.0,
        .u32Width = 1920,
        .u32Height = 1080,
        .enFunc = ST_Sys_Func_UVC,
        .pszStreamName = UVC_STREAM2,
        .eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE,//E_MI_SYS_BIND_TYPE_HW_RING,
        .u32BindPara = 0,
    },
};



MI_S32 IPUCreateDevice(char* pFirmwarePath, MI_U32 u32VarBufSize)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_IPU_DevAttr_t stDevAttr;
    stDevAttr.u32MaxVariableBufSize = u32VarBufSize;
    stDevAttr.u32YUV420_W_Pitch_Alignment = 16;
    stDevAttr.u32YUV420_H_Pitch_Alignment = 2;
    stDevAttr.u32XRGB_W_Pitch_Alignment = 16;
    s32Ret = MI_IPU_CreateDevice(&stDevAttr, NULL, pFirmwarePath, 0);
    return s32Ret;
}


MI_S32 IPUCreateChannel(MI_U32 *s32Channel, char *pModelImage, MI_U32 u32InBufDepth, MI_U32 u32OutBufDepth)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_SYS_GlobalPrivPoolConfig_t stGlobalPrivPoolConf;
    MI_IPUChnAttr_t stChnAttr;
    //create channel
    memset(&stGlobalPrivPoolConf, 0 ,sizeof(stGlobalPrivPoolConf));
    memset(&stChnAttr, 0, sizeof(stChnAttr));
    stChnAttr.u32InputBufDepth         = u32InBufDepth;
    stChnAttr.u32OutputBufDepth     = u32OutBufDepth;
    s32Ret = MI_IPU_CreateCHN((MI_IPU_CHN *)s32Channel, &stChnAttr, NULL, pModelImage);
    if(MI_SUCCESS != s32Ret)
    {
        printf("create IPU chn %d error[0x%x]\n", *s32Channel, s32Ret);
      //  stGlobalPrivPoolConf.bCreate = 0;
      //  MI_SYS_ConfigPrivateMMAPool(&stGlobalPrivPoolConf);
    }

    return s32Ret;
}


void ShowFloatOutPutTensor(float *pfBBox,float *pfClass, float *pfscore, float *pfDetect, int MAX_CNT)
{
    // show bbox
    int s32DetectCount = round(*pfDetect);
    //cout<<"BBox:"<<std::endl;
    cout.flags(ios::left);
    for(int i=0;i<s32DetectCount && i<MAX_CNT;i++)
    {
       for(int j=0;j<4;j++)
       {
            //cout<<setw(15)<<*(pfBBox+(i*ALIGN_UP(4,INNER_MOST_ALIGNMENT))+j);
       }
       if (i!=0)
       {
            //cout<<std::endl;
       }
    }

    //show class
    cout<<"Class:"<<std::endl;
    for(int  i=0;i<s32DetectCount && i<MAX_CNT;i++)
    {
       cout<<setw(15)<<round(*(pfClass+i));
    }
    cout<<std::endl;

    //show score
    cout<<"score:"<<std::endl;
    for(int  i=0;i<s32DetectCount && i<MAX_CNT;i++)
    {
       cout<<setw(15)<<*(pfscore+i);
    }
    cout<<std::endl;

    //show deteccout
    cout<<"DetectCount"<<std::endl;
    cout<<s32DetectCount<<std::endl;
}

MI_S32 IPUDestroyChannel(MI_U32 s32Channel, MI_U32 u32HeapSize)
{
    return MI_IPU_DestroyCHN(s32Channel);
}

void ST_Flush(void)
{
    char c;

    while((c = getchar()) != '\n' && c != EOF);
}

MI_BOOL ST_DoSetIqBin(MI_VPE_CHANNEL Vpechn,char *pConfigPath)
{
    MI_ISP_IQ_PARAM_INIT_INFO_TYPE_t status;
    MI_U8  u8ispreadycnt = 0;
    if (strlen(pConfigPath) == 0)
    {
        printf("IQ Bin File path NULL!\n");
        return FALSE;
    }

    do
    {
        if(u8ispreadycnt > 100)
        {
            printf("%s:%d, isp ready time out \n", __FUNCTION__, __LINE__);
            u8ispreadycnt = 0;
            break;
        }

        MI_ISP_IQ_GetParaInitStatus(Vpechn, &status);
        if(status.stParaAPI.bFlag != 1)
        {
            usleep(300*1000);
            u8ispreadycnt++;
            continue;
        }

        u8ispreadycnt = 0;

        printf("loading api bin...path:%s\n",pConfigPath);
        MI_ISP_API_CmdLoadBinFile(Vpechn, (char *)pConfigPath, 1234);

        usleep(10*1000);
    }while(!status.stParaAPI.bFlag);

    MI_ISP_AE_FLICKER_TYPE_e Flicker= SS_AE_FLICKER_TYPE_50HZ;
    MI_ISP_AE_SetFlicker(0, &Flicker);

    printf("MI_ISP_AE_SetFlicker!\n");
    return 0;
}

MI_BOOL ST_DoChangeHdrRes(MI_BOOL bEnableHdr, MI_U8 _u8ResIndex)
{
    MI_S32 s32SnrPad =0;
    MI_SNR_PAD_ID_e eSnrPad;
    MI_SNR_PADInfo_t  stPad0Info;
    MI_SNR_PlaneInfo_t stSnrPlane0Info;
    MI_VIF_DEV vifDev = 0;
    MI_VIF_CHN vifChn = 0;
    MI_VIF_PORT vifPort =0;
    MI_U8  u8VpeChn = 0;
    MI_VIF_WorkMode_e eVifWorkMode = E_MI_VIF_WORK_MODE_RGB_REALTIME;
    MI_VPE_HDRType_e eHdrType = E_MI_VPE_HDR_TYPE_OFF;
    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
    memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));

    MI_U32 u32CapWidth = 0, u32CapHeight = 0;
    MI_SYS_PixelFormat_e ePixFormat;

    MI_S32 select = 0;

    eSnrPad = (MI_SNR_PAD_ID_e)s32SnrPad;

    /************************************************
    Step1: VPE Stop ==> wait driver all buffer done
    *************************************************/
    STCHECKRESULT(MI_VPE_StopChannel(u8VpeChn));

    /************************************************
    Step2:  Destory VIF
    *************************************************/
    STCHECKRESULT(MI_VIF_DisableChnPort(vifChn, 0));
    STCHECKRESULT(MI_VIF_DisableDev(vifDev));

    /************************************************
    Step3:  Destory Sensor
    *************************************************/
    STCHECKRESULT(MI_SNR_Disable(eSnrPad));

    /************************************************
    Step4: Choice Hdr Type
    *************************************************/
    printf("sony sensor(ex imx307) use DOL, imx415 use VC, sc sensor(ex sc4238) use VC\n");

    if(bEnableHdr)
    {
        eHdrType = E_MI_VPE_HDR_TYPE_VC;
        printf("You select enable VC mode HDR\n");
    }
    else
    {
        eHdrType = E_MI_VPE_HDR_TYPE_OFF;
        printf("You select disable HDR\n");
    }

    /************************************************
    Step5:  Init Sensor
    *************************************************/
    STCHECKRESULT(MI_SNR_SetPlaneMode(eSnrPad, bEnableHdr));


    MI_U8 u8ResIndex =0;
    MI_U32 u32ResCount =0;
    MI_SNR_Res_t stRes;
    memset(&stRes, 0x0, sizeof(MI_SNR_Res_t));

    STCHECKRESULT(MI_SNR_QueryResCount(eSnrPad, &u32ResCount));
    for(u8ResIndex=0; u8ResIndex < u32ResCount; u8ResIndex++)
    {
       MI_SNR_GetRes(eSnrPad, u8ResIndex, &stRes);
       printf("index %d, Crop(%d,%d,%d,%d), outputsize(%d,%d), maxfps %d, minfps %d, ResDesc %s\n",
       u8ResIndex,
       stRes.stCropRect.u16X, stRes.stCropRect.u16Y, stRes.stCropRect.u16Width,stRes.stCropRect.u16Height,
       stRes.stOutputSize.u16Width, stRes.stOutputSize.u16Height,
       stRes.u32MaxFps,stRes.u32MinFps,
       stRes.strResDesc);
    }

    STCHECKRESULT(MI_SNR_SetRes(eSnrPad,(MI_U8)_u8ResIndex));
    STCHECKRESULT(MI_SNR_Enable(eSnrPad));

    STCHECKRESULT(MI_SNR_GetPadInfo(eSnrPad, &stPad0Info));
    STCHECKRESULT(MI_SNR_GetPlaneInfo(eSnrPad, 0, &stSnrPlane0Info));
    u32CapWidth = stSnrPlane0Info.stCapRect.u16Width;
    u32CapHeight = stSnrPlane0Info.stCapRect.u16Height;
    ePixFormat = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(stSnrPlane0Info.ePixPrecision, stSnrPlane0Info.eBayerId);

    /************************************************
    Step6:  init VIF
    *************************************************/
    MI_VIF_DevAttr_t stDevAttr;
    memset(&stDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));

    stDevAttr.eIntfMode = stPad0Info.eIntfMode;
    stDevAttr.eWorkMode = eVifWorkMode;
    stDevAttr.eHDRType = (MI_VIF_HDRType_e)eHdrType;
    if(stDevAttr.eIntfMode == E_MI_VIF_MODE_BT656)
        stDevAttr.eClkEdge = stPad0Info.unIntfAttr.stBt656Attr.eClkEdge;
    else
        stDevAttr.eClkEdge = E_MI_VIF_CLK_EDGE_DOUBLE;

    if(stDevAttr.eIntfMode == E_MI_VIF_MODE_MIPI)
        stDevAttr.eDataSeq =stPad0Info.unIntfAttr.stMipiAttr.eDataYUVOrder;
    else
        stDevAttr.eDataSeq = E_MI_VIF_INPUT_DATA_YUYV;

    if(stDevAttr.eIntfMode == E_MI_VIF_MODE_BT656)
        memcpy(&stDevAttr.stSyncAttr, &stPad0Info.unIntfAttr.stBt656Attr.stSyncAttr, sizeof(MI_VIF_SyncAttr_t));

    stDevAttr.eBitOrder = E_MI_VIF_BITORDER_NORMAL;

    STCHECKRESULT(MI_VIF_SetDevAttr(vifDev, &stDevAttr));
    STCHECKRESULT(MI_VIF_EnableDev(vifDev));

    MI_VIF_ChnPortAttr_t stVifPortInfo;
    memset(&stVifPortInfo, 0, sizeof(MI_VIF_ChnPortAttr_t));
    stVifPortInfo.stCapRect.u16X = stSnrPlane0Info.stCapRect.u16X;
    stVifPortInfo.stCapRect.u16Y = stSnrPlane0Info.stCapRect.u16Y;
    stVifPortInfo.stCapRect.u16Width =  stSnrPlane0Info.stCapRect.u16Width;
    stVifPortInfo.stCapRect.u16Height = stSnrPlane0Info.stCapRect.u16Height;
    stVifPortInfo.stDestSize.u16Width = u32CapWidth;
    stVifPortInfo.stDestSize.u16Height = u32CapHeight;
    stVifPortInfo.ePixFormat = ePixFormat;
    //stVifPortInfo.u32FrameModeLineCount for lowlantancy mode

    if(stDevAttr.eIntfMode == E_MI_VIF_MODE_BT656)
    {
        stVifPortInfo.eFrameRate = E_MI_VIF_FRAMERATE_FULL;
        stVifPortInfo.eCapSel = E_MI_SYS_FIELDTYPE_BOTH;
        stVifPortInfo.eScanMode = E_MI_SYS_FRAME_SCAN_MODE_PROGRESSIVE;
    }
    STCHECKRESULT(MI_VIF_SetChnPortAttr(vifChn, vifPort, &stVifPortInfo));
    STCHECKRESULT(MI_VIF_EnableChnPort(vifChn, vifPort));

    /************************************************
    Step6: Vpe Start
    *************************************************/
    MI_VPE_ChannelPara_t stVpeChParam;
    memset(&stVpeChParam, 0x0, sizeof(MI_VPE_ChannelPara_t));

    STCHECKRESULT(MI_VPE_GetChannelParam(u8VpeChn, &stVpeChParam));
    stVpeChParam.eHDRType = eHdrType;
    STCHECKRESULT(MI_VPE_SetChannelParam(u8VpeChn, &stVpeChParam));

    STCHECKRESULT(MI_VPE_StartChannel(u8VpeChn));

    return 0;
}
MI_S32 ST_Test()
{

    MI_S32 select = -1;
    MI_BOOL bEnableHdr = FALSE;
    MI_S8 s8SnrResIndex = 7;
    MI_SYS_WindowRect_t stChnCropWin;
    MI_BOOL bMirror = FALSE;
    MI_BOOL bFlip = FALSE;
    memset(&stChnCropWin, 0x0, sizeof(stChnCropWin));

    while(!g_bExit)
    {

        /* sensor imx415 */
        printf("****************************************************************\n");
        printf("1:open hdr,2:close hdr, 3:vpe-channel-crop 4: mirror 0:exit \n");
        printf("****************************************************************\n");
        scanf("%d", &select);
        ST_Flush();

        switch(select){
            case 1:
                bEnableHdr =TRUE;
                s8SnrResIndex = 2;
                strcpy(g_IspBinPath, IQ_HDR_FILE_PATH);
                g_load_iq_bin = 1;
                 if(g_bStartCapture)
                {
                    ST_DoSetIqBin(0, g_IspBinPath);
                    g_load_iq_bin = 0;
                }
                ST_DoChangeHdrRes(bEnableHdr,s8SnrResIndex);
                break;
           case 2:
                bEnableHdr =FALSE;
                s8SnrResIndex = 7;
                strcpy(g_IspBinPath, IQ_FILE_PATH);
                 g_load_iq_bin = 1;
                if(g_bStartCapture)
                {
                    ST_DoSetIqBin(0, g_IspBinPath);
                    g_load_iq_bin = 0;
                }
                ST_DoChangeHdrRes(bEnableHdr,s8SnrResIndex);
                break;
            case 3:

                stChnCropWin.u16X = 0;
                stChnCropWin.u16Y = 0;
                stChnCropWin.u16Width = 1280;
                stChnCropWin.u16Height = 720;
                STCHECKRESULT(MI_VPE_SetChannelCrop(0, &stChnCropWin));
                break;
             case 4:
                bMirror = TRUE;
                bFlip = FALSE;
                STCHECKRESULT(MI_SNR_SetOrien(E_MI_SNR_PAD_ID_0, bMirror, bFlip));
                break;

            case 0:
                g_bExit = TRUE;
                break;
            }
   }
        return MI_SUCCESS;

}

std::vector<DetectionBBoxInfo >  GetDetections(float *pfBBox,float *pfClass, float *pfScore, float *pfDetect)
{
    // show bbox
    int s32DetectCount = round(*pfDetect);
    std::vector<DetectionBBoxInfo > detections(s32DetectCount);
    for(int i=0;i<s32DetectCount;i++)
    {
        DetectionBBoxInfo  detection;
        memset(&detection,0,sizeof(DetectionBBoxInfo));
        //box coordinate
        detection.ymin =  *(pfBBox+(i*ALIGN_UP(4,INNER_MOST_ALIGNMENT))+0);
        detection.xmin =  *(pfBBox+(i*ALIGN_UP(4,INNER_MOST_ALIGNMENT))+1);
        detection.ymax =  *(pfBBox+(i*ALIGN_UP(4,INNER_MOST_ALIGNMENT))+2);
        detection.xmax =  *(pfBBox+(i*ALIGN_UP(4,INNER_MOST_ALIGNMENT))+3);


        //box class
        detection.classID = round(*(pfClass+i));


        //score
        detection.score = *(pfScore+i);
        detections.push_back(detection);

    }

    return detections;

}

MI_S32 ST_BaseModuleInit(ST_Config_S* pstConfig)
{
    MI_U32 u32CapWidth = 0, u32CapHeight = 0;
    MI_VIF_FrameRate_e eFrameRate = E_MI_VIF_FRAMERATE_FULL;
    MI_SYS_PixelFormat_e ePixFormat;
    ST_VPE_ChannelInfo_T stVpeChannelInfo;
    ST_Sys_BindInfo_T stBindInfo;
    MI_SNR_PADInfo_t  stPad0Info;
    MI_SNR_PlaneInfo_t stSnrPlane0Info;
    MI_VIF_WorkMode_e eVifWorkMode = E_MI_VIF_WORK_MODE_RGB_REALTIME;
    MI_VIF_HDRType_e eVifHdrType = E_MI_VIF_HDR_TYPE_OFF;
    MI_VPE_HDRType_e eVpeHdrType = E_MI_VPE_HDR_TYPE_OFF;
    MI_U32 u32ResCount =0;
    MI_U8 u8ResIndex =0;
    MI_SNR_Res_t stRes;
    MI_U32 u32ChocieRes =0;

    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
    memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));
    memset(&stRes, 0x0, sizeof(MI_SNR_Res_t));

    MI_SNR_PAD_ID_e      ePADId = (MI_SNR_PAD_ID_e)pstConfig->u8SnrPad;

    ST_DBG("Snr pad id:%d\n", (int)ePADId);

    MI_SYS_Init();
    if(pstConfig->s32HDRtype > 0)
        MI_SNR_SetPlaneMode(ePADId, TRUE);
    else
        MI_SNR_SetPlaneMode(ePADId, FALSE);

    MI_SNR_QueryResCount(ePADId, &u32ResCount);
    for(u8ResIndex=0; u8ResIndex < u32ResCount; u8ResIndex++)
    {
        MI_SNR_GetRes(ePADId, u8ResIndex, &stRes);
        printf("index %d, Crop(%d,%d,%d,%d), outputsize(%d,%d), maxfps %d, minfps %d, ResDesc %s\n",
        u8ResIndex,
        stRes.stCropRect.u16X, stRes.stCropRect.u16Y, stRes.stCropRect.u16Width,stRes.stCropRect.u16Height,
        stRes.stOutputSize.u16Width, stRes.stOutputSize.u16Height,
        stRes.u32MaxFps,stRes.u32MinFps,
        stRes.strResDesc);
    }

    MI_S8 s8SnrResIndex = pstConfig->s8SnrResIndex;

    if(s8SnrResIndex < 0)
    {
        printf("choice which resolution use, cnt %d\n", u32ResCount);
        do
        {
            scanf("%d", &u32ChocieRes);
            ST_Flush();
            MI_SNR_QueryResCount(ePADId, &u32ResCount);
            if(u32ChocieRes >= u32ResCount)
            {
                printf("choice err res %d > =cnt %d\n", u32ChocieRes, u32ResCount);
            }
        }while(u32ChocieRes >= u32ResCount);
    }
    else
    {
        if((MI_U32)s8SnrResIndex >= u32ResCount)
        {
            ST_ERR("snr index:%d exceed u32ResCount:%d, set default index 0\n", s8SnrResIndex, u32ResCount);
            s8SnrResIndex = 0;
        }

        u32ChocieRes = s8SnrResIndex;
    }
    printf("You select %d res\n", u32ChocieRes);

    MI_SNR_SetRes(ePADId,u32ChocieRes);
    MI_SNR_Enable(ePADId);

    MI_SNR_GetPadInfo(ePADId, &stPad0Info);
    MI_SNR_GetPlaneInfo(ePADId, 0, &stSnrPlane0Info);

    u32CapWidth = stSnrPlane0Info.stCapRect.u16Width;
    u32CapHeight = stSnrPlane0Info.stCapRect.u16Height;
    eFrameRate = E_MI_VIF_FRAMERATE_FULL;
    ePixFormat = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(stSnrPlane0Info.ePixPrecision, stSnrPlane0Info.eBayerId);

    //g_stStreamAttr[0].u32Width = u32CapWidth;
    //g_stStreamAttr[0].u32Height = u32CapHeight;

    /************************************************
    Step1:  init SYS
    *************************************************/
    STCHECKRESULT(ST_Sys_Init());

    if(ePADId == E_MI_SNR_PAD_ID_1)
    {
        //Bind vif dev 0 to sensor pad 1, because only vif dev0 can use realtime mode.
        MI_VIF_Dev2SnrPadMuxCfg_t stDev2SnrPad[4];
        stDev2SnrPad[0].eSensorPadID = E_MI_VIF_SNRPAD_ID_1;
        stDev2SnrPad[0].u32PlaneID = 0xff;
        stDev2SnrPad[1].eSensorPadID = E_MI_VIF_SNRPAD_ID_2;
        stDev2SnrPad[1].u32PlaneID = 0xff;
        stDev2SnrPad[2].eSensorPadID = E_MI_VIF_SNRPAD_ID_0;
        stDev2SnrPad[2].u32PlaneID = 0xff;
        stDev2SnrPad[3].eSensorPadID = E_MI_VIF_SNRPAD_ID_3;
        stDev2SnrPad[3].u32PlaneID = 0xff;

        MI_VIF_SetDev2SnrPadMux(stDev2SnrPad, 4);
    }
    /************************************************
    Step2:  init VIF(for IPC, only one dev)
    *************************************************/
    eVifHdrType = E_MI_VIF_HDR_TYPE_OFF;
    eVifWorkMode = E_MI_VIF_WORK_MODE_RGB_REALTIME;

    STCHECKRESULT(ST_Vif_EnableDev(0, eVifWorkMode, eVifHdrType, &stPad0Info));

    ST_VIF_PortInfo_T stVifPortInfoInfo;
    memset(&stVifPortInfoInfo, 0, sizeof(ST_VIF_PortInfo_T));
    stVifPortInfoInfo.u32RectX = 0;
    stVifPortInfoInfo.u32RectY = 0;
    stVifPortInfoInfo.u32RectWidth = u32CapWidth;
    stVifPortInfoInfo.u32RectHeight = u32CapHeight;
    stVifPortInfoInfo.u32DestWidth = u32CapWidth;
    stVifPortInfoInfo.u32DestHeight = u32CapHeight;
    stVifPortInfoInfo.eFrameRate = eFrameRate;
    stVifPortInfoInfo.ePixFormat = ePixFormat;
    STCHECKRESULT(ST_Vif_CreatePort(0, 0, &stVifPortInfoInfo));
    STCHECKRESULT(ST_Vif_StartPort(0, 0, 0));
    //if (enRotation != E_MI_SYS_ROTATE_NONE)
    {
        MI_BOOL bMirror = FALSE, bFlip = FALSE;
        //ExecFunc(MI_VPE_SetChannelRotation(0, enRotation), MI_SUCCESS);

        switch(pstConfig->enRotation)
        {
        case E_MI_SYS_ROTATE_NONE:
            bMirror= FALSE;
            bFlip = FALSE;
            break;
        case E_MI_SYS_ROTATE_90:
            bMirror = FALSE;
            bFlip = TRUE;
            break;
        case E_MI_SYS_ROTATE_180:
            bMirror = TRUE;
            bFlip = TRUE;
            break;
        case E_MI_SYS_ROTATE_270:
            bMirror = TRUE;
            bFlip = FALSE;
            break;
        default:
            break;
        }

        MI_SNR_SetOrien(ePADId, bMirror, bFlip);
        MI_VPE_SetChannelRotation(0, pstConfig->enRotation);
    }

    //for (unsigned int i = 0; i < g_device_num;i++)
    for (unsigned int i = 0; i < 1; i++)
    {
        ST_Stream_Attr_T *pstStreamAttr = &g_stStreamAttr[i];

        memset(&stVpeChannelInfo, 0, sizeof(ST_VPE_ChannelInfo_T));
        memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));

        stVpeChannelInfo.u16VpeMaxW = u32CapWidth;
        stVpeChannelInfo.u16VpeMaxH = u32CapHeight;
        stVpeChannelInfo.u32X = 0;
        stVpeChannelInfo.u32Y = 0;
        stVpeChannelInfo.u16VpeCropW = 0;
        stVpeChannelInfo.u16VpeCropH = 0;
        stVpeChannelInfo.eRunningMode = E_MI_VPE_RUN_REALTIME_MODE;

        stVpeChannelInfo.eFormat = ePixFormat;
        stVpeChannelInfo.e3DNRLevel = pstConfig->en3dNrLevel;
        stVpeChannelInfo.eHDRtype = eVpeHdrType;
        stVpeChannelInfo.bRotation = FALSE;
        if(ePADId == E_MI_SNR_PAD_ID_1)
        {
            stVpeChannelInfo.eBindSensorId = E_MI_VPE_SENSOR1;
        }
        /* crop vpe channel need set zoom mode */
        stVpeChannelInfo.u32ChnPortMode = E_MI_VPE_ZOOM_LDC_NULL;

        STCHECKRESULT(ST_Vpe_CreateChannel(pstStreamAttr->u32InputChn, &stVpeChannelInfo));
        STCHECKRESULT(ST_Vpe_StartChannel(pstStreamAttr->u32InputChn));

        stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
        stBindInfo.stSrcChnPort.u32DevId = 0;
        stBindInfo.stSrcChnPort.u32ChnId = 0;
        stBindInfo.stSrcChnPort.u32PortId = 0;
        stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VPE;
        stBindInfo.stDstChnPort.u32DevId = 0;
        stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_REALTIME;
        stBindInfo.stDstChnPort.u32ChnId = pstStreamAttr->u32InputChn;
        stBindInfo.stDstChnPort.u32PortId = pstStreamAttr->u32InputPort;
        stBindInfo.u32SrcFrmrate = 30;
        stBindInfo.u32DstFrmrate = 30;
        STCHECKRESULT(ST_Sys_Bind(&stBindInfo));
    }

    if(g_enable_iqserver)
    {
        ST_DBG("MI_IQSERVER_Open...\n");
        STCHECKRESULT(MI_IQSERVER_Open(u32CapWidth, u32CapHeight, 0));
    }

    return MI_SUCCESS;
}

MI_S32 ST_BaseModuleUnInit(void)
{
    ST_Sys_BindInfo_T stBindInfo;

    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
    stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
    stBindInfo.stSrcChnPort.u32DevId = 0;
    stBindInfo.stSrcChnPort.u32ChnId = 0;
    stBindInfo.stSrcChnPort.u32PortId = 0;

    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VPE;
    stBindInfo.stDstChnPort.u32DevId = 0;
    stBindInfo.stDstChnPort.u32ChnId = 0;
    stBindInfo.stDstChnPort.u32PortId = 0;

    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = 30;
    STCHECKRESULT(ST_Sys_UnBind(&stBindInfo));

    /************************************************
    Step1:  destory VPE
    *************************************************/
    STCHECKRESULT(ST_Vpe_StopChannel(0));
    STCHECKRESULT(ST_Vpe_DestroyChannel(0));

    /************************************************
    Step2:  destory VIF
    *************************************************/
    STCHECKRESULT(ST_Vif_StopPort(0, 0));
    STCHECKRESULT(ST_Vif_DisableDev(0));

    /************************************************
    Step3:  destory SYS
    *************************************************/
    STCHECKRESULT(ST_Sys_Exit());

    if(g_enable_iqserver)
    {
        MI_IQSERVER_Close();
    }
    MI_SYS_Exit();
    return MI_SUCCESS;
}

void *ST_DIVPGetResult(void *args)
{
    MI_SYS_ChnPort_t stChnPort;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE stBufHandle;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_S32 s32Fd = 0;
    fd_set read_fds;
    struct timeval TimeoutVal;
    char szFileName[128];
    int fd = 0;
    MI_U32 u32GetFramesCount = 0;
    MI_BOOL _bWriteFile = TRUE;

    stChnPort.eModId = E_MI_MODULE_ID_DIVP;
    stChnPort.u32DevId = 0;
    stChnPort.u32ChnId = DIVP_CHN_FOR_VDF;
    stChnPort.u32PortId = 0;

    s32Ret = MI_SYS_GetFd(&stChnPort, &s32Fd);
    if(MI_SUCCESS != s32Ret)
    {
        ST_ERR("MI_SYS_GetFd 0, error, %X\n", s32Ret);
        return NULL;
    }
    s32Ret = MI_SYS_SetChnOutputPortDepth(&stChnPort, 2, 3);
    if (MI_SUCCESS != s32Ret)
    {
        ST_ERR("MI_SYS_SetChnOutputPortDepth err:%x, chn:%d,port:%d\n", s32Ret,
            stChnPort.u32ChnId, stChnPort.u32PortId);
        return NULL;
    }

    sprintf(szFileName, "divp%d.es", stChnPort.u32ChnId);
    printf("start to record %s\n", szFileName);
    fd = open(szFileName, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd < 0)
    {
        ST_ERR("create %s fail\n", szFileName);
    }

    while (1)
    {
        FD_ZERO(&read_fds);
        FD_SET(s32Fd, &read_fds);

        TimeoutVal.tv_sec  = 1;
        TimeoutVal.tv_usec = 0;

        s32Ret = select(s32Fd + 1, &read_fds, NULL, NULL, &TimeoutVal);

        if(s32Ret < 0)
        {
            ST_ERR("select failed!\n");
            //  usleep(10 * 1000);
            continue;
        }
        else if(s32Ret == 0)
        {
            ST_ERR("get divp frame time out\n");
            //usleep(10 * 1000);
            continue;
        }
        else
        {
            if(FD_ISSET(s32Fd, &read_fds))
            {
                s32Ret = MI_SYS_ChnOutputPortGetBuf(&stChnPort, &stBufInfo, &stBufHandle);

                if(MI_SUCCESS != s32Ret)
                {
                    //ST_ERR("MI_SYS_ChnOutputPortGetBuf err, %x\n", s32Ret);
                    continue;
                }

                // save one Frame YUV data
                if (fd > 0)
                {
                    if(_bWriteFile)
                    {
                        write(fd, stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0] +
                            stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[1] /2);
                    }

                }

                ++u32GetFramesCount;
                printf("channelId[%u] u32GetFramesCount[%u]\n", stChnPort.u32ChnId, u32GetFramesCount);

                MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            }
        }
    }

    if (fd > 0)
    {
        close(fd);
        fd = -1;
    }

    printf("exit record\n");
    return NULL;
}

void ST_DefaultConfig(ST_Config_S *pstConfig)
{
    pstConfig->s32UseOnvif     = 0;
    pstConfig->s32UseVdf    = 0;
    pstConfig->s32LoadIQ    = 0;
    pstConfig->s32HDRtype    = 0;
    pstConfig->enSensorType = ST_Sensor_Type_IMX291;
    pstConfig->enRotation = E_MI_SYS_ROTATE_NONE;
}

void ST_DefaultArgs(ST_Config_S *pstConfig)
{
    memset(pstConfig, 0, sizeof(ST_Config_S));
    ST_DefaultConfig(pstConfig);

    pstConfig->s32UseOnvif = 0;
    pstConfig->s32UseVdf = 0;
    pstConfig->s32LoadIQ = 0;
    pstConfig->enRotation = E_MI_SYS_ROTATE_NONE;
    pstConfig->en3dNrLevel = E_MI_VPE_3DNR_LEVEL1;
    pstConfig->s8SnrResIndex = -1;
}

void ST_HandleSig(MI_S32 signo)
{
    if(signo == SIGINT)
    {
        ST_INFO("catch Ctrl + C, exit normally\n");

        g_bExit = TRUE;
    }
}

static MI_S32 UVC_Init(void *uvc)
{
    return MI_SUCCESS;
}

static MI_S32 UVC_Deinit(void *uvc)
{
    return MI_SUCCESS;
}

static ST_UvcDev_t * Get_UVC_Device(void *uvc)
{
    ST_UVC_Device_t *pdev = (ST_UVC_Device_t*)uvc;

    for (int i = 0; i < g_UvcSrc->devnum;i++)
    {
        ST_UvcDev_t *dev = &g_UvcSrc->dev[i];
        if (!strcmp(dev->name, pdev->name))
        {
            return dev;
        }
    }
    return NULL;
}

static MI_S32 UVC_UP_FinishBuffer(void *uvc,ST_UVC_BufInfo_t *bufInfo)
{
    ST_UvcDev_t *dev = Get_UVC_Device(uvc);
    MI_S32 s32Ret = MI_SUCCESS;
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_SYS_BUF_HANDLE stBufHandle = NULL;
    VENC_STREAMS_t * pUserptrStream = NULL;
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;

    if (!dev)
        return -1;

    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            stBufHandle = bufInfo->b.handle;

            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
            {
                printf("%s Release Frame Failed\n", __func__);
                return s32Ret;
            }
            break;
        case V4L2_PIX_FMT_NV12:
            stBufHandle = bufInfo->b.handle;

            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
            {
                printf("%s Release Frame Failed\n", __func__);
                return s32Ret;
            }
            break;
        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
            pUserptrStream = (VENC_STREAMS_t*)bufInfo->b.handle;

            s32Ret = MI_VENC_ReleaseStream(VencChn, &pUserptrStream->stStream);
            if (MI_SUCCESS != s32Ret)
            {
                printf("%s Release Frame Failed\n", __func__);
                return s32Ret;
            }

            pUserptrStream->used = false;
            break;
    }
    return MI_SUCCESS;
}

static MI_S32 UVC_UP_FillBuffer(void *uvc,ST_UVC_BufInfo_t *bufInfo)
{
    ST_UvcDev_t * dev = Get_UVC_Device(uvc);
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE stBufHandle;
    MI_SYS_ChnPort_t dstChnPort;
    VENC_STREAMS_t * pUserptrStream = NULL;
    MI_VENC_Stream_t * pstStream = NULL;
    MI_VENC_Pack_t stPack[4];
    MI_VENC_ChnStat_t stStat;
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;

    if (!dev)
        return -1;

    dstChnPort = dev->setting.dstChnPort;
    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                return -EINVAL;

            bufInfo->b.start = (long unsigned int)stBufInfo.stFrameData.pVirAddr[0];
            bufInfo->length = stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0];

            bufInfo->b.handle = (long unsigned int)stBufHandle;
            break;
        case V4L2_PIX_FMT_NV12:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                return -EINVAL;

            bufInfo->b.start = (long unsigned int)stBufInfo.stFrameData.pVirAddr[0];
            bufInfo->length = stBufInfo.stFrameData.u16Height
                     * (stBufInfo.stFrameData.u32Stride[0] + stBufInfo.stFrameData.u32Stride[1] / 2);
            bufInfo->b.handle = (long unsigned int)stBufHandle;
            break;
        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
            for (int i = 0;i < g_maxbuf_cnt; i++)
            {
                if (!dev->res.pstuserptr_stream[i].used)
                {
                    pUserptrStream = &dev->res.pstuserptr_stream[i];
                    break;
                }
            }
            if (!pUserptrStream)
            {
                return -EINVAL;
            }
            pstStream = &pUserptrStream->stStream;

            memset(&stPack, 0, sizeof(MI_VENC_Pack_t) * 4);
            pstStream->pstPack = stPack;

            s32Ret = MI_VENC_Query(VencChn, &stStat);
            if(s32Ret != MI_SUCCESS || stStat.u32CurPacks == 0)
                return -EINVAL;

            pstStream->u32PackCount = 1;//only need 1 packet

            s32Ret = MI_VENC_GetStream(VencChn, pstStream, 40);
            if (MI_SUCCESS != s32Ret)
                return -EINVAL;

            bufInfo->b.start = (long unsigned int)pstStream->pstPack[0].pu8Addr;
            bufInfo->length = pstStream->pstPack[0].u32Len;

            bufInfo->b.handle = (long unsigned int)pUserptrStream;
            pUserptrStream->used = true;

            if (g_reqIdr_cnt++<5)
            {
                MI_VENC_RequestIdr(VencChn, TRUE);
            }
            break;
        default:
            return -EINVAL;
    }
    return MI_SUCCESS;
}
void osdInit(void);

static MI_S32 UVC_MM_FillBuffer(void *uvc,ST_UVC_BufInfo_t *bufInfo)
{
    ST_UvcDev_t * dev = Get_UVC_Device(uvc);
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_U32 u32Size, i;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE stBufHandle;
    MI_SYS_ChnPort_t dstChnPort;
    MI_VENC_Stream_t stStream;
    MI_VENC_Pack_t stPack[4];
    MI_VENC_ChnStat_t stStat;

    MI_U8 *u8CopyData = (MI_U8 *)bufInfo->b.buf;
    MI_U32 *pu32length = (MI_U32 *)&bufInfo->length;
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;

    if (!dev)
        return -1;

    dstChnPort = dev->setting.dstChnPort;

    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
            {
                return -EINVAL;
            }

            *pu32length = stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0];
            memcpy(u8CopyData, stBufInfo.stFrameData.pVirAddr[0], *pu32length);

            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                printf("%s Release Frame Failed\n", __func__);

            break;
        case V4L2_PIX_FMT_NV12:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                return -EINVAL;

            *pu32length = stBufInfo.stFrameData.u16Height
                    * (stBufInfo.stFrameData.u32Stride[0] + stBufInfo.stFrameData.u32Stride[1] / 2);


            memcpy(u8CopyData, stBufInfo.stFrameData.pVirAddr[0],
                            stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0]);
            u8CopyData += stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0];
            memcpy(u8CopyData, stBufInfo.stFrameData.pVirAddr[1],
                            stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[1]/2);
            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                printf("%s Release Frame Failed\n", __func__);
            break;
        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
            memset(&stStream, 0, sizeof(MI_VENC_Stream_t));
            memset(&stPack, 0, sizeof(MI_VENC_Pack_t) * 4);
            stStream.pstPack = stPack;

            s32Ret = MI_VENC_Query(VencChn, &stStat);
            if(s32Ret != MI_SUCCESS || stStat.u32CurPacks == 0)
                return -EINVAL;

            stStream.u32PackCount = stStat.u32CurPacks;

            s32Ret = MI_VENC_GetStream(VencChn, &stStream, 40);
            if (MI_SUCCESS != s32Ret)
                return -EINVAL;

            for(i = 0;i < stStat.u32CurPacks; i++)
            {
                u32Size = stStream.pstPack[i].u32Len;
                memcpy(u8CopyData,stStream.pstPack[i].pu8Addr, u32Size);
                u8CopyData += u32Size;
            }
            *pu32length = u8CopyData - (MI_U8 *)bufInfo->b.buf;


            bufInfo->is_tail = true;//default is frameEnd
            s32Ret = MI_VENC_ReleaseStream(VencChn, &stStream);
            if (MI_SUCCESS != s32Ret)
                printf("%s Release Frame Failed\n", __func__);

            if (g_reqIdr_cnt++<5)
            {
                MI_VENC_RequestIdr(VencChn, TRUE);
            }
            break;
        default:
            printf("unknown format %d\n", dev->setting.fcc);
            return -EINVAL;
    }
    return MI_SUCCESS;
}


static MI_S32 UVC_StartCapture(void *uvc,Stream_Params_t format)
{
    ST_UvcDev_t *dev = Get_UVC_Device(uvc);
    ST_Stream_Attr_T *pstStreamAttr;
    MI_SYS_ChnPort_t *dstChnPort;
    ST_Sys_BindInfo_T stBindInfo;
    MI_U32 u32Width, u32Height;

    /************************************************
    Step0:  Initial general param
    *************************************************/
    if (!dev)
        return -1;

    memset(&dev->setting, 0x00, sizeof(dev->setting));
    dev->setting.fcc = format.fcc;
    dev->setting.u32Width = format.width;
    dev->setting.u32Height = format.height;
    dev->setting.u32FrameRate = format.frameRate;

    dstChnPort = &dev->setting.dstChnPort;
pstStreamAttr = g_stStreamAttr;
    u32Width = dev->setting.u32Width;
    u32Height = dev->setting.u32Height;

    //if (dev->setting.fcc != V4L2_PIX_FMT_NV12 && dev->setting.fcc != V4L2_PIX_FMT_YUYV)
    //    	u32Height = ALIGN_UP(u32Height, 16);


    /************************************************
    Step1:  start VPE port
    *************************************************/
    MI_U32 VpeChn = pstStreamAttr[dev->dev_index].u32InputChn;
    MI_U32 VpePortId = pstStreamAttr[dev->dev_index].u32InputPort;
    ST_VPE_PortInfo_T stVpePortInfo;

    stVpePortInfo.DepVpeChannel   = VpeChn;
    stVpePortInfo.u16OutputWidth  = u32Width;
    stVpePortInfo.u16OutputHeight = u32Height;
    stVpePortInfo.eCompressMode = E_MI_SYS_COMPRESS_MODE_NONE;

    /************************************************
    Step2: Init Venc Option
    ************************************************/
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;
    MI_U32 u32FrameRate = 0;
    MI_VENC_ChnAttr_t stChnAttr;
    MI_U32  u32VenBitRate =0;
    MI_U32 VencDev = 0;
    bool bByFrame = true;

    memset(&stChnAttr, 0, sizeof(MI_VENC_ChnAttr_t));

    /************************************************
    Step3: Init Bind Option
    ************************************************/
    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
    stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
    stBindInfo.stSrcChnPort.u32DevId = 0;
    stBindInfo.stSrcChnPort.u32ChnId = VpeChn;
    stBindInfo.stSrcChnPort.u32PortId = VpePortId;

    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
    stBindInfo.stDstChnPort.u32ChnId = VencChn;
    stBindInfo.stDstChnPort.u32PortId = 0;

    stBindInfo.u32SrcFrmrate = dev->setting.u32FrameRate;
    stBindInfo.u32DstFrmrate = dev->setting.u32FrameRate;
    u32FrameRate =  dev->setting.u32FrameRate;

    if (dev->setting.fcc != V4L2_PIX_FMT_MJPEG &&
        dev->setting.fcc != V4L2_PIX_FMT_H264 &&
        dev->setting.fcc != V4L2_PIX_FMT_H265)
    {
        stBindInfo.eBindType = pstStreamAttr[dev->dev_index].eBindType;
        stBindInfo.u32BindParam = u32Height;
    }
    else
    {
        stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
        stBindInfo.u32BindParam = 0;
    }


    if (u32Width * u32Height > 2560 *1440)
    {
        u32VenBitRate = 1024 * 1024 * 8;
    }
    else if (u32Width * u32Height >= 1920 *1080)
    {
        u32VenBitRate = 1024 * 1024 * 4;
    }
    else if(u32Width * u32Height < 640*480)
    {
        u32VenBitRate = 1024 * 500;
    }
    else
    {
        u32VenBitRate = 1024 * 1024 * 2;
    }

    if (g_bitrate[dev->dev_index])
        u32VenBitRate = g_bitrate[dev->dev_index] * 1024 * 1024;

    if (!dev->res.pstuserptr_stream)
    {
        dev->res.pstuserptr_stream = (VENC_STREAMS_t*)calloc(g_maxbuf_cnt, sizeof(VENC_STREAMS_t));
    }

    g_reqIdr_cnt = 0;

    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;
            STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

            *dstChnPort = stBindInfo.stSrcChnPort;

            STCHECKRESULT(MI_SYS_SetChnOutputPortDepth(dstChnPort, g_maxbuf_cnt+1, g_maxbuf_cnt+2));
            break;
        case V4L2_PIX_FMT_NV12:
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

            *dstChnPort = stBindInfo.stSrcChnPort;

            STCHECKRESULT(MI_SYS_SetChnOutputPortDepth(dstChnPort, g_maxbuf_cnt+1, g_maxbuf_cnt+2));
            break;
        case V4L2_PIX_FMT_MJPEG:
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));
#if 1
                    stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_MJPEGFIXQP;
                    stChnAttr.stRcAttr.stAttrMjpegFixQp.u32SrcFrmRateNum = u32FrameRate;
                    stChnAttr.stRcAttr.stAttrMjpegFixQp.u32SrcFrmRateDen = 1;
                    stChnAttr.stVeAttr.stAttrJpeg.u32PicWidth = u32Width;
                    stChnAttr.stVeAttr.stAttrJpeg.u32PicHeight = u32Height;
                    stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicWidth =  u32Width;
                    stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicHeight = ALIGN_UP(u32Height, 16);
                    stChnAttr.stVeAttr.stAttrJpeg.bByFrame = true;
                    stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_JPEGE;

                    //stChnAttr.stRcAttr.stAttrMjpegFixQp.u32Qfactor = 50; //default 80, [0~90]
#else
                   stChnAttr.stVeAttr.stAttrJpeg.u32PicWidth = u32Width;
                   stChnAttr.stVeAttr.stAttrJpeg.u32PicHeight = u32Height;
                   stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicWidth = u32Width;
                   stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicHeight = ALIGN_UP(u32Height, 16);;
                   stChnAttr.stVeAttr.stAttrJpeg.bByFrame = bByFrame;

                   stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_MJPEGCBR;
                   stChnAttr.stRcAttr.stAttrMjpegCbr.u32BitRate = 1024*1024*15*u32VenBitRate;
                   stChnAttr.stRcAttr.stAttrMjpegCbr.u32SrcFrmRateNum = 30;
                   stChnAttr.stRcAttr.stAttrMjpegCbr.u32SrcFrmRateDen = 1;
                   stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_JPEGE;

#endif

            STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));

            STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
            stBindInfo.stDstChnPort.u32DevId = VencDev;
            STCHECKRESULT(ST_Sys_Bind(&stBindInfo));
            STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
            STCHECKRESULT(ST_Venc_StartChannel(VencChn));

            *dstChnPort = stBindInfo.stDstChnPort;
            break;
        case V4L2_PIX_FMT_H264:
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

            stChnAttr.stVeAttr.stAttrH264e.u32PicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH264e.u32PicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH264e.u32MaxPicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH264e.u32MaxPicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH264e.bByFrame = bByFrame;
            stChnAttr.stVeAttr.stAttrH264e.u32BFrameNum = 2;
            stChnAttr.stVeAttr.stAttrH264e.u32Profile = 1;

            stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H264CBR;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32BitRate = u32VenBitRate;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32FluctuateLevel = 0;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32Gop = 30;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateNum = u32FrameRate;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateDen = 1;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32StatTime = 0;
            stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_H264E;
            STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));
           {
                MI_VENC_ParamH264SliceSplit_t stSliceSplit = {true, 17};
                MI_VENC_SetH264SliceSplit(VencChn, &stSliceSplit);
            }
            STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
            stBindInfo.stDstChnPort.u32DevId = VencDev;
            STCHECKRESULT(ST_Sys_Bind(&stBindInfo));
            STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
            STCHECKRESULT(ST_Venc_StartChannel(VencChn));

            *dstChnPort = stBindInfo.stDstChnPort;
            break;
        case V4L2_PIX_FMT_H265:
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

            stChnAttr.stVeAttr.stAttrH265e.u32PicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH265e.u32PicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH265e.u32MaxPicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH265e.u32MaxPicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH265e.bByFrame = bByFrame;

            stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H265CBR;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32BitRate = u32VenBitRate;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateNum = u32FrameRate;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateDen = 1;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32Gop = 30;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32FluctuateLevel = 0;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32StatTime = 0;
            stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_H265E;
            STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));

            STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
            stBindInfo.stDstChnPort.u32DevId = VencDev;
            STCHECKRESULT(ST_Sys_Bind(&stBindInfo));
            STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
            STCHECKRESULT(ST_Venc_StartChannel(VencChn));

            *dstChnPort = stBindInfo.stDstChnPort;
            break;
        default:
            return -EINVAL;
    }
    osdInit();
    if(g_load_iq_bin)
    {
        ST_DoSetIqBin(0, g_IspBinPath);
        g_load_iq_bin = 0;
    }

    g_bStartCapture = TRUE;

    printf("Capture u32Width: %d, u32height: %d, format: %s\n",u32Width,u32Height,
        dev->setting.fcc==V4L2_PIX_FMT_YUYV ? "YUYV":(dev->setting.fcc==V4L2_PIX_FMT_NV12 ? "NV12":
        (dev->setting.fcc==V4L2_PIX_FMT_MJPEG ?"MJPEG":(dev->setting.fcc==V4L2_PIX_FMT_H264 ? "H264":"H265"))));

    return MI_SUCCESS;
}

static MI_S32 UVC_StopCapture(void *uvc)
{
    ST_UvcDev_t *dev = Get_UVC_Device(uvc);
    ST_Sys_BindInfo_T stBindInfo;
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_U32 VpeChn, VpePortId, VencDev, VencChn;

    if (!dev)
        return -1;

    /************************************************
    Step0:  General Param Set
    *************************************************/
    VpeChn = pstStreamAttr[dev->dev_index].u32InputChn;
    VpePortId = pstStreamAttr[dev->dev_index].u32InputPort,
    VencChn = pstStreamAttr[dev->dev_index].vencChn,

    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
    stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
    stBindInfo.stSrcChnPort.u32DevId = 0;
    stBindInfo.stSrcChnPort.u32ChnId = VpeChn;
    stBindInfo.stSrcChnPort.u32PortId = VpePortId;

    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
    stBindInfo.stDstChnPort.u32ChnId = VencChn;
    stBindInfo.stDstChnPort.u32PortId = 0;

    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = dev->setting.u32FrameRate;

    if (dev->res.pstuserptr_stream)
    {
        free(dev->res.pstuserptr_stream);
        dev->res.pstuserptr_stream = NULL;
    }

    /************************************************
    Step1:  Stop All Other Modules
    *************************************************/
    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
            break;
        case V4L2_PIX_FMT_NV12:
            STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
            break;
        case V4L2_PIX_FMT_MJPEG:
            STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
            stBindInfo.stDstChnPort.u32DevId = VencDev;
            STCHECKRESULT(ST_Sys_UnBind(&stBindInfo));
            STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
            STCHECKRESULT(ST_Venc_StopChannel(VencChn));
            STCHECKRESULT(ST_Venc_DestoryChannel(VencChn));
            break;
        case V4L2_PIX_FMT_H264:
            STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
            stBindInfo.stDstChnPort.u32DevId = VencDev;
            STCHECKRESULT(ST_Sys_UnBind(&stBindInfo));
            STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
            STCHECKRESULT(ST_Venc_StopChannel(VencChn));
            STCHECKRESULT(ST_Venc_DestoryChannel(VencChn));
            break;
        case V4L2_PIX_FMT_H265:
            STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
            stBindInfo.stDstChnPort.u32DevId = VencDev;
            STCHECKRESULT(ST_Sys_UnBind(&stBindInfo));
            STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
            STCHECKRESULT(ST_Venc_StopChannel(VencChn));
            STCHECKRESULT(ST_Venc_DestoryChannel(VencChn));
            break;
        default:
            return -EINVAL;
    }

    return MI_SUCCESS;
}

MI_S32 ST_UvcDeinit()
{
    if (!g_UvcSrc)
        return -1;

    for (int i = 0; i < g_UvcSrc->devnum; i++)
    {
        ST_UvcDev_t *dev = &g_UvcSrc->dev[i];
        STCHECKRESULT(ST_UVC_StopDev((dev->handle)));
        STCHECKRESULT(ST_UVC_DestroyDev(dev->handle));
        STCHECKRESULT(ST_UVC_Uninit(dev->handle));
    }
    return MI_SUCCESS;
}


MI_S32 ST_UvcInitDev(ST_UvcDev_t *dev, MI_U32 maxpacket, MI_U8 mult, MI_U8 burst, MI_U8 c_intf, MI_U8 s_intf, MI_S32 mode)
{
    ST_UVC_Setting_t pstSet={g_maxbuf_cnt, maxpacket, mult, burst, c_intf, s_intf, (UVC_IO_MODE_e)mode, USB_ISOC_MODE};
    ST_UVC_MMAP_BufOpts_t m = {UVC_MM_FillBuffer};
    ST_UVC_USERPTR_BufOpts_t u = {UVC_UP_FillBuffer, UVC_UP_FinishBuffer};

    ST_UVC_OPS_t fops = { UVC_Init,
                          UVC_Deinit,
                          {{}},
                          UVC_StartCapture,
                          UVC_StopCapture};
    if (mode==UVC_MEMORY_MMAP)
        fops.m = m;
    else
        fops.u = u;

    printf(ASCII_COLOR_YELLOW "ST_UvcInitDev: name:%s bufcnt:%d mult:%d burst:%d ci:%d si:%d, Mode:%s" ASCII_COLOR_END "\n",
                    dev->name, g_maxbuf_cnt, mult, burst, c_intf, s_intf, mode==UVC_MEMORY_MMAP?"mmap":"userptr");

    ST_UVC_ChnAttr_t pstAttr ={pstSet,fops};
    STCHECKRESULT(ST_UVC_Init(dev->name, &dev->handle));
    STCHECKRESULT(ST_UVC_CreateDev(dev->handle, &pstAttr));
    STCHECKRESULT(ST_UVC_StartDev(dev->handle));
    return MI_SUCCESS;
}
pthread_mutex_t g_mutex_UpadteOsdState = PTHREAD_MUTEX_INITIALIZER;

MI_S32 ST_UvcInit(MI_S32 devnum, MI_U32 *maxpacket, MI_U8 *mult, MI_U8 *burst, MI_U8 *intf, MI_S32 mode)
{
    char devnode[20] = "/dev/video0";

    if (devnum > MAX_UVC_DEV_NUM)
    {
        printf(ASCII_COLOR_YELLOW "%s Max Uvc Dev Num %d\n" ASCII_COLOR_END "\n", __func__, MAX_UVC_DEV_NUM);
        devnum = MAX_UVC_DEV_NUM;
    }

    g_UvcSrc = (ST_UvcSrc_t*)malloc(sizeof(g_UvcSrc) + sizeof(ST_UvcDev_t) * devnum);
    g_UvcSrc->devnum = devnum;

    for (int i = 0; i < devnum; i++)
    {
        ST_UvcDev_t *dev = &g_UvcSrc->dev[i];
        sprintf(devnode, "/dev/video%d", i);
        dev->dev_index = i;
        memcpy(dev->name, devnode, sizeof(devnode));
        ST_UvcInitDev(dev, maxpacket[i], mult[i], burst[i], intf[2*i], intf[2*i+1], mode);
    }
    return MI_SUCCESS;
}

void ST_DoExitProc(void *args)
{
    g_bExit = TRUE;
}

void getbox(   const vector<DetectionBBoxInfo > detections, const float threshold)
{
  // Retrieve detections.
  //vector<DetectionBBoxInfo>  detectionsInImage;

   for (unsigned int j = 0; j < detections.size(); j++) 
   {
       DetectionBBoxInfo bbox;
       const int label = detections[j].classID;
       const float score = detections[j].score;
       if (score < threshold) 
           continue;

       bbox.xmin =  detections[j].xmin * 320;
       bbox.xmin = bbox.xmin < 0 ? 0:bbox.xmin ;
       bbox.ymin =  detections[j].ymin * 240;
       bbox.ymin = bbox.ymin < 0 ? 0:bbox.ymin ;
       bbox.xmax =  detections[j].xmax * 320;
       bbox.ymax =  detections[j].ymax * 240;
       bbox.score = score;
       bbox.classID = label;
       printf("position,x:%f,y:%f,w:%f,h:%f\n",bbox.xmin,bbox.ymin,bbox.xmax - bbox.xmin,bbox.ymax-bbox.ymin);
       //detectionsInImage.push_back(bbox);
   }
}
#define DIVP_CHN 0
#define DIVP_PORT 0
#define DIVP_FRAME_RATE 15
#define DIVP_OUTPUT_WIDTH 320
#define DIVP_OUTPUT_HEIGHT 240
static int dump_divpimage(void* pdataY, int size,int width,int height)
{
    int w = width;
    int h = height;
    printf("dump_divp size(%d)\n", size);
    static int id = 0;
    char filename[64] = {0};
    if(id > 2)
    id = 0;
    sprintf(filename, "/mnt/argb%02d_%dx%d_rgb.rgb565", id, w, h);
    FILE* fs = fopen(filename,"wb");
    fwrite((char*)pdataY,1,w*h*3/2,fs);
    fclose(fs);
    id++;
    return 0;
}
static MI_RGN_PaletteTable_t g_stPaletteTable = {
    { //index0 ~ index15
     {  0,   0,   0,   0}, {255, 255,   0,   0}, {255,   0, 255,   0}, {255,   0,   0, 255},
     {255, 255, 255,   0}, {255,   0, 112, 255}, {255,   0, 255, 255}, {255, 255, 255, 255},
     {255, 128,   0,   0}, {255, 128, 128,   0}, {255, 128,   0, 128}, {255,   0, 128,   0},
     {255,   0,   0, 128}, {255,   0, 128, 128}, {255, 128, 128, 128}, {255,  64,  64,  64}}
};
typedef struct _Color_t
{
    MI_U8  a;
    MI_U8  r;
    MI_U8  g;
    MI_U8  b;
} Color_t;
typedef struct
{
    MI_RGN_PixelFormat_e ePixelFmt;
    MI_U32 u32Color;
}DrawRgnColor_t;
typedef struct
{
    MI_U32 u16X;
    MI_U32 u16Y;
}DrawPoint_t;


MI_RGN_HANDLE rgnRectHandle = 1;
MI_S32 getVideoSize(MI_VENC_CHN      vencChn, MI_U32 &width, MI_U32 &height )
{
    MI_VENC_ChnAttr_t stVencChnAttr;
    MI_S32 s32Ret = -1;
    if(vencChn < 0)
        return -1;
    memset(&stVencChnAttr, 0, sizeof(MI_VENC_ChnAttr_t));
    s32Ret = MI_VENC_GetChnAttr(vencChn, &stVencChnAttr);
    if(MI_SUCCESS != s32Ret)
    {
        printf("MI_VENC_GetChnAttr fail, %d\n", s32Ret);
        return s32Ret;
    }

    switch(stVencChnAttr.stVeAttr.eType)
    {
        case E_MI_VENC_MODTYPE_H264E:
            width  = stVencChnAttr.stVeAttr.stAttrH264e.u32PicWidth;
            height = stVencChnAttr.stVeAttr.stAttrH264e.u32PicHeight;
            break;
        case E_MI_VENC_MODTYPE_H265E:
            width  = stVencChnAttr.stVeAttr.stAttrH265e.u32PicWidth;
            height = stVencChnAttr.stVeAttr.stAttrH265e.u32PicHeight;
            break;
        case E_MI_VENC_MODTYPE_JPEGE:
            width  = stVencChnAttr.stVeAttr.stAttrJpeg.u32PicWidth;
            height = stVencChnAttr.stVeAttr.stAttrJpeg.u32PicHeight;
            break;
        default:
            printf("Not support Venc type: %d\n", stVencChnAttr.stVeAttr.eType);
    }
    return MI_SUCCESS;
}
static Color_t g_stBlackColor     = {(MI_U8)255, (MI_U8)0,   (MI_U8)0,  (MI_U8)0};
static Color_t g_stRedColor       = {(MI_U8)255, (MI_U8)255, (MI_U8)0,   (MI_U8)0};

void DrawRect(void *pBaseAddr, MI_U32 u32Stride, DrawPoint_t stLeftTopPt, DrawPoint_t stRightBottomPt, MI_U8 u8BorderWidth, DrawRgnColor_t stColor)
{
    MI_U32 i = 0;
    MI_U32 j = 0;
    MI_U32 u32Width = stRightBottomPt.u16X - stLeftTopPt.u16X + 1;
    MI_U32 u32Height = stRightBottomPt.u16Y - stLeftTopPt.u16Y + 1;

    if ( (u8BorderWidth > u32Width/2) || (u8BorderWidth > u32Height/2) )
    {
        printf("invalid border width\n");
        return;
    }


    switch(stColor.ePixelFmt)
    {
            case E_MI_RGN_PIXEL_FORMAT_I2:
            {
                MI_U8 *pDrawBase = (MI_U8*)pBaseAddr;
                if (stLeftTopPt.u16X%4 || stRightBottomPt.u16X%4 || (u8BorderWidth > u32Width/4) || (u8BorderWidth > u32Height/4))
                {
                    printf("invalid rect position\n");
                    return;
                }
                for (i = 0; i < u32Width/4; i++)
                {
                    for (j = 0; j < u8BorderWidth && ((stLeftTopPt.u16X/4 + i) < u32Stride); j++)
                    {
                        *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+j)+stLeftTopPt.u16X/4+i) = (stColor.u32Color&0x03) | ((stColor.u32Color&0x03) << 2) | ((stColor.u32Color&0x03) << 4) | ((stColor.u32Color&0x03) << 6);     // copy 1 byte
                        *(pDrawBase+u32Stride*(stRightBottomPt.u16Y-j)+stLeftTopPt.u16X/4+i) = (stColor.u32Color&0x03) | ((stColor.u32Color&0x03) << 2) | ((stColor.u32Color&0x03) << 4) | ((stColor.u32Color&0x03) << 6); // copy 1 byte
                    }
                }
                for (i = 0; i < u32Height; i++)
                {
                    for (j = 0; j < u8BorderWidth/4; j++)
                    {
                        if((stLeftTopPt.u16X/4 + j) < u32Stride)
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stLeftTopPt.u16X/4+j) = (stColor.u32Color&0x03) | ((stColor.u32Color&0x03) << 2) | ((stColor.u32Color&0x03) << 4) | ((stColor.u32Color&0x03) << 6);    // copy 1 byte
                        }
                        if((stRightBottomPt.u16X/4 - j) < u32Stride)
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stRightBottomPt.u16X/4-j) = (stColor.u32Color&0x03) | ((stColor.u32Color&0x03) << 2) | ((stColor.u32Color&0x03) << 4) | ((stColor.u32Color&0x03) << 6);// copy 1 byte
                        }
                    }
                    if (u8BorderWidth % 4)
                    {
                        if(((stLeftTopPt.u16X/4 + u8BorderWidth/4) < u32Stride))
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stLeftTopPt.u16X/4+u8BorderWidth/4) &= 0xf0;
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stLeftTopPt.u16X/4+u8BorderWidth/4) |= (stColor.u32Color&0x03) | ((stColor.u32Color&0x03) << 2) ;
                        }
                        if(((stRightBottomPt.u16X/4 - u8BorderWidth/4) < u32Stride))
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stRightBottomPt.u16X/4-u8BorderWidth/4) &= 0x0f;
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stRightBottomPt.u16X/4-u8BorderWidth/4) |= ((stColor.u32Color&0x03) | ((stColor.u32Color&0x03) << 2) ) << 4;
                        }
                    }
                }
            }
            break;
        case E_MI_RGN_PIXEL_FORMAT_I4:
            {
                MI_U8 *pDrawBase = (MI_U8*)pBaseAddr;

                if (stLeftTopPt.u16X%2 || stRightBottomPt.u16X%2)
                {
                    printf("invalid rect position\n");
                    return;
                }

                for (i = 0; i < u32Width/2; i++)
                {
                    for (j = 0; j < u8BorderWidth && ((stLeftTopPt.u16X/2 + i) < u32Stride); j++)
                    {

                        *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+j)+stLeftTopPt.u16X/2+i) = (stColor.u32Color&0x0f) | ((stColor.u32Color&0x0f) << 4);     // copy 1 byte
                        *(pDrawBase+u32Stride*(stRightBottomPt.u16Y-j)+stLeftTopPt.u16X/2+i) = (stColor.u32Color&0x0f) | ((stColor.u32Color&0x0f) << 4); // copy 1 byte
                    }
                }

                for (i = 0; i < u32Height; i++)
                {
                    for (j = 0; j < u8BorderWidth/2; j++)
                    {
                        if((stLeftTopPt.u16X/2 + j) < u32Stride)
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stLeftTopPt.u16X/2+j) = (stColor.u32Color&0x0f) | ((stColor.u32Color&0x0f) << 4);    // copy 1 byte
                        }

                        if((stRightBottomPt.u16X/2 - j) < u32Stride)
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stRightBottomPt.u16X/2-j) = (stColor.u32Color&0x0f) | ((stColor.u32Color&0x0f) << 4);// copy 1 byte
                        }
                    }

                    if (u8BorderWidth % 2)
                    {
                        if(((stLeftTopPt.u16X/2 + u8BorderWidth/2) < u32Stride))
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stLeftTopPt.u16X/2+u8BorderWidth/2) &= 0xf0;
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stLeftTopPt.u16X/2+u8BorderWidth/2) |= stColor.u32Color&0x0f;
                        }

                        if(((stRightBottomPt.u16X/2 - u8BorderWidth/2) < u32Stride))
                        {
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stRightBottomPt.u16X/2-u8BorderWidth/2) &= 0x0f;
                            *(pDrawBase+u32Stride*(stLeftTopPt.u16Y+i)+stRightBottomPt.u16X/2-u8BorderWidth/2) |= (stColor.u32Color&0x0f) << 4;
                        }
                    }
                }
            }
            break;
        case E_MI_RGN_PIXEL_FORMAT_ARGB1555:
            {
                MI_U16 *pDrawBase = (MI_U16*)pBaseAddr;

                for (i = 0; i < u32Width; i++)
                {
                    for (j = 0; j < u8BorderWidth && (stLeftTopPt.u16X + i) < u32Stride/2; j++)
                    {
                        *(pDrawBase+u32Stride/2*(stLeftTopPt.u16Y+j)+stLeftTopPt.u16X+i) = stColor.u32Color & 0xffff;    // copy 2 byte, app check alignment
                        *(pDrawBase+u32Stride/2*(stRightBottomPt.u16Y-j)+stLeftTopPt.u16X+i) = stColor.u32Color & 0xffff;// copy 2 byte, app check alignment
                    }
                }

                for (i = 0; i < u32Height; i++)
                {
                    for (j = 0; j < u8BorderWidth; j++)
                    {
                        if((stLeftTopPt.u16X + j) < u32Stride/2)
                        {
                            *(pDrawBase+u32Stride/2*(stLeftTopPt.u16Y+i)+stLeftTopPt.u16X+j) = stColor.u32Color & 0xffff;    // copy 2 byte, app check alignment
                        }

                        if((stRightBottomPt.u16X - j) < u32Stride/2)
                        {
                            *(pDrawBase+u32Stride/2*(stLeftTopPt.u16Y+i)+stRightBottomPt.u16X-j) = stColor.u32Color & 0xffff;// copy 2 byte, app check alignment
                        }
                    }
                }
            }
            break;
        default:
            printf("format not support\n");
    }
}
typedef struct _RectWidgetAttr_s
{
    MI_SYS_WindowRect_t *pstRect;
    MI_S32 s32RectCnt;
    MI_U8 u8BorderWidth;
    MI_U8 u32Color;
    MI_RGN_PixelFormat_e pmt;
    Color_t* pfColor;
    Color_t* pbColor;
    MI_BOOL bFill;
    MI_BOOL bHard;
    MI_BOOL bOutline;
} RectWidgetAttr_t;
#ifndef RGB2PIXEL1555
#define RGB2PIXEL1555(a,r,g,b)    (((a & 0x80) << 8) | ((r & 0xF8) << 7) | ((g & 0xF8) << 2) | ((b & 0xF8) >> 3))
#endif
#ifndef ALIGN_UP
#define ALIGN_UP(value, align)    ((value+align-1) / align * align)
#endif

#ifndef ALIGN_DOWN
#define ALIGN_DOWN(value, align)  (value / align * align)
#endif

static MI_S32 updateOsdRectWidgetTmp(const MI_RGN_CanvasInfo_t *pRgnInfo, RectWidgetAttr_t *pstRectWidgetAttr)
{
   MI_S32 s32Ret = E_MI_ERR_FAILED;
   MI_S32 i = 0;
   DrawRgnColor_t stColor;
   DrawPoint_t stLefTopPt;
   DrawPoint_t stRightBottomPt;
   MI_RGN_CanvasInfo_t stRgnCanvasInfo;
   MI_SYS_WindowRect_t *pstRectTmp;

   if(NULL == pRgnInfo)
   {
       s32Ret = E_MI_ERR_ILLEGAL_PARAM;
       return s32Ret;
   }

   if(NULL == pstRectWidgetAttr)
   {
       printf("updateOsdRectWidget() the input pointer is NULL!\n");
       s32Ret = E_MI_ERR_NULL_PTR;
       return s32Ret;
   }

 //  pthread_mutex_lock(&g_stMutexMixerOsdTextRgnBuf);

    memcpy(&stRgnCanvasInfo, pRgnInfo, sizeof(MI_RGN_CanvasInfo_t));
   stColor.ePixelFmt = stRgnCanvasInfo.ePixelFmt;

   for (i = 0; i < pstRectWidgetAttr->s32RectCnt; i++)
   {
       pstRectTmp = (MI_SYS_WindowRect_t *)((pstRectWidgetAttr->pstRect) + i);
       if(pstRectTmp->u16Width == 0 || pstRectTmp->u16Height == 0)
           continue;
       stLefTopPt.u16X = pstRectTmp->u16X;
       stLefTopPt.u16Y = pstRectTmp->u16Y;
       stRightBottomPt.u16X = pstRectTmp->u16X + pstRectTmp->u16Width;
       stRightBottomPt.u16Y = pstRectTmp->u16Y + pstRectTmp->u16Height;

     /*  MIXER_DBG("Rect[%d]:x=%d, y=%d, w=%d, h=%d, fmt=%s\n", i, stLefTopPt.u16X, stLefTopPt.u16Y,
                  stRightBottomPt.u16X - stLefTopPt.u16X, stRightBottomPt.u16Y - stLefTopPt.u16Y, (0==pstRectWidgetAttr->pmt)?"ARGB1555":"I4");
    */
       switch(pstRectWidgetAttr->pmt)
       {
           case E_MI_RGN_PIXEL_FORMAT_ARGB1555:
               if(E_MI_RGN_PIXEL_FORMAT_ARGB1555 != stRgnCanvasInfo.ePixelFmt)
               {
                   printf("config the wrong poxer format, only support ARGB1555 now\n");
                 //  pthread_mutex_unlock(&g_stMutexMixerOsdTextRgnBuf);
                   return s32Ret;
               }

               stColor.u32Color = RGB2PIXEL1555(pstRectWidgetAttr->pfColor->a, pstRectWidgetAttr->pfColor->r,
                                                pstRectWidgetAttr->pfColor->g, pstRectWidgetAttr->pfColor->b);
               break;

           case E_MI_RGN_PIXEL_FORMAT_I4:
               if(E_MI_RGN_PIXEL_FORMAT_I4 != stRgnCanvasInfo.ePixelFmt)
               {
                   printf("config the wrong poxer format, only support I4 now\n");
                  // pthread_mutex_unlock(&g_stMutexMixerOsdTextRgnBuf);
                   return s32Ret;
               }

               stLefTopPt.u16X = ALIGN_DOWN(stLefTopPt.u16X, 2);
               stRightBottomPt.u16X = ALIGN_UP(stRightBottomPt.u16X, 2);
               stColor.u32Color = pstRectWidgetAttr->u32Color;
               break;

           case E_MI_RGN_PIXEL_FORMAT_I2:
               if(E_MI_RGN_PIXEL_FORMAT_I2 != stRgnCanvasInfo.ePixelFmt)
               {
                   printf("config the wrong poxer format, only support I2 now\n");
                 //  pthread_mutex_unlock(&g_stMutexMixerOsdTextRgnBuf);
                   return s32Ret;
               }
               stLefTopPt.u16X = ALIGN_DOWN(stLefTopPt.u16X, 4);
               stRightBottomPt.u16X = ALIGN_UP(stRightBottomPt.u16X, 4);
               stColor.u32Color = pstRectWidgetAttr->u32Color;
               break;
           default:
               printf("OSD only support %s now\n", (0==pstRectWidgetAttr->pmt)?"ARGB1555":(2==pstRectWidgetAttr->pmt)?"I2":"I4");
       }

       DrawRect((void*)stRgnCanvasInfo.virtAddr, stRgnCanvasInfo.u32Stride, stLefTopPt, stRightBottomPt, pstRectWidgetAttr->u8BorderWidth, stColor);
   }

   s32Ret = MI_RGN_OK;
 //  pthread_mutex_unlock(&g_stMutexMixerOsdTextRgnBuf);

   return s32Ret;
}



static int UpdateDLARect()
{
    MI_S32 s32VencChn = 0x0;
    MI_SYS_WindowRect_t stRect;
    MI_RGN_HANDLE hHandle = rgnRectHandle;
    RectWidgetAttr_t stRectWidgetAttr;
    ST_Point_T stPoint;
    MI_RGN_CanvasInfo_t stRgnCanvasInfo;
    static MI_BOOL mBeCurRect = FALSE;
    MI_SYS_WindowRect_t stDLARect;
    MI_U32 venc_width = 0;
    MI_U32 venc_height = 0;
    MI_S32 s32Ret = E_MI_ERR_FAILED;
    
    MI_BOOL beFine = FALSE;
    int i;
    MI_U8 clolor = 2;
    if(((MI_S32)hHandle <= MI_RGN_HANDLE_NULL  || hHandle >= MI_RGN_MAX_HANDLE))
    {
        printf(" mBeCurRect %d.  beFine=%d  hHandle=%d\n", mBeCurRect,beFine,hHandle);
        return 0;
    }
    
    if(MI_RGN_OK != MI_RGN_GetCanvasInfo(hHandle, &stRgnCanvasInfo))
    {
        printf("%s:%d call MI_RGN_GetCanvasInfo() fail.. handle:%d\n", __func__,__LINE__, hHandle);
        return 0;
    }
    
    stRect.u16X = 0;
    stRect.u16Y = 0;
    stRect.u16Width  = stRgnCanvasInfo.stSize.u32Width;
    stRect.u16Height = stRgnCanvasInfo.stSize.u32Height;

     MI_U32 dd = 0;
     MI_U8 *pu8BaseAddr = NULL;
     if(((stRect.u16X + stRect.u16Width)  <= stRgnCanvasInfo.stSize.u32Width) && \
        ((stRect.u16Y + stRect.u16Height) <= stRgnCanvasInfo.stSize.u32Height))
     {
         for(dd = 0; dd < stRect.u16Height; dd++)
         {
             pu8BaseAddr = (MI_U8*)(stRgnCanvasInfo.virtAddr + stRgnCanvasInfo.u32Stride*(dd + stRect.u16Y) + stRect.u16X / 2);

             for(i = 0; i < stRect.u16Width / 2; i++)
             {
                 *(pu8BaseAddr + i) = ((0 & 0x0F) << 4) | (0 & 0x0F);
             }
         }
     }

    getVideoSize(s32VencChn,venc_width,venc_height);
    printf("venc w:%d,h:%d\n",venc_width,venc_height);

    memset(&stDLARect, 0x00, sizeof(MI_SYS_WindowRect_t));
    stDLARect.u16X = 100;
    stDLARect.u16Y = 100;
    stDLARect.u16Width = 1500;
    stDLARect.u16Height = 800;
    memset(&stRectWidgetAttr, 0, sizeof(RectWidgetAttr_t));
    stRectWidgetAttr.pstRect = &stDLARect;
    stRectWidgetAttr.s32RectCnt = 1;
    stRectWidgetAttr.u8BorderWidth = 4;
    stRectWidgetAttr.pmt = E_MI_RGN_PIXEL_FORMAT_I4; 
    stRectWidgetAttr.pfColor = &g_stRedColor;
    stRectWidgetAttr.pbColor = &g_stBlackColor;
    stRectWidgetAttr.bFill = FALSE;
    stRectWidgetAttr.u32Color = 1;
    updateOsdRectWidgetTmp(&stRgnCanvasInfo, &stRectWidgetAttr);

    exit:
    if(MI_RGN_UpdateCanvas(hHandle) != MI_RGN_OK)
    {
        printf("MI_RGN_UpdateCanvas fail\n");
    }

    return 0;
}

void* OSD_Task(void *argu)
{
    MI_U32 tick = 0x0;
    MI_BOOL bVisible = TRUE;
    MI_S32  s32Idx = 0;
    MI_VENC_CHN s32VencChn = 0;
    MI_RGN_HANDLE u32OsdHandle = MI_RGN_HANDLE_NULL;
    struct timeval now;
    struct timespec outtime;

    printf("start OSD_Task \n");
    while(FALSE == g_bExit)
    {
         UpdateDLARect();
         sleep(1);
    }
//osd_task_exit:
    printf("quit OSD_Task\n");
    pthread_exit(NULL);

    return NULL;
}
MI_S32 getDividedNumber(MI_S32 value)
{
    if(0 == value % 32)       return value / 32;
    else if(0 == value % 30)  return value / 30;
    else if(0 == value % 28)  return value / 28;
    else if(0 == value % 27)  return value / 27;
    else if(0 == value % 25)  return value / 25;
    else if(0 == value % 24)  return value / 24;
    else if(0 == value % 22)  return value / 22;
    else if(0 == value % 21)  return value / 21;
    else if(0 == value % 20)  return value / 20;
    else if(0 == value % 18)  return value / 18;
    else if(0 == value % 16)  return value / 16;
    else if(0 == value % 15)  return value / 15;
    else if(0 == value % 14)  return value / 14;
    else if(0 == value % 12)  return value / 12;
    else if(0 == value % 10)  return value / 10;
    else if(0 == value % 9)   return value / 9;
    else if(0 == value % 8)   return value / 8;
    else if(0 == value % 7)   return value / 7;
    else if(0 == value % 6)   return value / 6;
    else if(0 == value % 5)   return value / 5;
    else if(0 == value % 4)   return value / 4;
    else if(0 == value % 3)   return value / 3;
    else if(0 == value % 2)   return value / 2;
    else return value;
}

void osdInit(void)
{
    MI_S32 s32Ret = E_MI_ERR_FAILED;
    MI_RGN_Attr_t stRgnAttr;
    MI_RGN_ChnPortParam_t stRgnChnPortParam;
    MI_RGN_ChnPort_t      stRgnChnPort;
    pthread_t thread_OSD;
    s32Ret = MI_RGN_Init(&g_stPaletteTable);
    if(MI_RGN_OK != s32Ret)
    {
        printf("MI_RGN_Init error(%X)\n", s32Ret);
        return;
    }
    memset(&stRgnAttr, 0, sizeof(MI_RGN_Attr_t));
    stRgnAttr.eType = E_MI_RGN_TYPE_OSD;
    stRgnAttr.stOsdInitParam.ePixelFmt = E_MI_RGN_PIXEL_FORMAT_I4;
    stRgnAttr.stOsdInitParam.stSize.u32Width  = 1920;
    stRgnAttr.stOsdInitParam.stSize.u32Height = 1080;
    s32Ret = MI_RGN_Create(rgnRectHandle,&stRgnAttr);
    if(MI_RGN_OK != s32Ret)
    {
        printf("MI_RGN_Create error(0x%X)\n", s32Ret);
        return;
    }
    memset(&stRgnChnPort, 0, sizeof(MI_RGN_ChnPort_t));
    stRgnChnPort.eModId = E_MI_RGN_MODID_VPE;
    stRgnChnPort.s32DevId = 0;
    stRgnChnPort.s32ChnId = 0;
    stRgnChnPort.s32OutputPortId = 0;
    memset(&stRgnChnPortParam, 0x00, sizeof(MI_RGN_ChnPortParam_t));
    stRgnChnPortParam.bShow = TRUE;
    stRgnChnPortParam.stPoint.u32X = 0;
    stRgnChnPortParam.stPoint.u32Y = 0;
    if(((stRgnAttr.stOsdInitParam.stSize.u32Width <= 1920) && (stRgnAttr.stOsdInitParam.stSize.u32Height <= 1080)) ||
       ((stRgnAttr.stOsdInitParam.stSize.u32Width <= 1080) && (stRgnAttr.stOsdInitParam.stSize.u32Height <= 1920)))
    {
        stRgnChnPortParam.unPara.stOsdChnPort.stColorInvertAttr.eInvertColorMode = E_MI_RGN_ABOVE_LUMA_THRESHOLD;
        stRgnChnPortParam.unPara.stOsdChnPort.stColorInvertAttr.u16WDivNum = getDividedNumber(stRgnAttr.stOsdInitParam.stSize.u32Width);
        stRgnChnPortParam.unPara.stOsdChnPort.stColorInvertAttr.u16HDivNum = getDividedNumber(stRgnAttr.stOsdInitParam.stSize.u32Height);

        if((1920 == stRgnAttr.stOsdInitParam.stSize.u32Width) && (1080 == stRgnAttr.stOsdInitParam.stSize.u32Height))
        {
            stRgnChnPortParam.unPara.stOsdChnPort.stColorInvertAttr.u16HDivNum /= 2;
        }
        else if((1080 == stRgnAttr.stOsdInitParam.stSize.u32Width) && (1920 == stRgnAttr.stOsdInitParam.stSize.u32Height))
        {
            stRgnChnPortParam.unPara.stOsdChnPort.stColorInvertAttr.u16WDivNum /= 2;
        }
        stRgnChnPortParam.unPara.stOsdChnPort.stColorInvertAttr.u16LumaThreshold = 96;
        stRgnChnPortParam.unPara.stOsdChnPort.stColorInvertAttr.bEnableColorInv = FALSE;/*可能有问题*/
    }
    stRgnChnPortParam.unPara.stOsdChnPort.u32Layer = (MI_U32)rgnRectHandle;
    s32Ret = MI_RGN_AttachToChn(rgnRectHandle, &stRgnChnPort, &stRgnChnPortParam);
    if(s32Ret != MI_RGN_OK)
    {
        printf("MI_RGN_AttachToChn error(0x%X)\n MI_RGN_Destroy ret=(0x%X)\n", s32Ret, MI_RGN_Destroy(rgnRectHandle));
        return;
    }
    pthread_attr_t attr; 
    MI_S32 policy = 0;
    pthread_attr_init(&attr);
    policy = SCHED_FIFO;
    pthread_attr_setschedpolicy(&attr,policy);
    struct sched_param s_parm;
    s_parm.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &s_parm);
    pthread_create(&thread_OSD, &attr, OSD_Task, (void*)NULL);
    pthread_setname_np(thread_OSD , "OSD_Task");
}

MI_S32  ST_DlaStart()
{
    MI_S32 s32Ret = MI_SUCCESS;
    ST_VPE_PortInfo_T stVpePortInfo;
    MI_SYS_ChnPort_t stChnPort;
    MI_DIVP_OutputPortAttr_t stDivpPortAttr;
    ST_Sys_BindInfo_T stBindInfo;
    MI_DIVP_ChnAttr_t stDivpChnAttr;
    MI_VPE_PortMode_t stVpeMode;
    memset(&stChnPort,0,sizeof(ST_VPE_PortInfo_T));
    memset(&stVpePortInfo, 0, sizeof(ST_VPE_PortInfo_T));
    memset(&stDivpPortAttr, 0, sizeof(MI_DIVP_OutputPortAttr_t));
    memset(&stBindInfo, 0, sizeof(ST_Sys_BindInfo_T));
    memset(&stDivpChnAttr, 0,sizeof(MI_DIVP_ChnAttr_t));
    /*start vpe port*/
    stVpePortInfo.DepVpeChannel = 0;
    stVpePortInfo.u16OutputWidth = 1920;
    stVpePortInfo.u16OutputHeight = 1080;
    stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    stVpePortInfo.eCompressMode = E_MI_SYS_COMPRESS_MODE_NONE;
    STCHECKRESULT(ST_Vpe_StartPort(1, &stVpePortInfo));
    stChnPort.eModId = E_MI_MODULE_ID_VPE;
    stChnPort.u32ChnId = stVpePortInfo.DepVpeChannel;
    stChnPort.u32DevId = 0;
    stChnPort.u32PortId = 0;
    MI_SYS_SetChnOutputPortDepth(&stChnPort, 1, 4);
    /*create divp*/
    memset(&stVpeMode, 0, sizeof(MI_VPE_PortMode_t));
    ExecFunc(MI_VPE_GetPortMode(0, 1, &stVpeMode), MI_VPE_OK);
    stDivpChnAttr.bHorMirror = FALSE;
    stDivpChnAttr.bVerMirror = FALSE;
    stDivpChnAttr.eDiType = E_MI_DIVP_DI_TYPE_OFF;
    stDivpChnAttr.eRotateType = E_MI_SYS_ROTATE_NONE;
    stDivpChnAttr.eTnrLevel = E_MI_DIVP_TNR_LEVEL_OFF;
    stDivpChnAttr.stCropRect.u16X = 0;
    stDivpChnAttr.stCropRect.u16Y = 0;
    stDivpChnAttr.stCropRect.u16Width = stVpeMode.u16Width;
    stDivpChnAttr.stCropRect.u16Height = stVpeMode.u16Height;
    printf("vpe port 1 ,width:%d,height:%d\n",stVpeMode.u16Width,stVpeMode.u16Height);
    stDivpChnAttr.u32MaxWidth = stVpeMode.u16Width;
    stDivpChnAttr.u32MaxHeight = stVpeMode.u16Height;
    ExecFunc(MI_DIVP_CreateChn(DIVP_CHN, &stDivpChnAttr), MI_SUCCESS);
    
    stDivpPortAttr.u32Width     = DIVP_OUTPUT_WIDTH;
    stDivpPortAttr.u32Height    = DIVP_OUTPUT_HEIGHT;
    stDivpPortAttr.eCompMode    = E_MI_SYS_COMPRESS_MODE_NONE;
    stDivpPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_ARGB8888;
    printf("divp output u32Width:%d, u32Height:%d\n", stDivpPortAttr.u32Width, stDivpPortAttr.u32Height);

    ExecFunc(MI_DIVP_SetOutputPortAttr(DIVP_CHN, &stDivpPortAttr), MI_SUCCESS);
    ExecFunc(MI_DIVP_StartChn(DIVP_CHN), MI_SUCCESS);

    stBindInfo.stSrcChnPort.eModId    = E_MI_MODULE_ID_VPE;
    stBindInfo.stSrcChnPort.u32DevId  = 0;
    stBindInfo.stSrcChnPort.u32ChnId  = 0;
    stBindInfo.stSrcChnPort.u32PortId = 1;

    stBindInfo.stDstChnPort.eModId    = E_MI_MODULE_ID_DIVP;
    stBindInfo.stDstChnPort.u32DevId  = 0;
    stBindInfo.stDstChnPort.u32ChnId  = DIVP_CHN;
    stBindInfo.stDstChnPort.u32PortId = 0;

    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = DIVP_FRAME_RATE;
    stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    MI_SYS_BindChnPort2(&stBindInfo.stSrcChnPort, &stBindInfo.stDstChnPort, \
                                 stBindInfo.u32SrcFrmrate, stBindInfo.u32DstFrmrate, \
                                 stBindInfo.eBindType, stBindInfo.u32BindParam);
    ExecFunc(MI_SYS_SetChnOutputPortDepth(&stBindInfo.stDstChnPort, 3, 4), MI_SUCCESS);
    MI_S32 s32Chn2GetFd ;
    memset(&stChnPort, 0x00, sizeof(MI_SYS_ChnPort_t));

    stChnPort.eModId      = E_MI_MODULE_ID_DIVP;
    stChnPort.u32DevId    = 0;
    stChnPort.u32ChnId    = DIVP_CHN;
    stChnPort.u32PortId   = 0;
    s32Ret = MI_SYS_GetFd(&stChnPort, &s32Chn2GetFd);
    if (s32Ret < 0)
    {
        printf("divp ch: %d, get fd. err\n", stChnPort.u32ChnId);
        return -1;
    }

    /*get divp data*/
    fd_set read_fds;
    struct timeval tv;
	MI_S32 s32GetBufRet = -1;
    MI_SYS_ChnPort_t stChnOutputPort;
    MI_IPU_TensorVector_t Vector4Affine;
    MI_SYS_BufInfo_t        stBufInfo;
    MI_SYS_BUF_HANDLE       stBufHandle;
    MI_SYS_BUF_HANDLE       stBufHandle_;

    char * pFirmwarePath ="/config/dla/ipu_firmware.bin";
    char * pModelImgPath ="ssd_mobilenet_v1_fixed.sim_sgsimg.img";
    //char * pModelImgPath ="hc_fixed.sim_sgsimg.img";
    // char * pModelImgPath ="1M_fixed_int16_320.sim_sgsimg.img";
    MI_U32 u32ChannelID = 0;
    MI_IPU_SubNet_InputOutputDesc_t desc;
    MI_IPU_OfflineModelStaticInfo_t OfflineModelInfo;
    //1.create device
    s32Ret = MI_IPU_GetOfflineModeStaticInfo(NULL, pModelImgPath, &OfflineModelInfo);
    if(MI_SUCCESS != s32Ret)
    {
        printf("get model variable buffer size failed!",s32Ret);
        return -1;
    }
    if(MI_SUCCESS !=IPUCreateDevice(pFirmwarePath,OfflineModelInfo.u32VariableBufferSize))
    {
        printf("create ipu device failed!");
        return -1;
    }
    printf("OfflineModelInfo.u32VariableBufferSize:%d\n",OfflineModelInfo.u32VariableBufferSize);
    
    //2.create channel
    if(MI_SUCCESS!=IPUCreateChannel(&u32ChannelID, pModelImgPath,1,1))
    {
         printf("create ipu channel failed!");
         MI_IPU_DestroyDevice();
         return -1;
    }
    //3.get input/output tensor
    MI_IPU_GetInOutTensorDesc(u32ChannelID, &desc);
    MI_IPU_TensorVector_t stInputTensorVector, stOutputTensorVector;
    MI_SYS_WindowRect_t stVpeToCrop;
    while(!g_bExit)
    {
        FD_ZERO(&read_fds);
        FD_SET(s32Chn2GetFd, &read_fds);

        tv.tv_sec = 0;
        tv.tv_usec = 100 * 1000;
        s32Ret = select(s32Chn2GetFd + 1, &read_fds, NULL, NULL, &tv);
        if (s32Ret < 0)
        {
            printf("get divp data error:ret=%d,fd:%d\n",s32Ret,s32Chn2GetFd);
            return -1;
        }
        else if (0 == s32Ret)
        {
            printf("get divp data timeout,fd:%d\n",s32Chn2GetFd);
            continue;
        }
        else
        {
            if(FD_ISSET(s32Chn2GetFd, &read_fds))
            {
                stChnOutputPort.eModId      = E_MI_MODULE_ID_DIVP;
                stChnOutputPort.u32DevId    = 0;
                stChnOutputPort.u32ChnId    = DIVP_CHN;
                stChnOutputPort.u32PortId   = 0;
                memset(&stBufInfo, 0x0, sizeof(MI_SYS_BufInfo_t));
                s32GetBufRet = MI_SYS_ChnOutputPortGetBuf(&stChnOutputPort, &stBufInfo, &stBufHandle);
                if(s32GetBufRet != MI_SUCCESS)
                {
                    printf("get divp data failed\n");
                    continue;
                }
                if (0 && MI_SUCCESS == s32GetBufRet)
                {
                    dump_divpimage((void*)stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u32BufSize,DIVP_OUTPUT_WIDTH,DIVP_OUTPUT_HEIGHT);
                    MI_SYS_ChnOutputPortPutBuf(stBufHandle);
                }
            }
        }

        // prepare input vector
        memset(&stInputTensorVector, 0, sizeof(MI_IPU_TensorVector_t));
        stInputTensorVector.u32TensorCount = desc.u32InputTensorCount;
        if (stBufInfo.eBufType == E_MI_SYS_BUFDATA_RAW)
        {
            //printf("E_MI_SYS_BUFDATA_RAW\n");
            stInputTensorVector.astArrayTensors[0].phyTensorAddr[0] = stBufInfo.stRawData.phyAddr;
            stInputTensorVector.astArrayTensors[0].ptTensorData[0] = stBufInfo.stRawData.pVirAddr;
        }
        else if (stBufInfo.eBufType == E_MI_SYS_BUFDATA_FRAME)
        {
            //printf("E_MI_SYS_BUFDATA_FRAME\n");
            stInputTensorVector.astArrayTensors[0].phyTensorAddr[0] = stBufInfo.stFrameData.phyAddr[0];
            stInputTensorVector.astArrayTensors[0].ptTensorData[0] = stBufInfo.stFrameData.pVirAddr[0];
    
            stInputTensorVector.astArrayTensors[0].phyTensorAddr[1] = stBufInfo.stFrameData.phyAddr[1];
            stInputTensorVector.astArrayTensors[0].ptTensorData[1] = stBufInfo.stFrameData.pVirAddr[1];
            printf("size:%d\n",stBufInfo.stFrameData.u32BufSize);
            dump_divpimage((void*)stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u32BufSize,DIVP_OUTPUT_WIDTH,DIVP_OUTPUT_HEIGHT);
        }
        memset(&stOutputTensorVector, 0, sizeof(MI_IPU_TensorVector_t));
        if (MI_SUCCESS != (s32Ret = MI_IPU_GetOutputTensors(u32ChannelID, &stOutputTensorVector)))
        {
            printf("MI_IPU_GetOutputTensors failed!\n");
            MI_IPU_PutInputTensors(u32ChannelID, &stInputTensorVector);
            return -1;
        }
        if(MI_SUCCESS != (s32Ret = MI_IPU_Invoke(u32ChannelID, &stInputTensorVector, &stOutputTensorVector)))
        {
            printf("MI_IPU_Invoke failed \n");
            MI_IPU_PutOutputTensors(u32ChannelID,&stOutputTensorVector);
            MI_IPU_PutInputTensors(u32ChannelID, &stInputTensorVector);
            return -1;
        }
        float *pfBBox = (float *)stOutputTensorVector.astArrayTensors[0].ptTensorData[0];
        float *pfClass = (float *)stOutputTensorVector.astArrayTensors[1].ptTensorData[0];
        float *pfScore = (float *)stOutputTensorVector.astArrayTensors[2].ptTensorData[0];
        float *pfDetect = (float *)stOutputTensorVector.astArrayTensors[3].ptTensorData[0];
        int max_cnt = 9;
        ShowFloatOutPutTensor( pfBBox, pfClass, pfScore, pfDetect,max_cnt);
        std::vector<DetectionBBoxInfo >  detections  = GetDetections(pfBBox,pfClass,  pfScore,  pfDetect);
        getbox(detections,0.6);
        MI_SYS_ChnOutputPortPutBuf(stBufHandle);
        MI_IPU_PutOutputTensors(u32ChannelID,&stOutputTensorVector);
    }
    return MI_SUCCESS;
}
static void help_message(char **argv)
{
    printf("\n");
    printf("usage: %s \n", argv[0]);
    printf(" -a set sensor pad\n");
    printf(" -A set sensor resolution index\n");
    printf(" -b bitrate\n");
    printf(" -B burst -m mult -p maxpacket\n");
    printf(" -M 0:mmap, 1:userptr\n");
    printf(" -N num of uvc stream\n");
    printf(" -i set iq api.bin,ex: -i \"/customer/imx415_api.bin\" \n");
    printf(" -I c_intf,s_intf\n");
    printf("    c_intf: control interface\n");
    printf("    s_intf: streaming interface\n");
    printf(" -t Trace level (0-6) \n");
    printf(" -q open iqserver\n");
    printf(" -h help message\n");
    printf("multi stram param, using ',' to seperate\n");
    printf("\nExample: %s -N2 -m0,0 -M1 -I0,1,2,3\n", argv[0]);
    printf("\n");
}

#define PARSE_PARM(instance)\
{\
    int i = 0; \
    do { \
        if (!i) {\
            p = strtok(optarg, ","); \
        } \
        if (p) {\
            instance[i++] = atoi(p); \
        } \
    } while((p = strtok(NULL, ","))!=NULL); \
}

int main(int argc, char **argv)
{
    char *p;
    MI_U64 uuid = 0;
    MI_U32 maxpacket[MAX_UVC_DEV_NUM] = {1024, 1024,192};
    MI_U8 mult[MAX_UVC_DEV_NUM] = {2, 2, 0},
          burst[MAX_UVC_DEV_NUM] = {0, 0, 0};
    MI_U8 intf[2 * MAX_UVC_DEV_NUM] = {0};
    MI_S32 result = 0, mode = UVC_MEMORY_MMAP;
    MI_S32 trace_level = UVC_DBG_ERR;
    struct sigaction sigAction;
    sigAction.sa_handler = ST_HandleSig;
    sigemptyset(&sigAction.sa_mask);
    sigAction.sa_flags = 0;
    sigaction(SIGINT, &sigAction, NULL);

    ST_DefaultArgs(&g_stConfig);
//set bitrate
    while((result = getopt(argc, argv, "a:A:b:B:M:N:m:p:I:t:i:qh")) != -1)
    {
        switch(result)
        {
            case 'a':
                g_stConfig.u8SnrPad  = strtol(optarg, NULL, 10);
                break;

            case 'A':
                g_stConfig.s8SnrResIndex = strtol(optarg, NULL, 10);
                break;

            case 'b':
                PARSE_PARM(g_bitrate);
                break;
            case 'B':
                PARSE_PARM(burst);
                break;
            case 'm':
                PARSE_PARM(mult);
                break;
            case 'M':
                mode = strtol(optarg, NULL, 10);
                break;
            case 'N':
                g_device_num = strtol(optarg, NULL, 10);
                break;
            case 'p':
                PARSE_PARM(maxpacket);
                break;
            case 'I':
                PARSE_PARM(intf);
                break;
            case 't':
                trace_level = strtol(optarg, NULL, 10);
                break;
            case 'q':
                g_enable_iqserver = TRUE;
                break;
            case 'h':
                help_message(argv);
                return 0;
            case 'i':
                strcpy(g_IspBinPath, optarg);
                ST_DBG("g_IspBinPath:%s\n", g_IspBinPath);
                g_load_iq_bin = TRUE;
                break;
            default: break;
        }
    }

    STCHECKRESULT(ST_BaseModuleInit(&g_stConfig));
    ST_UVC_SetTraceLevel(trace_level);
    ST_UvcInit(g_device_num, maxpacket, mult, burst, intf, mode);
    ST_DlaStart();

    while(!g_bExit)
    {
        usleep(100 * 1000);
#if USE_TEST
        ST_Test();
#endif

    }
    usleep(100 * 1000);
    ST_UvcDeinit();
    STCHECKRESULT(ST_BaseModuleUnInit());

    return 0;
}

