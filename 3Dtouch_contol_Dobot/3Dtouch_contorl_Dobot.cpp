
/*******************************************************************************
 ��������
*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <unistd.h>
# define Sleep(x) usleep((x) * 1000)
#endif



#include "DobotDll/DobotDll.h"
#include "DobotDll/DobotType.h"
#include "DobotDll/dobotdll_global.h"
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>

//#pragma comment(lib, "E:/software_document/VS2019/3Dtouch_control_Dobot/3Dtouch_contol_Dobot/lib/DobotDll.lib")


void InitDobot(void);
HDCallbackCode HDCALLBACK DevicePositionCallback(void* pUserData);
//void PrintDevicePosition();

/*******************************************************************************
 ������ Main function.
*******************************************************************************/
int main(int argc, char* argv[])
{
 //   Pose pose;
 //   float pose_x, pose_y, pose_z, pose_r, pose_jointAngle[4] = { 0 };
    JOGCmd jogcmd;
    CPCmd cpcmd;
    HOMECmd homecmd;
    HHD hHD;
    HDErrorInfo error;
    float xposition_init = 0, chazhi = 0;
    jogcmd.isJoint = 0;
 //Connect Dobot
    int ConnectDobot_result = ConnectDobot(argv[1], 115200, NULL, NULL, NULL);
    switch (ConnectDobot_result) {
    case DobotConnect_NoError:
        printf("Dobot connect success");
        break;
    case DobotConnect_NotFound:
        printf("Dobot not found!");
        return -2;
        break;
    case DobotConnect_Occupied:
        printf("Invalid port name or Dobot is occupied by other application!");
        return -3;
        break;
    default:
        break;
    }
//��ʼ��Dobot
//   InitDobot();
     SetHOMECmd(&homecmd, false, NULL);

//��ʼ��3Dtouch�豸
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        return -1;
    }
        printf("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

//��ʼ3Dtouch�豸���ȳ���
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        return -1;
    }

    printf("Press any key to quit.\n\n");
    while (!_kbhit())               //ͷ�ļ�Ϊconio.h   _kbhit()������keyoard������ʱ�ͻ�return 1
    {
        
        hduVector3Dd position;
        hdScheduleSynchronous(DevicePositionCallback, position,
            HD_DEFAULT_SCHEDULER_PRIORITY);
        printf("Device position: %.3f %.3f %.3f\n",
               position[0], position[1], position[2]);

        

        cpcmd.cpMode = 1;
        cpcmd.x = 200 + 0.8 * position[0];   //0.8Ϊ�Լ��趨�ı���ϵ��
        cpcmd.y = 0 - 0.8 * position[2] - 0.8*88.114;
        cpcmd.z = 0;
        int SetCPCmd_result = SetCPCmd(&cpcmd, 0, 0);
        switch (SetCPCmd_result) {
        case DobotCommunicate_NoError:
            printf("ָ����������");
            break;
        case DobotCommunicate_BufferFull:
            printf("ָ���������");
            return -2;
            break;
        case DobotCommunicate_Timeout:
            printf("ָ���޷��أ����³�ʱ");
            return -3;
            break;
        default:
            break;
        }
        SetQueuedCmdClear();
    }                              

    
 // Disconnect Dobot
    DisconnectDobot();
//�ر�3Dtouch���ȳ��򲢹ر�3Dtouch�豸
    hdStopScheduler();
    hdDisableDevice(hHD);

    return 0;
}

/*******************************************************************************
 �����ú���   Sub-function
*******************************************************************************/

HDCallbackCode HDCALLBACK DevicePositionCallback(void* pUserData)
{
    HDdouble* pPosition = (HDdouble*)pUserData;

    hdBeginFrame(hdGetCurrentDevice());
    hdGetDoublev(HD_CURRENT_POSITION, pPosition);
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_DONE;
}



void InitDobot(void)
{
    // Command timeout
    SetCmdTimeout(3000);
    // Clear old commands and set the queued command running
    SetQueuedCmdClear();
    SetQueuedCmdStartExec();

    // Device SN  ��ȡ��е�����к�
    char deviceSN[64];
    GetDeviceSN(deviceSN, sizeof(deviceSN));
    printf("Device SN:%s\r\n", deviceSN);

    // Device Name ��ȡ��е������
    char deviceName[64];
    GetDeviceName(deviceName, sizeof(deviceName));
    printf("Device Name:%s\r\n", deviceName);

    // Device version information  ��ȡ��е�۰汾��Ϣ
    uint8_t majorVersion, minorVersion, revision, hwVersion;
    GetDeviceVersion(&majorVersion, &minorVersion, &revision, &hwVersion);
    printf("Device information:V%d.%d.%d\r\n", majorVersion, minorVersion, revision);

    // Set the end effector parameters   ����ĩ�˼о߲���
    EndEffectorParams endEffectorParams;
    memset(&endEffectorParams, 0, sizeof(EndEffectorParams));
    endEffectorParams.xBias = 71.6f;
    SetEndEffectorParams(&endEffectorParams, false, NULL);

    // 1. Set the JOG parameters   ���ùؽ��˶�����
    JOGJointParams jogJointParams;
    for (uint32_t i = 0; i < 4; i++) {
        jogJointParams.velocity[i] = 200;
        jogJointParams.acceleration[i] = 200;
    }
    SetJOGJointParams(&jogJointParams, false, NULL);

    //���õѿ�������ϵ�㶯�ٶ�
    JOGCoordinateParams jogCoordinateParams;
    for (uint32_t i = 0; i < 4; i++) {
        jogCoordinateParams.velocity[i] = 200;
        jogCoordinateParams.acceleration[i] = 200;
    }
    SetJOGCoordinateParams(&jogCoordinateParams, false, NULL);

    //����ʾ�������ٶ�����ٶȰٷֱȣ��������ã�Ĭ��Ϊ 50%
    JOGCommonParams jogCommonParams;
    jogCommonParams.velocityRatio = 50;
    jogCommonParams.accelerationRatio = 50;
    SetJOGCommonParams(&jogCommonParams, false, NULL);

    // 2. Set the PTP parameters
    //����ʾ�����ֵĹؽ�����ϵ�˶�����
    PTPJointParams ptpJointParams;
    for (uint32_t i = 0; i < 4; i++) {
        ptpJointParams.velocity[i] = 200;
        ptpJointParams.acceleration[i] = 200;
    }
    SetPTPJointParams(&ptpJointParams, false, NULL);

    //����ʾ�����ֵĵѿ�������ϵ�˶�������
    PTPCoordinateParams ptpCoordinateParams;
    ptpCoordinateParams.xyzVelocity = 200;
    ptpCoordinateParams.xyzAcceleration = 200;
    ptpCoordinateParams.rVelocity = 200;
    ptpCoordinateParams.rAcceleration = 200;
    SetPTPCoordinateParams(&ptpCoordinateParams, false, NULL);

    //����ʾ�������� JUMP �˶�ģʽʱ���˶�����
    PTPJumpParams ptpJumpParams;
    ptpJumpParams.jumpHeight = 10;
    ptpJumpParams.zLimit = 150;
    SetPTPJumpParams(&ptpJumpParams, false, NULL);

    // 3. Set the CP parameters
    CPParams cpparams;
    cpparams.acc = 200;
    cpparams.juncitionVel = 200;
    cpparams.realTimeTrack = 1;
    SetCPParams(&cpparams, 0, 0);
}

/*******************************************************************************
 ������������δʹ�õ�����
*******************************************************************************/

//����豸ĩ��λ��
//void PrintDevicePosition()
//{
//    hduVector3Dd position;
//    hdScheduleSynchronous(DevicePositionCallback, position,
//        HD_DEFAULT_SCHEDULER_PRIORITY);
//
//    printf("Device position: %.3f %.3f %.3f\n",
//        position[0], position[1], position[2]);
//}


 //��ȡDobotĩ��λ��   
    //while (1)
    //{
    //    int GetPose_result = GetPose(&pose);
    //    if (GetPose_result == DobotCommunicate_NoError) {
    //        pose_x = pose.x;
    //        pose_y = pose.y;
    //        pose_z = pose.z;
    //        pose_r = pose.r;
    //        for (int i = 0; i < 4; i++) {
    //            pose_jointAngle[i] = pose.jointAngle[i];
    //        }
    //    }
    //    printf("Device position: %.3f %.3f %.3f\n", pose_x, pose_y, pose_z);
    //}

//chazhi = position[0] - xposition_init;
//if (abs(chazhi) > 5)
//{
//    if (chazhi > 5)
//    {
//        jogcmd.cmd = 1;
//        xposition_init = position[0];
//    }
//    else
//    {
//        jogcmd.cmd = 2;
//        xposition_init = position[0];
//    }
//}
//else
//{
//    jogcmd.cmd = 0;
//}
//SetJOGCmd(&jogcmd, 0, 0);
//printf("chazhi=%.3f\n", chazhi);