#ifndef DOBOTDLL_H
#define DOBOTDLL_H

#include "DobotType.h"

extern "C" int DobotExec(void);

extern "C" int SearchDobot(char *dobotNameList, uint32_t maxLen);
extern "C" int ConnectDobot(const char *portName, uint32_t baudrate, char *fwType, char *version);
extern "C" int DisconnectDobot(void);

extern "C" int SetCmdTimeout(uint32_t cmdTimeout);

// Device information
extern "C" int SetDeviceSN(const char *deviceSN);
extern "C" int GetDeviceSN(char *deviceSN, uint32_t maxLen);

extern "C" int SetDeviceName(const char *deviceName);
extern "C" int GetDeviceName(char *deviceName, uint32_t maxLen);

extern "C" int GetDeviceVersion(uint8_t *majorVersion, uint8_t *minorVersion, uint8_t *revision);

// Pose and Kinematics parameters are automatically get
extern "C" int GetPose(Pose *pose);
extern "C" int ResetPose(bool manual, float rearArmAngle, float frontArmAngle);
extern "C" int GetKinematics(Kinematics *kinematics);

// Alarms
extern "C" int GetAlarmsState(uint8_t *alarmsState, uint32_t *len, uint32_t maxLen);
extern "C" int ClearAllAlarmsState(void);

// HOME
extern "C" int SetHOMEParams(HOMEParams *homeParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetHOMEParams(HOMEParams *homeParams);

extern "C" int SetHOMECmd(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex);

// Handheld teach
extern "C" int SetHHTTrigMode(HHTTrigMode hhtTrigMode);
extern "C" int GetHHTTrigMode(HHTTrigMode *hhtTrigMode);

extern "C" int SetHHTTrigOutputEnabled(bool isEnabled);
extern "C" int GetHHTTrigOutputEnabled(bool *isEnabled);

extern "C" int GetHHTTrigOutput(bool *isTriggered);

// EndEffector
extern "C" int SetEndEffectorParams(EndEffectorParams *endEffectorParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetEndEffectorParams(EndEffectorParams *endEffectorParams);

extern "C" int SetEndEffectorLaser(bool enableCtrl, bool on, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetEndEffectorLaser(bool *isCtrlEnabled, bool *isOn);

extern "C" int SetEndEffectorSuctionCup(bool enableCtrl, bool suck, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetEndEffectorSuctionCup(bool *isCtrlEnabled, bool *isSucked);

extern "C" int SetEndEffectorGripper(bool enableCtrl, bool grip, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetEndEffectorGripper(bool *isCtrlEnabled, bool *isGripped);

// Arm orientation
extern "C" int SetArmOrientation(ArmOrientation armOrientation, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetArmOrientation(ArmOrientation *armOrientation);

// JOG functions
extern "C" int SetJOGJointParams(JOGJointParams *jointJogParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetJOGJointParams(JOGJointParams *jointJogParams);

extern "C" int SetJOGCoordinateParams(JOGCoordinateParams *coordinateJogParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetJOGCoordinateParams(JOGCoordinateParams *coordinateJogParams);

extern "C" int SetJOGCommonParams(JOGCommonParams *jogCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetJOGCommonParams(JOGCommonParams *jogCommonParams);
extern "C" int SetJOGCmd(JOGCmd *jogCmd, bool isQueued, uint64_t *queuedCmdIndex);

// PTP functions
extern "C" int SetPTPJointParams(PTPJointParams *ptpJointParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetPTPJointParams(PTPJointParams *ptpJointParams);
extern "C" int SetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams);
extern "C" int SetPTPJumpParams(PTPJumpParams *ptpJumpParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetPTPJumpParams(PTPJumpParams *ptpJumpParams);
extern "C" int SetPTPCommonParams(PTPCommonParams *ptpCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetPTPCommonParams(PTPCommonParams *ptpCommonParams);

extern "C" int SetPTPCmd(PTPCmd *ptpCmd, bool isQueued, uint64_t *queuedCmdIndex);

// CP functions
extern "C" int SetCPParams(CPParams *cpParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetCPParams(CPParams *cpParams);
extern "C" int SetCPCmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int SetCPLECmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex);

// ARC
extern "C" int SetARCParams(ARCParams *arcParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetARCParams(ARCParams *arcParams);
extern "C" int SetARCCmd(ARCCmd *arcCmd, bool isQueued, uint64_t *queuedCmdIndex);

// WAIT
extern "C" int SetWAITCmd(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex);

// TRIG
extern "C" int SetTRIGCmd(TRIGCmd *trigCmd, bool isQueued, uint64_t *queuedCmdIndex);

// EIO
extern "C" int SetIOMultiplexing(IOMultiplexing *ioMultiplexing, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetIOMultiplexing(IOMultiplexing *ioMultiplexing);

extern "C" int SetIODO(IODO *ioDO, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetIODO(IODO *ioDO);

extern "C" int SetIOPWM(IOPWM *ioPWM, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" int GetIOPWM(IOPWM *ioPWM);

extern "C" int GetIODI(IODI *ioDI);
extern "C" int GetIOADC(IOADC *ioADC);

extern "C" int SetEMotor(EMotor *eMotor, bool isQueued, uint64_t *queuedCmdIndex);

// CAL
extern "C" int SetAngleSensorStaticError(float rearArmAngleError, float frontArmAngleError);
extern "C" int GetAngleSensorStaticError(float *rearArmAngleError, float *frontArmAngleError);
extern "C" int SetAngleSensorCoef(float rearArmAngleCoef, float frontArmAngleCoef);
extern "C" int GetAngleSensorCoef(float *rearArmAngleCoef, float *frontArmAngleCoef);

extern "C" int SetBaseDecoderStaticError(float baseDecoderError);
extern "C" int GetBaseDecoderStaticError(float *baseDecoderError);

// WIFI
extern "C" int SetWIFIConfigMode(bool enable);
extern "C" int GetWIFIConfigMode(bool *isEnabled);
extern "C" int SetWIFISSID(const char *ssid);
extern "C" int GetWIFISSID(char *ssid, uint32_t maxLen);
extern "C" int SetWIFIPassword(const char *password);
extern "C" int GetWIFIPassword(char *password, uint32_t maxLen);
extern "C" int SetWIFIIPAddress(WIFIIPAddress *wifiIPAddress);
extern "C" int GetWIFIIPAddress(WIFIIPAddress *wifiIPAddress);
extern "C" int SetWIFINetmask(WIFINetmask *wifiNetmask);
extern "C" int GetWIFINetmask(WIFINetmask *wifiNetmask);
extern "C" int SetWIFIGateway(WIFIGateway *wifiGateway);
extern "C" int GetWIFIGateway(WIFIGateway *wifiGateway);
extern "C" int SetWIFIDNS(WIFIDNS *wifiDNS);
extern "C" int GetWIFIDNS(WIFIDNS *wifiDNS);
extern "C" int GetWIFIConnectStatus(bool *isConnected);

// TEST
extern "C" int GetUserParams(UserParams *userParams);
extern "C" int GetPTPTime(PTPCmd *ptpCmd, uint32_t *ptpTime);

// Queued command
extern "C" int SetQueuedCmdStartExec(void);
extern "C" int SetQueuedCmdStopExec(void);
extern "C" int SetQueuedCmdForceStopExec(void);
extern "C" int SetQueuedCmdStartDownload(uint32_t totalLoop, uint32_t linePerLoop);
extern "C" int SetQueuedCmdStopDownload(void);
extern "C" int SetQueuedCmdClear(void);
extern "C" int GetQueuedCmdCurrentIndex(uint64_t *queuedCmdCurrentIndex);

#endif // DOBOTDLL_H
