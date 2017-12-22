#ifndef __MASTER_H__
#define __MASTER_H__
#include "joint.h"

#if defined CAN_PEAK_LINUX
  #include "can_peak_linux.h"
#elif defined PCAN_BASIC
  #include "pcan_basic.h"
#endif

#define MAX_CAN_DEVICES 4

#ifdef __cplusplus
extern "C"
{
#endif

int32_t startMaster(uint8_t masterId);
int32_t stopMaster(uint8_t masterId);
int32_t joinMaster(uint8_t masterId);
int32_t setControlLoopFreq(int32_t hz);
canSend_t masterLoadSendFunction(uint8_t masterId);

#ifdef __cplusplus
}
#endif
#endif
