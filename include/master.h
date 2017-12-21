#ifndef __MASTER_H__
#define __MASTER_H__
#include "joint.h"

#if defined CAN_PEAK_LINUX
  #include "can_peak_linux.h"
#elif defined PCAN_BASIC
  #include "pcan_basic.h"
#endif
#ifdef __cplusplus
extern "C"
{
#endif
uint8_t can1Send(Message* msg);
uint8_t can2Send(Message* msg);

int32_t startMaster(void);
int32_t stopMaster(void);
int32_t joinMaster(void);
int32_t setControlLoopFreq(int32_t hz);
#ifdef __cplusplus
}
#endif
#endif
