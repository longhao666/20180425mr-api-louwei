#ifndef __MASTER_H__
#define __MASTER_H__
#include "joint.h"

#if defined CAN_PEAK_LINUX
  #include "can_peak_linux.h"
#elif defined PCAN_BASIC
  #include "pcan_basic.h"
#endif


#define MAX_JOINTS 10

int32_t addJoint(Joint* pJoint);
int32_t delJoint(Joint* pJoint);
Joint* findJoint(uint16_t id);
void scanJoint(uint16_t id);
void canReadThread(Message* msg);
int32_t startMaster(void);
int32_t stopMaster(void);
int32_t setTimerCallback(uint8_t ms, void* timerPtr);

#endif