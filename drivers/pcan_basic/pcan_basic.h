#ifndef __PCAN_BASIC_H__
#define __PCAN_BASIC_H__

#include "can_driver.h"
#include <PCANBasic.h>

#define TIMEVAL unsigned long long
#define TIMEVAL_MAX ~(TIMEVAL)0
#define MS_TO_TIMEVAL(ms) ms*1000L
#define US_TO_TIMEVAL(us) us

#define CAN_HANDLE TPCANHandle


void CreateReceiveTask(CAN_HANDLE fd0, TASK_HANDLE* Thread, void* ReceiveLoopPtr);
void DestroyReceiveTask(TASK_HANDLE* Thread);
void WaitReceiveTaskEnd(TASK_HANDLE* Thread);

uint8_t canChangeBaudRate_driver(CAN_HANDLE fd, char* baud);
CAN_HANDLE canOpen_driver(char* busno, char* baud);
void canReset_driver(CAN_HANDLE handle, char* baud);
uint8_t canSend_driver(CAN_HANDLE fd0, Message const *m);
uint8_t canReceive_driver(CAN_HANDLE fd0, Message *m);

void setTimerInterval(uint32_t t);
// int32_t setTimerCb_driver(uint8_t ms, void* timerPtr);
void StartTimerLoop(int32_t hz, void* periodCall);
void StopTimerLoop(void);

#endif
