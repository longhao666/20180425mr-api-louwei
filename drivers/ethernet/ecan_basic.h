#ifndef __PCAN_BASIC_H__
#define __PCAN_BASIC_H__

#include "can_driver.h"

#define TIMEVAL unsigned long long
#define TIMEVAL_MAX ~(TIMEVAL)0
#define MS_TO_TIMEVAL(ms) ms*1000L
#define US_TO_TIMEVAL(us) us

#define CAN_HANDLE uint8_t

#ifdef __cplusplus
extern "C"
{
#endif

void CreateReceiveTask(CAN_HANDLE fd0, TASK_HANDLE* Thread, void* ReceiveLoopPtr);
void DestroyReceiveTask(TASK_HANDLE* Thread);
void WaitReceiveTaskEnd(TASK_HANDLE* Thread);

uint8_t canChangeBaudRate_driver(CAN_HANDLE fd, char* baud);
CAN_HANDLE canOpen_driver(const char* busno, const char* baud);
void canReset_driver(CAN_HANDLE handle, char* baud);
uint8_t canSend_driver(CAN_HANDLE fd0, Message const *m);
uint8_t canReceive_driver(CAN_HANDLE fd0, Message *m);
int canClose_driver(CAN_HANDLE handle);

#ifdef __cplusplus
}
#endif

#endif
