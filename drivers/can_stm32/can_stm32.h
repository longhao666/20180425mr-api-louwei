#ifndef _CAN_STM32_H_
#define _CAN_STM32_H_
#include "stm32f4xx.h"
#include "can_driver.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef CAN_HandleTypeDef* CAN_HANDLE;

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
