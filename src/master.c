#include <math.h>
#include <sys/time.h>
#include "master.h"

#define UNUSED(arg) (void)arg

int32_t jointPeriodSend(void* tv);
void canDispatch(Module *d, Message *msg);

TASK_HANDLE hReceiveTask1;
TASK_HANDLE hReceiveTask2;

// Max two CAN Ports
CAN_HANDLE hCan1 = 0;
CAN_HANDLE hCan2 = 0;
CAN_HANDLE hCanUsed = 0;

uint8_t can1Send(Message* msg) {
  return canSend_driver(hCan1, msg);
}

uint8_t can2Send(Message* msg) {
  return canSend_driver(hCan2, msg);
}

/// CAN read thread or interrupt
void _canReadISR(Message* msg) {
  uint16_t cob_id = msg->cob_id;
  uint16_t id = geNodeId(cob_id);

  // assume module is a joint
  Joint* pJoint = jointSelect(id);
  if (pJoint->basicModule)
      canDispatch(pJoint->basicModule, msg);
}

int32_t startMaster(void) {
  // Open and Initiallize CAN Port
  hCan1 = canOpen_driver("pcan1", "1M");
  // Use CAN1 as the device
  hCanUsed = hCan1;

  // Create and Start thread to read CAN message
  CreateReceiveTask(hCan1, &hReceiveTask1, _canReadISR);

  StartTimerLoop(200, jointPeriodSend);

  return 0;
}

int32_t stopMaster(void) {
  StopTimerLoop();
  DestroyReceiveTask(&hReceiveTask1);
  return 0;
}

int32_t joinMaster(void) {
  WaitReceiveTaskEnd(&hReceiveTask1);
}

int32_t setControlLoopFreq(uint16_t hz) {
  float val = 1000000.0/(float)hz;
  setTimerInterval(round(val));
  return 0;
}

