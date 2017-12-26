#include <math.h>
#include "master.h"

#define UNUSED(arg) (void)arg

int32_t jointPeriodSend(void* tv);
void canDispatch(Module *d, Message *msg);

// Max 4 CAN Ports
CAN_HANDLE hCan[MAX_CAN_DEVICES] = { 0 };
TASK_HANDLE hReceiveTask[MAX_CAN_DEVICES];

uint8_t can1Send(Message* msg) { return canSend_driver(hCan[0], msg);}
uint8_t can2Send(Message* msg) { return canSend_driver(hCan[1], msg); }
uint8_t can3Send(Message* msg) { return canSend_driver(hCan[2], msg); }
uint8_t can4Send(Message* msg) { return canSend_driver(hCan[3], msg); }

/// CAN read thread or interrupt
void _canReadISR(Message* msg) {
  uint16_t cob_id = msg->cob_id;
  uint16_t id = getNodeId(cob_id);

  // assume module is a joint
  Joint* pJoint = jointSelect(id);
  if (pJoint && pJoint->basicModule)
      canDispatch(pJoint->basicModule, msg);
}

int32_t startMaster(uint8_t masterId) {
  // Open and Initiallize CAN Port
  hCan[masterId] = canOpen_driver("pcan1", "1M");
  // Use CAN1 as the device
  ILOG("Hello world");

  // Create and Start thread to read CAN message
  CreateReceiveTask(hCan[masterId], &hReceiveTask[masterId], _canReadISR);

  StartTimerLoop(-1, jointPeriodSend);

  return MR_ERROR_OK;
}

int32_t stopMaster(uint8_t masterId) {
  StopTimerLoop();
  DestroyReceiveTask(&hReceiveTask[masterId]);
  return MR_ERROR_OK;
}

void* masterLoadSendFunction(uint8_t masterId) {
	switch (masterId) {
	case 0: return (void*)can1Send;
	case 1: return (void*)can2Send;
	case 2: return (void*)can3Send;
	case 3: return (void*)can4Send;
	}
	return NULL;
}

int32_t joinMaster(uint8_t masterId) {
  WaitReceiveTaskEnd(&hReceiveTask[masterId]);
  DestroyReceiveTask(&hReceiveTask[masterId]);
  return MR_ERROR_OK;
}

int32_t setControlLoopFreq(int32_t hz) {
  float val = 0;
  if (hz != -1) {
      val = 1000000.0f/(float)hz;
  }
  setTimerInterval(round(val));
  return MR_ERROR_OK;
}

