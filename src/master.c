#include <math.h>
#include "master.h"

#define UNUSED(arg) (void)arg

int32_t jointPeriodSend(void* tv);
void canDispatch(Module *d, Message *msg);

CAN_HANDLE hCan[MAX_CAN_DEVICES] = { 0 };

uint8_t can1Send(Message* msg) { return canSend_driver(hCan[0], msg); }
uint8_t can2Send(Message* msg) { return canSend_driver(hCan[1], msg); }
uint8_t can3Send(Message* msg) { return canSend_driver(hCan[2], msg); }
uint8_t can4Send(Message* msg) { return canSend_driver(hCan[3], msg); }
uint8_t can5Send(Message* msg) {/* return canSend_driver(hCan[3], msg); */}
uint8_t can6Send(Message* msg) {/* return canSend_driver(hCan[3], msg); */}

// Max 4 CAN Ports
TASK_HANDLE hReceiveTask[MAX_CAN_DEVICES];
canSend_t hCansendHandler[MAX_CAN_DEVICES] = { can1Send, can2Send, can3Send, can4Send };

/// CAN read thread or interrupt
void _canReadISR(Message* msg) {
  uint16_t cob_id = msg->cob_id;
  uint16_t id = getNodeId(cob_id);

  // assume module is a joint
  Joint* pJoint = jointSelect(id);
  if (pJoint && pJoint->basicModule)
      canDispatch(pJoint->basicModule, msg);
}

int32_t __stdcall startMaster(char* busname, uint8_t masterId) {
  // Open and Initiallize CAN Port
  if (hCan[masterId] != 0) {
	  ELOG("masterId %d has been combined to CAN device HANDLE 0x%X", masterId, hCan[masterId]);
	  return MR_ERROR_ILLDATA;
  }
  hCan[masterId] = canOpen_driver(busname, "1M");
  // Use CAN1 as the device

  // Create and Start thread to read CAN message
  CreateReceiveTask(hCan[masterId], &hReceiveTask[masterId], _canReadISR);

  StartTimerLoop(-1, jointPeriodSend);

  return MR_ERROR_OK;
}

int32_t __stdcall stopMaster(uint8_t masterId) {
  hCan[masterId] = 0;
  StopTimerLoop();
  DestroyReceiveTask(&hReceiveTask[masterId]);
  return MR_ERROR_OK;
}

//void* __stdcall masterLoadSendFunction(uint8_t masterId) {
//	switch (masterId) {
//	case 0: return (void*)can1Send;
//	case 1: return (void*)can2Send;
//	case 2: return (void*)can3Send;
//	case 3: return (void*)can4Send;
//	}
//	return NULL;
//}

int32_t __stdcall joinMaster(uint8_t masterId) {
  WaitReceiveTaskEnd(&hReceiveTask[masterId]);
  DestroyReceiveTask(&hReceiveTask[masterId]);
  return MR_ERROR_OK;
}

int32_t __stdcall setControlLoopFreq(int32_t hz) {
  float val = 0;
  if (hz != -1) {
      val = 1000000.0f/(float)hz;
  }
  setTimerInterval(round(val));
  return MR_ERROR_OK;
}

