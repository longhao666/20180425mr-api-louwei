#include <math.h>
#include <sys/time.h>
#include "master.h"

#define UNUSED(arg) (void)arg

TASK_HANDLE hReceiveTask1;
TASK_HANDLE hReceiveTask2;

Joint* jointStack[MAX_JOINTS];
uint16_t jointNbr = 0;

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

Joint* findJoint(uint16_t id) {
  uint16_t i;
  for (i = 0; i < jointNbr; i++) {
    if (id == *(jointStack[i]->jointId))
      return jointStack[i];
  }

  return 0;
}

int32_t delJoint(Joint* pJoint) {
  uint16_t i;
  if (!jointNbr) {
    MSG_ERROR("Joint Stack Underflow");
    return -2;
  }
  if(!pJoint) {
    MSG_ERROR("Joint is NULL");
    return -3;
  }

  for (i = 0; i < jointNbr; i++) {
    if (pJoint == jointStack[i])
      break;
  }
  for (; i < jointNbr - 1; i++) {
    jointStack[i] = jointStack[i+1];
  }
  jointStack[jointNbr--] = 0;
  jointDestroy(pJoint);
  return 0;
}

int32_t addJoint(Joint* pJoint) {
  if (jointNbr >= MAX_JOINTS) {
    MSG_ERROR("Joint Stack Overflow");
    return -1;   
  }
  else {
    if (pJoint != findJoint(*(pJoint->jointId)))
      jointStack[jointNbr++] = pJoint;
    else return -1;
  }
  return 0;
}

void scanJoint(uint16_t id) {
  Joint* pJoint = jointInit(id, can1Send);
  jointGetId(pJoint, 0);
  jointDestroy(pJoint);
}

int32_t onTypeUpdate(void* handle, void* args) {
  UNUSED(args);
  Joint* pJoint = (Joint*)handle;
  if (isJointType(*(pJoint->jointType))){
    addJoint(pJoint);
    return 0;  
  }
  // Not a Joint
  delJoint(pJoint);
  return -1;
}

/// CAN read thread or interrupt
void _canReadISR(Message* msg) {
  uint16_t cob_id = msg->cob_id;
  uint16_t id = geNodeId(cob_id);

  // assume module is a joint
 Joint* pJoint = findJoint(id);
 if (!pJoint) {

    /// Here a joint is initiallized, and memory is attributed, BUT if
    /// no reply of jointGetType, memory cannot be released
    pJoint = jointInit(id, can1Send);
    // addJoint(pJoint);
    jointGetType(pJoint, onTypeUpdate);

    // pGripper = gripperInit(id, can2Send);
    // gripperGetType(pGripper, onTypeUpdate2);
  }
  jointMsgRoute(pJoint, msg);
}

int32_t _proceedPeriodSend(struct timeval* tv) {
  for (int16_t i = 0; i < jointNbr; i++) {
    jointSendPVTSync(jointStack[i], 0x11, 0x22);
  }

}

int32_t startMaster(void) {

  // Open and Initiallize CAN Port
  hCan1 = canOpen_driver("pcan1", "1M");
  // Use CAN1 as the device
  hCanUsed = hCan1;

  // Create and Start thread to read CAN message
  CreateReceiveTask(hCan1, &hReceiveTask1, _canReadISR);

  StartTimerLoop(200, _proceedPeriodSend);

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

// int32_t setTimerCallback(uint8_t ms, void* timerPtr) {
//   setTimerCb_driver(ms, timerPtr);
// }

