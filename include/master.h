#ifndef __MASTER_H__
#define __MASTER_H__
#include "joint.h"
#include "gripper.h"

#if defined PCAN_BASIC
  #include "pcan_basic.h"
#endif

#if defined ECAN_BASIC
    #include "ecan_basic.h"
#endif

#if defined CAN_STM32
    #include "can_stm32.h"
#endif

#endif
