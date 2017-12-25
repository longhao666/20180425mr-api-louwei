#ifndef __MASTER_H__
#define __MASTER_H__
#include "joint.h"

#if defined CAN_PEAK_LINUX
  #include "can_peak_linux.h"
#elif defined PCAN_BASIC
  #include "pcan_basic.h"
#endif


#endif
