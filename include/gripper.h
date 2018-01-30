#ifndef __GRIPPER_H__
#define __GRIPPER_H__
#include "mrapi.h"
#include "module.h"

typedef struct td_gripper
{
	Module* basicModule;
	uint16_t* gripperId;
	uint16_t* gripperType;
	uint8_t isOnline;
}Gripper;

#endif