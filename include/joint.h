#ifndef __JOINT_H__
#define __JOINT_H__
#include "mrapi.h"
#include "module.h"

#define JOINT_OFFLINE 0
#define JOINT_ONLINE 1

typedef uint8_t rec_t[8];

typedef struct td_joint
{
	void* basicModule;
	uint16_t* jointId;
	uint16_t* jointType;
	uint8_t isOnline;

	rec_t txQue[MAX_BUFS];
	uint16_t txQueFront;
	uint16_t txQueRear;
	jQueShortHandler_t jointBufUnderflowHandler;
}Joint;

#endif

