#ifndef __JOINT_H__
#define __JOINT_H__
#include "module.h"

#define MAX_JOINTS 10
#define MAX_BUFS 128
#define WARNING_BUFS 20

#define JOINT_OFFLINE 0
#define JOINT_ONLINE 0

//模块类型宏定义
#define MODEL_TYPE_M14        0x010
#define MODEL_TYPE_M17        0x020
#define MODEL_TYPE_M17V2      0x021
#define MODEL_TYPE_M20        0x030
#define MODEL_TYPE_M20V2      0x031
#define MODEL_TYPE_M20E       0x031
#define MODEL_TYPE_LIFT       0x040

//驱动器模式定义
#define MODE_OPEN             0       //开环模式
#define MODE_CURRENT          1       //电流模式
#define MODE_SPEED            2       //速度模式
#define MODE_POSITION         3       //位置模式
#define MODE_CYCLESYNC        4       //循环同步

//错误字节MASK定义
#define ERROR_MASK_OVER_CURRENT   0x0001    //过流
#define ERROR_MASK_OVER_VOLTAGE   0x0002    //过压
#define ERROR_MASK_UNDER_VOLTAGE  0x0004    //欠压
#define ERROR_MASK_OVER_TEMP      0x0008    //过温
#define ERROR_MASK_BATTERY        0x0010    //编码器电池错误
#define ERROR_MASK_ENCODER        0x0020    //码盘错误
//#define ERROR_MASK_POTEN          0x0040    //电位器错误
#define ERROR_MASK_CURRENT_INIT   0x0080    //电流检测错误
//#define ERROR_MASK_FUSE           0x0100    //保险丝断开错误

#define isJointType(t) (t==MODEL_TYPE_M14)||(t==MODEL_TYPE_M17)||(t==MODEL_TYPE_M17V2)||(t==MODEL_TYPE_M20)||(t==MODEL_TYPE_M20V2) \
						||(t==MODEL_TYPE_M20E)||(t==MODEL_TYPE_LIFT)
#define isJointMode(t) (t==MODE_OPEN)||(t==MODE_CURRENT)||(t==MODE_SPEED)||(t==MODE_POSITION)||(t==MODE_CYCLESYNC)

typedef uint8_t rec_t[8];
typedef int32_t (*jointBufHandler_t)(void* handle, uint16_t len);

typedef struct td_joint
{
    Module* basicModule;
    uint16_t* jointId;
    uint16_t* jointType;
    uint8_t isOnline;

    rec_t txQue[MAX_BUFS];
    uint16_t txQueFront;
    uint16_t txQueRear;
    jointBufHandler_t jointBufUnderflowHandler;
}Joint;

Joint*  jointUp(uint16_t id, canSend_t canSend); //construct Joint and put it in joint stack
int32_t jointDown(Joint* pJoint);          //destruct joint and remove it from joint stack
Joint*  jointSelect(uint16_t id);  //find joint by it's ID

void jointStartServo(Joint* pJoint, jointBufHandler_t handler);
void jointStopServo(Joint* pJoint);

int32_t jointPush(Joint* pJoint, uint8_t* buf);
int32_t jointPoll(Joint* pJoint, uint8_t* buf);

int32_t jointGetId(Joint* pJoint, mCallback_t callBack);
int32_t jointGetType(Joint* pJoint, mCallback_t callBack);
int32_t jointGetVoltage(Joint* pJoint, mCallback_t callBack);

int32_t jointSetMode(Joint* pJoint, uint16_t mode,  mCallback_t callBack);

int32_t jointGetIdTimeout(Joint* pJoint, int32_t timeout);
int32_t jointGetTypeTimeout(Joint* pJoint, int32_t timeout);

int32_t jointSetModeTimeout(Joint* pJoint, uint16_t mode, int32_t timeout);
#endif

