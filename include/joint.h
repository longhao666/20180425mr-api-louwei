#ifndef __JOINT_H__
#define __JOINT_H__
#include "module.h"

#define MAX_JOINTS 10

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
#define isJointMode(t) (t==MODE_OPEN)||(t==MODE_CURRENT)||(t==MODE_SPEED)||(t==MODE_POSITION)

typedef struct td_joint
{
    Module* basicModule;
    uint16_t* jointId;
    uint16_t* jointType;
}Joint;

Joint*  jointUp(uint16_t id, canSend_t canSend);
int32_t jointDown(Joint* pJoint);
Joint*  jointSelect(uint16_t id);

int32_t jointGetId(Joint* pJoint, mCallback_t callBack);
int32_t jointGetType(Joint* pJoint, mCallback_t callBack);
int32_t jointGetVoltage(Joint* pJoint, mCallback_t callBack);

int32_t jointSetMode(Joint* pJoint, uint16_t mode,  mCallback_t callBack);

int32_t jointGetTypeTimeout(Joint* pJoint, int32_t timeout);


#endif

