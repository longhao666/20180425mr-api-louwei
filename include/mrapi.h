#ifndef _MRAPI_H_
#define _MRAPI_H_
#include <stdint.h>

#define DEBUG_MSG_CONSOLE_ON
//#define DEBUG_MSG_CONSOLE_OFF
#define LOG_LEVEL 1
#define MAX_CAN_DEVICES 4

////* JOINTS *///
#define MAX_JOINTS 10
#define MAX_BUFS 128
#define WARNING_BUFS 20

//ģ�����ͺ궨��
#define MODEL_TYPE_M14        0x010
#define MODEL_TYPE_M17        0x020
#define MODEL_TYPE_M17V2      0x021
#define MODEL_TYPE_M20        0x030
#define MODEL_TYPE_M20V2      0x031
#define MODEL_TYPE_M20E       0x031
#define MODEL_TYPE_LIFT       0x040

//������ģʽ����
#define MODE_OPEN             0       //����ģʽ
#define MODE_CURRENT          1       //����ģʽ
#define MODE_SPEED            2       //�ٶ�ģʽ
#define MODE_POSITION         3       //λ��ģʽ
#define MODE_CYCLESYNC        4       //ѭ��ͬ��

//�����ֽ�MASK����
#define ERROR_MASK_OVER_CURRENT   0x0001    //����
#define ERROR_MASK_OVER_VOLTAGE   0x0002    //��ѹ
#define ERROR_MASK_UNDER_VOLTAGE  0x0004    //Ƿѹ
#define ERROR_MASK_OVER_TEMP      0x0008    //����
#define ERROR_MASK_BATTERY        0x0010    //��������ش���
#define ERROR_MASK_ENCODER        0x0020    //���̴���
//#define ERROR_MASK_POTEN          0x0040    //��λ������
#define ERROR_MASK_CURRENT_INIT   0x0080    //����������
//#define ERROR_MASK_FUSE           0x0100    //����˿�Ͽ�����

#define isJointType(t) (t==MODEL_TYPE_M14)||(t==MODEL_TYPE_M17)||(t==MODEL_TYPE_M17V2)||(t==MODEL_TYPE_M20)||(t==MODEL_TYPE_M20V2) \
						||(t==MODEL_TYPE_M20E)||(t==MODEL_TYPE_LIFT)
#define isJointMode(t) (t==MODE_OPEN)||(t==MODE_CURRENT)||(t==MODE_SPEED)||(t==MODE_POSITION)||(t==MODE_CYCLESYNC)

typedef void* JOINT_HANDLE;
typedef int32_t(*jQueShortHandler_t)(JOINT_HANDLE pJoint, uint16_t len);
typedef int32_t(*jCallback_t)(uint16_t id, uint16_t index, void* args);

#ifdef __cplusplus
extern "C"
{
#endif
	int32_t __stdcall startMaster(uint8_t masterId);
	int32_t __stdcall stopMaster(uint8_t masterId);
	int32_t __stdcall joinMaster(uint8_t masterId);
	int32_t __stdcall setControlLoopFreq(int32_t hz);
	void* __stdcall masterLoadSendFunction(uint8_t masterId);

	JOINT_HANDLE __stdcall jointUp(uint16_t id, void* canSend); //construct Joint and put it in joint stack
	int32_t      __stdcall jointDown(JOINT_HANDLE pJoint);          //destruct joint and remove it from joint stack
	JOINT_HANDLE __stdcall jointSelect(uint16_t id);  //find joint by it's ID

	void __stdcall jointStartServo(JOINT_HANDLE pJoint, jQueShortHandler_t handler);
	void __stdcall jointStopServo(JOINT_HANDLE pJoint);

	int32_t __stdcall jointPush(JOINT_HANDLE pJoint, uint8_t* buf);
	int32_t __stdcall jointPoll(JOINT_HANDLE pJoint, uint8_t* buf);

	int32_t __stdcall jointGetId(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack);
	int32_t __stdcall jointGetType(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack);
	int32_t __stdcall jointGetVoltage(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack);

	int32_t __stdcall jointSetMode(JOINT_HANDLE pJoint, uint16_t mode, int32_t timeout, jCallback_t callBack);

#ifdef __cplusplus
}
#endif
#endif

