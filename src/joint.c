#include <stdlib.h>
#include <string.h>
#include "joint.h"

#define JOINT_OFFLINE 0
#define JOINT_ONLINE 1

//内存控制表读写权限
const uint8_t joint_accessType[10][16] = 
{
    {//0x0*字段
        RO,			//字头
        RW,			//驱动器ID
        RO,			//驱动器型号
        RO,			//固件版本
        RO,			//错误代码
        RO,			//系统电压
        RO,			//系统温度
        RO,			//模块减速比
        RW,			//232端口波特率（本版本已移除）
        RW,			//CAN总线波特率
        RW,			//使能驱动器标志
        RW,			//上电使能驱动器标志
        RW,			//保存数据到Flash标志
        RW,			//下次上电进入bootloader
        RW,			//将当前位置设置为零点标志
        RW,			//清除错误标志
    },
    {//0x1*字段
        RO,			//当前电流低16位（mA）
        RO,			//当前电流高16位（mA）
        RO,			//当前速度低16位（units/s）
        RO,			//当前速度高16位（units/s）
        RO,			//当前位置低16位（units）
        RO,			//当前位置高16位（units）
        NO_ACCESS,			//数字电位器值（本版本已移除）
        RO,			//零点位置偏移量低16位（units）
        RO,			//零点位置偏移量高16位（units）
    },
    {//0x2*字段
        RO,			//电机内阻
        RO,			//电机电感
        RO,			//电机额定电压
        RO,			//电机额定电流
        NO_ACCESS,			//码盘线数（本版本已移除）
        NO_ACCESS,			//当前霍尔状态（本版本已移除）
        RO,      //绝对编码器单圈数据
        RO,      //绝对编码器多圈数据
        RO,      //多圈状态信息
        RO,      //电池电压
        RO,      //X
        RO,      //Y
        RO,      //Z
    },
    {//0x3*字段
        RW,			//工作模式，0-开环，1-电流模式，2-速度模式，3-位置模式
        RW,			//开环模式下占空比（0~100）
        RW,			//目标电流低16位（mA）
        RW,			//目标电流高16位（mA）
        RW,			//目标速度低16位（units/s）
        RW,			//目标速度高16位（units/s）
        RW,			//目标位置低16位（units）
        RW,			//目标位置高16位（units）
    },
    {//0x4*字段
        RW,			//最大电流（mA）
        RW,			//最大速度（rpm）
        RW,			//最大加速度（rpm/s）
        RW,			//最小位置低16位（units）
        RW,			//最小位置高16位（units）
        RW,			//最大位置低16位（units）
        RW,			//最大位置高16位（units）
    },
    {//0x5*字段
        RW,			//三闭环参数锁定标志
        RW,			//电流环P参数
        RW,			//电流环I参数
        RW,			//电流环D参数
        RW,			//速度环P参数
        RW,			//速度环I参数
        RW,			//速度环D参数
        RW,			//速度死区
        RW,			//位置环P参数
        RW,			//位置环I参数
        RW,			//位置环D参数
        RW,			//位置死区
        RW,			//电流前馈
        RW,			//电流前馈
        RW,			//电流前馈
    },
    {//0x6*字段
        NO_ACCESS,			//
        RW,			//电流环P参数
        RW,			//电流环I参数
        RW,			//电流环D参数
        RW,			//速度环P参数
        RW,			//速度环I参数
        RW,			//速度环D参数
        RW,			//速度死区
        RW,			//位置环P参数
        RW,			//位置环I参数
        RW,			//位置环D参数
        RW,			//位置死区
    },
    {//0x7*字段
        NO_ACCESS,			//
        RW,			//电流环P参数
        RW,			//电流环I参数
        RW,			//电流环D参数
        RW,			//速度环P参数
        RW,			//速度环I参数
        RW,			//速度环D参数
        RW,			//速度死区
        RW,			//位置环P参数
        RW,			//位置环I参数
        RW,			//位置环D参数
        RW,			//位置死区
    },
    {//0x8*字段
        RW,			//刹车释放命令
        RO,			//刹车状态
    },
    {//0x9*字段
        RW,			//记录对象标志MASK
        RW,			//触发源，0为开环触发，1为电流触发，2为速度触发，3为位置触发，4为用户触发
        RW,			//触发方式，0为上升沿，1为下降沿，2为连续采样
        RW,			//用户触发标志
        RW,			//记录时间间隔（对10kHZ的分频值）
        RW,			//记录时间偏置（默认以信号过零点时刻±50次数据）
        RO,			//目标电流数据集
        RO,			//实际电流数据集
        RO,			//目标速度数据集
        RO,			//实际速度数据集
        RO,			//目标位置数据集
        RO,			//实际位置数据集
    },
};

#define CMD_IN_PROGRESS -1
#define CMD_ACK_OK      1
#define CMD_ACK_NOK      2
#define CMD_IDLE        0

extern canSend_t hCansendHandler[MAX_CAN_DEVICES];

jCallback_t jointRxCb[CMDMAP_LEN] = { NULL };  // call back of read Joint ID
jCallback_t jointTxCb[CMDMAP_LEN] = { NULL };  // call back of read Joint ID
Joint* jointStack[MAX_JOINTS];    // online joint stack
uint16_t jointNbr = 0;
int32_t rx_flag[CMDMAP_LEN] = {CMD_IDLE};
int32_t tx_flag[CMDMAP_LEN] = {CMD_IDLE};


// callback of read command
int32_t _onCommonReadEntry(void* module, uint16_t index, void* args) {
  Module* d = (Module*)module;
  rx_flag[index] = CMD_ACK_OK;
  if (jointRxCb[index] != NULL) {
	  jointRxCb[index](*d->moduleId, index, (void*)&d->memoryTable[index]);
	  //jointRxCb[index] = NULL;
  }

  return MR_ERROR_OK;
}

int32_t _onCommonWriteEntry(void* module, uint16_t index, void* args) {
  Module* d = (Module*)module;
  if (*(uint8_t*)args == 1)
	  tx_flag[index] = CMD_ACK_OK;
  else
	  tx_flag[index] = CMD_ACK_NOK;

  if (jointTxCb[index] != NULL) {
	  jointTxCb[index](*d->moduleId, index, args);
	  //jointTxCb[index] = NULL;
  }

  return MR_ERROR_OK;
}


float __stdcall pulse2degree(int32_t pos_q31) {
	return (float)pos_q31 / 65536.0f * 360.0f;  // degree
}

int32_t __stdcall degree2pulse(float pos_f) {
	return (int32_t)(pos_f / 360.0f * 65536.0f);
}

int32_t __stdcall jointPush(JOINT_HANDLE h, float pos, float speed, float current _DEF_ARG) {
	Joint* pJoint = (Joint*)h;
	int32_t buf[3] = {0};
	if (!pJoint) {
		return MR_ERROR_ILLDATA;
	}
//    if (pJoint->txQueFront == (pJoint->txQueRear+1)%MAX_SERVO_BUFS) { //full
//        return MR_ERROR_QXMTFULL;
//    }
	buf[0] = (int32_t)(pos / 360.0f*65536.0f);
	buf[1] = (int32_t)(speed / 360.0f*65536.0f*(float)(*pJoint->jointRatio));
	buf[2] = (int32_t)(current *1000.0f);

//	writeEntryNR(pJoint->basicModule, SYS_POSITION_L, &buf[0], 4);
	// only first 8 bytes will be send to joint
	writeSyncMsg(pJoint->basicModule, 0x200, (void*)buf);

//	memcpy((void*)pJoint->txQue[pJoint->txQueRear], (void*)buf, 12);
//    pJoint->txQueRear = (pJoint->txQueRear+1)%MAX_SERVO_BUFS;
    return MR_ERROR_OK;
}

/// 从内存表获取实际位置和实际电流
int32_t __stdcall jointPoll(JOINT_HANDLE h, float* pos, float* speed, float* current) {
	Joint* pJoint = (Joint*)h;
	int32_t temp;
	if (!pJoint) {
		return MR_ERROR_ILLDATA;
	}
	if (pos) {
		memcpy(&temp, &(pJoint->basicModule->memoryTable[SYS_POSITION_L]), 4);
		*pos = (float)temp / 65536.0f * 360.0f; // degree
	}
	if (speed) {
		memcpy(&temp, &(pJoint->basicModule->memoryTable[SYS_SPEED_L]), 4);
		*speed = (float)temp / 65536.0f * 360.0f; // degree/s
	}
	if (current) {
		memcpy(&temp, &(pJoint->basicModule->memoryTable[SYS_CURRENT_L]), 4);
		*current = (float)temp / 1000.0f;  // A
	}

	return MR_ERROR_OK;
}

/// 从内存表获取实际位置和实际电流 (通过scope)
int32_t __stdcall jointPollScope(JOINT_HANDLE h, float* pos, float* speed, float* current) {
	Joint* pJoint = (Joint*)h;
	int32_t temp;
	if (!pJoint)
		return MR_ERROR_ILLDATA;
	if (pos) {
		memcpy(&temp, &(pJoint->basicModule->memoryTable[SCP_MEAPOS_L]), 4);
		*pos = (float)temp / 65536.0f * 360.0f; // degree
	}
	if (speed) {
		memcpy(&temp, &(pJoint->basicModule->memoryTable[SCP_MEASPD_L]), 4);
		*speed = (float)temp / 65536.0f * 360.0f; // degree/s
	}
	if (current) {
		memcpy(&temp, &(pJoint->basicModule->memoryTable[SCP_MEACUR_L]), 4);
		*current = (float)temp / 1000.0f;  // A
	}

	return MR_ERROR_OK;
}

int32_t _jointSendPVTSeq(Joint* h) {
  uint8_t buf[12];
  Joint* pJoint = (Joint*)h;
  uint16_t len = (pJoint->txQueRear + MAX_SERVO_BUFS - pJoint->txQueFront) % MAX_SERVO_BUFS;
  if (len < WARNING_SERVO_BUFS) {
	  if (pJoint->jointBufUnderflowHandler)
		  pJoint->jointBufUnderflowHandler(pJoint, len);
	  else return -2; //Sevo stopped
  }
  if (len == 0) {//empty
	  writeSyncMsg(pJoint->basicModule, 0x200, NULL);
  }
  memcpy((void*)buf, (void*)pJoint->txQue[pJoint->txQueFront], 12);
  pJoint->txQueFront = (pJoint->txQueFront + 1) % MAX_SERVO_BUFS;
  // only first 8 bytes will be send to joint
  writeSyncMsg(pJoint->basicModule, 0x200, (void*)buf);

  return MR_ERROR_OK;
}

// called by master in timerThread
int32_t jointPeriodSend(void* tv) {
  for (int16_t i = 0; i < jointNbr; i++) {
    _jointSendPVTSeq(jointStack[i]);
  }
  return MR_ERROR_OK;
}

Joint* jointConstruct(uint16_t id, canSend_t canSend) {
  uint16_t indexMap[4] = {SYS_POSITION_L, SYS_POSITION_H, SYS_CURRENT_L, SYS_CURRENT_H};
  Joint* pJoint = (Joint*)malloc(sizeof(Joint));
  Module* pModule;
  pJoint->basicModule = (Module*)malloc(sizeof(Module));
  pModule = pJoint->basicModule;
  pModule->memoryLen = CMDMAP_LEN;
  pModule->memoryTable = (uint16_t*)calloc(CMDMAP_LEN, sizeof(uint16_t));
  memset(pModule->memoryTable, 0, CMDMAP_LEN* sizeof(uint16_t));
  pModule->readDoneCb = (mCallback_t*)calloc(CMDMAP_LEN, sizeof(mCallback_t));
  pModule->writeDoneCb = (mCallback_t*)calloc(CMDMAP_LEN, sizeof(mCallback_t));
  pModule->accessType = (uint8_t*)joint_accessType;

  pModule->memoryTable[SYS_ID] = id;
  pJoint->jointId = (uint16_t*)&(pModule->memoryTable[SYS_ID]);
  pJoint->jointRatio = (uint16_t*)&(pModule->memoryTable[SYS_REDU_RATIO]);
  pModule->moduleId = pJoint->jointId;
  pModule->canSend = canSend;

  pJoint->jointType = (uint16_t*)&(pModule->memoryTable[SYS_MODEL_TYPE]);

  pJoint->isOnline = JOINT_OFFLINE;
  pJoint->txQueFront = 0;
  pJoint->txQueRear = 0;
  memset((void*)(pJoint->txQue), 0, sizeof(rec_t)*MAX_SERVO_BUFS);
  pJoint->jointBufUnderflowHandler = NULL;

  setSyncReceiveMap(pJoint->basicModule, indexMap);

  return pJoint;
}

int32_t jointDestruct(Joint* pJoint) {
  Module* pModule = (Module*)pJoint->basicModule;
  if (pJoint) {
    if (pModule->memoryTable)
      free(pModule->memoryTable);
    if (pModule->readDoneCb)
      free(pModule->readDoneCb);
    if (pModule->writeDoneCb)
      free(pModule->writeDoneCb);
    free(pModule);
    free(pJoint);
    return MR_ERROR_OK;
  }
  return MR_ERROR_ILLDATA;
}

void __stdcall jointStartServo(JOINT_HANDLE h, jQueShortHandler_t handler) {
	Joint* pJoint = (Joint*)h;
    if (pJoint)
        pJoint->jointBufUnderflowHandler = handler;
}

void __stdcall jointStopServo(JOINT_HANDLE h) {
	Joint* pJoint = (Joint*)h;
	pJoint->jointBufUnderflowHandler = NULL;
}

JOINT_HANDLE __stdcall jointSelect(uint16_t id) {
  uint16_t i;
  for (i = 0; i < jointNbr; i++) {
    if (id == *(jointStack[i]->jointId))
      return (JOINT_HANDLE)jointStack[i];
  }

  return NULL;
}

JOINT_HANDLE __stdcall jointUp(uint16_t joindId, uint8_t masterId) {
	int32_t res, i = 0;
	Joint* pJoint = jointConstruct(joindId, (canSend_t)hCansendHandler[masterId]);

	if (jointNbr >= MAX_JOINTS) {
		ELOG("Joint Stack Overflow");
		return NULL;
	} else {
		if (pJoint != jointSelect(*(pJoint->jointId)))
			jointStack[jointNbr++] = pJoint; // push into stack
		else return (JOINT_HANDLE)pJoint; // already in the stack
	}
	res = jointGetType(pJoint, NULL, 5000, NULL);
	if ((res == 0) && isJointType(*(pJoint->jointType))) {
		while ((*(pJoint->jointRatio) == 0) && i < 10) {
			i++;
			res = jointGetRatio(pJoint, NULL, 1000, NULL);
		}
		if (i < 10)
			return (JOINT_HANDLE)pJoint;
	}
	jointStack[--jointNbr] = NULL; // delete from stack
	jointDestruct(pJoint);
	return NULL;
}

int32_t __stdcall jointDown(JOINT_HANDLE h) {
  uint16_t i;
  Joint* pJoint = h;
  if (!jointNbr) {
    ELOG("Joint Stack Underflow");
    return MR_ERROR_ILLDATA;
  }
  if(!pJoint) {
    ELOG("Joint is NULL");
    return MR_ERROR_ILLDATA;
  }

  for (i = 0; i < jointNbr; i++) {
    if (pJoint == jointStack[i])
      break;
  }
  for (; i < jointNbr - 1; i++) {
    jointStack[i] = jointStack[i+1];
  }
  jointStack[--jointNbr] = NULL;
  jointDestruct(pJoint);
  return MR_ERROR_OK;
}

/// Get Information from Joints

/// waiting for n us, if return MR_ERROR_OK, id will be stored in pJoint
int32_t __stdcall jointGet(uint8_t index, uint8_t datLen, Joint* pJoint, void* data, int32_t timeout, jCallback_t callBack) { //us
	int16_t i;
	Module* pModule;
	if (pJoint == NULL) {
		return MR_ERROR_ILLDATA;
	}
	pModule = pJoint->basicModule;
    if (timeout == -1) {
		jointRxCb[index] = callBack;
		readEntryCallback(pModule, index, datLen, _onCommonReadEntry);
        return MR_ERROR_OK;
	}
	//timeout is not INFINITE
	if (callBack != NULL) WLOG("callback will not work");
	if (rx_flag[index] == CMD_IN_PROGRESS) {
		//reading in process
        return MR_ERROR_BUSY;
	}
	rx_flag[index] = CMD_IN_PROGRESS;
	readEntryCallback(pModule, index, datLen, _onCommonReadEntry);
	for (i = 0; i < timeout; i++) {
		if (rx_flag[index] == CMD_ACK_OK) {
			if (data)
				memcpy(data, (void*)&(pModule->memoryTable[index]), datLen);
			rx_flag[index] = CMD_IDLE;
            return MR_ERROR_OK;
		}
		delay_us(1);
	}
	rx_flag[index] = CMD_IDLE;
    return MR_ERROR_TIMEOUT;
}
/// 如果timeout为infinite且callBack为空，则调用无返回的写函数writeEntryNR
int32_t __stdcall jointSet(uint8_t index, uint8_t datLen, Joint* pJoint, void* data, int32_t timeout, jCallback_t callBack) { //us
	int16_t i;
	int32_t ret;
	Module* pModule;
	if (pJoint == NULL) {
		return MR_ERROR_ILLDATA;
	}
	pModule = pJoint->basicModule;
	if (timeout == -1) { //INFINITE
		if (callBack == NULL) {
			writeEntryNR(pModule, index, data, datLen);
		} else {
			jointTxCb[index] = callBack;
			writeEntryCallback(pModule, index, data, datLen, _onCommonWriteEntry);
		}
        return MR_ERROR_OK;
	}
	//timeout is not INFINITE
	if (callBack != NULL) ILOG("callback will not work");
	if (tx_flag[index] == CMD_IN_PROGRESS) {
		//reading in process
        return MR_ERROR_BUSY;
	}
	tx_flag[index] = CMD_IN_PROGRESS;
	writeEntryCallback(pModule, index, data, datLen, _onCommonWriteEntry);
	for (i = 0; i < timeout; i++) {
		if (tx_flag[index] != CMD_IN_PROGRESS) {
            if (tx_flag[index] == CMD_ACK_OK) ret = MR_ERROR_ACK1;
            else if (tx_flag[index] == CMD_ACK_NOK) ret = MR_ERROR_ACK0;
			tx_flag[index] = CMD_IDLE;
			return ret;
		}
		delay_us(1);
	}
	tx_flag[index] = CMD_IDLE;
    return MR_ERROR_TIMEOUT;
}

/// waiting for n us, if return 0, id will be stored in pJoint
int32_t __stdcall jointGetId(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) { //us
	return jointGet(SYS_ID, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetType(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) { //us
	return jointGet(SYS_MODEL_TYPE, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetError(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_ERROR, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetVoltage(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_VOLTAGE, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetTemp(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_TEMP, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetRatio(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_REDU_RATIO, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetBaudrate(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_BAUDRATE_CAN, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetCurrent(JOINT_HANDLE pJoint, uint32_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_CURRENT_L, 4, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetSpeed(JOINT_HANDLE pJoint, uint32_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_SPEED_L, 4, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetPosition(JOINT_HANDLE pJoint, uint32_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(SYS_POSITION_L, 4, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetMode(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(TAG_WORK_MODE, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetMaxSpeed(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(LIT_MAX_SPEED, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetMaxAcceleration(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(LIT_MAX_ACC, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGePositionLimit(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return jointGet(LIT_MIN_POSITION_L, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t __stdcall jointGetCurrP(JOINT_HANDLE pJoint, uint16_t* pValue, int32_t timeout, jCallback_t callBack) {
	return jointGet(S_CURRENT_P, 2, (Joint*)pJoint, pValue, timeout, callBack);
}

int32_t __stdcall jointGetCurrI(JOINT_HANDLE pJoint, uint16_t* iValue, int32_t timeout, jCallback_t callBack) {
	return jointGet(S_CURRENT_I, 2, (Joint*)pJoint, iValue, timeout, callBack);
}

int32_t __stdcall jointGetSpeedP(JOINT_HANDLE pJoint, uint16_t* pValue, int32_t timeout, jCallback_t callBack) {
	return jointGet(S_SPEED_P, 2, (Joint*)pJoint, pValue, timeout, callBack);
}

int32_t __stdcall jointGetSpeedI(JOINT_HANDLE pJoint, uint16_t* iValue, int32_t timeout, jCallback_t callBack) {
	return jointGet(S_SPEED_I, 2, (Joint*)pJoint, iValue, timeout, callBack);
}

int32_t __stdcall jointGetPositionP(JOINT_HANDLE pJoint, uint16_t* pValue, int32_t timeout, jCallback_t callBack) {
	return jointGet(S_POSITION_P, 2, (Joint*)pJoint, pValue, timeout, callBack);
}

int32_t __stdcall jointGetPositionDs(JOINT_HANDLE pJoint, uint16_t* dsValue, int32_t timeout, jCallback_t callBack) {
	return jointGet(S_POSITION_DS, 2, (Joint*)pJoint, dsValue, timeout, callBack);
}

int32_t __stdcall jointSetID(JOINT_HANDLE pJoint, uint16_t id, int32_t timeout, jCallback_t callBack) {
	return jointSet(SYS_ID, 2, (Joint*)pJoint, (void*)&id, timeout, callBack);
}

int32_t __stdcall jointSetBaudrate(JOINT_HANDLE pJoint, uint16_t baud, int32_t timeout, jCallback_t callBack) { //us
	if (isJointMode(baud)) {
		return jointSet(SYS_BAUDRATE_CAN, 2, (Joint*)pJoint, (void*)&baud, timeout, callBack);
	}
	return MR_ERROR_ILLDATA;
}

int32_t __stdcall jointSetEnable(JOINT_HANDLE pJoint, uint16_t isEnable, int32_t timeout, jCallback_t callBack) {
	return jointSet(SYS_ENABLE_DRIVER, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t __stdcall jointSetPowerOnStatus(JOINT_HANDLE pJoint, uint16_t isEnable, int32_t timeout, jCallback_t callBack) {
	return jointSet(SYS_ENABLE_ON_POWER, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t __stdcall jointSetSave2Flash(JOINT_HANDLE pJoint, int32_t timeout, jCallback_t callBack) {
	uint16_t isEnable = 1;
	return jointSet(SYS_SAVE_TO_FLASH, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t __stdcall jointSetZero(JOINT_HANDLE pJoint, int32_t timeout, jCallback_t callBack) {
	uint16_t isEnable = 1;
	return jointSet(SYS_SET_ZERO_POS, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t __stdcall jointSetClearError(JOINT_HANDLE pJoint, int32_t timeout, jCallback_t callBack) {
	uint16_t isEnable = 1;
	return jointSet(SYS_CLEAR_ERROR, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t __stdcall jointSetMode(JOINT_HANDLE pJoint, uint16_t mode, int32_t timeout, jCallback_t callBack) { //us
	if (isJointMode(mode)) {
		return jointSet(TAG_WORK_MODE, 2, (Joint*)pJoint, (void*)&mode, timeout, callBack);
	}
	return MR_ERROR_ILLDATA;
}

int32_t __stdcall jointSetSpeed(JOINT_HANDLE pJoint, int32_t speed, int32_t timeout, jCallback_t callBack) {
	return jointSet(TAG_SPEED_L, 4, (Joint*)pJoint, (void*)&speed, timeout, callBack);
}

int32_t __stdcall jointSetPosition(JOINT_HANDLE pJoint, int32_t position, int32_t timeout, jCallback_t callBack) {
	return jointSet(TAG_POSITION_L, 4, (Joint*)pJoint, (void*)&position, timeout, callBack);
}

int32_t __stdcall jointSetMaxSpeed(JOINT_HANDLE pJoint, int32_t maxspeed, int32_t timeout, jCallback_t callBack) {
	return jointSet(LIT_MAX_SPEED, 2, (Joint*)pJoint, (void*)&maxspeed, timeout, callBack);
}

int32_t __stdcall jointSetMaxAcceleration(JOINT_HANDLE pJoint, int32_t maxacc, int32_t timeout, jCallback_t callBack) {
	return jointSet(LIT_MAX_ACC, 2, (Joint*)pJoint, (void*)&maxacc, timeout, callBack);
}

int32_t __stdcall jointSetPositionLimit(JOINT_HANDLE pJoint, int32_t position_min, int32_t position_max, int32_t timeout, jCallback_t callBack) {

	return jointSet(LIT_MIN_POSITION_L, 8, (Joint*)pJoint, (void*)&position_min, timeout, callBack);
}

int32_t __stdcall jointSetCurrP(JOINT_HANDLE pJoint, uint16_t pValue, int32_t timeout, jCallback_t callBack) {
	return jointSet(S_CURRENT_P, 2, (Joint*)pJoint, (void*)&pValue, timeout, callBack);
}

int32_t __stdcall jointSetCurrI(JOINT_HANDLE pJoint, uint16_t iValue, int32_t timeout, jCallback_t callBack) {
	return jointSet(S_CURRENT_I, 2, (Joint*)pJoint, (void*)&iValue, timeout, callBack);
}

int32_t __stdcall jointSetSpeedP(JOINT_HANDLE pJoint, uint16_t pValue, int32_t timeout, jCallback_t callBack) {
	return jointSet(S_SPEED_P, 2, (Joint*)pJoint, (void*)&pValue, timeout, callBack);
}

int32_t __stdcall jointSetSpeedI(JOINT_HANDLE pJoint, uint16_t iValue, int32_t timeout, jCallback_t callBack) {
	return jointSet(S_SPEED_I, 2, (Joint*)pJoint, (void*)&iValue, timeout, callBack);
}

int32_t __stdcall jointSetPositionP(JOINT_HANDLE pJoint, uint16_t pValue, int32_t timeout, jCallback_t callBack) {
	return jointSet(S_POSITION_P, 2, (Joint*)pJoint, (void*)&pValue, timeout, callBack);
}

int32_t __stdcall jointSetPositionD(JOINT_HANDLE pJoint, uint16_t dValue, int32_t timeout, jCallback_t callBack) {
	return jointSet(S_POSITION_D, 2, (Joint*)pJoint, (void*)&dValue, timeout, callBack);
}

int32_t __stdcall jointSetPositionDs(JOINT_HANDLE pJoint, uint16_t dsValue, int32_t timeout, jCallback_t callBack) {
	return jointSet(S_POSITION_DS, 2, (Joint*)pJoint, (void*)&dsValue, timeout, callBack);
}

int32_t __stdcall jointSetScpMask(JOINT_HANDLE pJoint, uint16_t mask, int32_t timeout, jCallback_t callBack) {
	return jointSet(SCP_MASK, 2, (Joint*)pJoint, (void*)&mask, timeout, callBack);
}

int32_t __stdcall jointSetScpInterval(JOINT_HANDLE pJoint, uint16_t interval, int32_t timeout, jCallback_t callBack) {
	return jointSet(SCP_REC_TIM, 2, (Joint*)pJoint, (void*)&interval, timeout, callBack);
}

int32_t __stdcall jointSetBootloader(JOINT_HANDLE pJoint, uint16_t mask, int32_t timeout, jCallback_t callBack) {
	return jointSet(SYS_IAP, 2, (Joint*)pJoint, (void*)&mask, timeout, callBack);
}




