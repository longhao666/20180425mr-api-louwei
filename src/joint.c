#include <stdlib.h>
#include <string.h>
#include "joint.h"

#define CMDMAP_LEN            160    //

//系统状态相关
#define SYS_ID                0x01    //驱动器ID
#define SYS_MODEL_TYPE        0x02    //驱动器型号
#define SYS_FW_VERSION        0x03    //固件版本
#define SYS_ERROR             0x04    //错误代码
#define SYS_VOLTAGE           0x05    //系统电压
#define SYS_TEMP              0x06    //系统温度
#define SYS_REDU_RATIO        0x07    //模块减速比
//#define SYS_BAUDRATE_232      0x08    //232端口波特率
#define SYS_BAUDRATE_CAN      0x09    //CAN总线波特率
#define SYS_ENABLE_DRIVER     0x0a    //驱动器使能标志
#define SYS_ENABLE_ON_POWER   0x0b    //上电使能驱动器标志
#define SYS_SAVE_TO_FLASH     0x0c    //保存数据到Flash标志
//#define SYS_DEMA_ABSPOS       0x0d    //自动标定绝对位置标志
#define SYS_SET_ZERO_POS      0x0e    //将当前位置设置为零点标志
#define SYS_CLEAR_ERROR       0x0f    //清除错误标志

#define SYS_CURRENT_L         0x10    //当前电流低16位（mA）
#define SYS_CURRENT_H         0x11    //当前电流高16位（mA）
#define SYS_SPEED_L           0x12    //当前速度低16位（units/s）
#define SYS_SPEED_H           0x13    //当前速度高16位（units/s）
#define SYS_POSITION_L        0x14    //当前位置低16位（units）
#define SYS_POSITION_H        0x15    //当前位置高16位（units）
#define SYS_POTEN_VALUE       0x16    //数字电位器值
#define SYS_ZERO_POS_OFFSET_L 0x17    //零点位置偏移量低16位（units）
#define SYS_ZERO_POS_OFFSET_H 0x18    //零点位置偏移量高16位（units）

//电机相关信息
#define MOT_RES               0x20    //电机内阻
#define MOT_INDUC             0x21    //电机电感
#define MOT_RATED_VOL         0x22    //电机额定电压
#define MOT_RATED_CUR         0x23    //电机额定电流
//#define MOT_ENC_LINES         0x24    //码盘线数
//#define MOT_HALL_VALUE        0x25    //当前霍尔状态
#define MOT_ST_DAT            0x26    //绝对编码器单圈数据
#define MOT_MT_DAT            0x27    //绝对编码器多圈数据
#define MOT_ENC_STA           0x28    //绝对编码器状态寄存器
#define BAT_VOLT              0x29    //编码器电池电压 *10mV
#define ACC_X                 0x2A    //加速度计x轴 *1000mg
#define ACC_Y                 0x2B    //加速度计y轴 *1000mg
#define ACC_Z                 0x2C    //加速度计z轴 *1000mg

//控制目标值
#define TAG_WORK_MODE         0x30    //工作模式，0-开环，1-电流模式，2-速度模式，3-位置模式
#define TAG_OPEN_PWM          0x31    //开环模式下占空比（0~100）
#define TAG_CURRENT_L         0x32    //目标电流低16位（mA）
#define TAG_CURRENT_H         0x33    //目标电流高16位（mA）
#define TAG_SPEED_L           0x34    //目标速度低16位（units/s）
#define TAG_SPEED_H           0x35    //目标速度高16位（units/s）
#define TAG_POSITION_L        0x36    //目标位置低16位（units）
#define TAG_POSITION_H        0x37    //目标位置高16位（units）

//控制限制值
#define LIT_MAX_CURRENT       0x40    //最大电流（mA）
#define LIT_MAX_SPEED         0x41    //最大速度（rpm）
#define LIT_MAX_ACC           0x42    //最大加速度（rpm/s）
#define LIT_MIN_POSITION_L    0x43    //最小位置低16位（units）
#define LIT_MIN_POSITION_H    0x44    //最小位置高16位（units）
#define LIT_MAX_POSITION_L    0x45    //最大位置低16位（units）
#define LIT_MAX_POSITION_H    0x46    //最大位置高16位（units）

//三闭环环相关
#define SEV_PARAME_LOCKED     0x50    //三闭环参数锁定标志, 0-不锁定(自动切换), 1-低速（S）,2-中速（M）,3-高速（L）

#define S_CURRENT_P           0x51    //电流环P参数
#define S_CURRENT_I           0x52    //电流环I参数
#define S_CURRENT_D           0x53    //电流环D参数
#define S_SPEED_P             0x54    //速度环P参数
#define S_SPEED_I             0x55    //速度环I参数
#define S_SPEED_D             0x56    //速度环D参数
#define S_SPEED_DS            0x57    //速度P死区
#define S_POSITION_P          0x58    //位置环P参数
#define S_POSITION_I          0x59    //位置环I参数
#define S_POSITION_D          0x5A    //位置环D参数
#define S_POSITION_DS         0x5B    //位置P死区
#define S_CURRENT_FD          0x5C    //电流前馈
#define M_CURRENT_FD          0x5D    //电流前馈
#define L_CURRENT_FD          0x5E    //电流前馈

#define M_CURRENT_P           0x61    //电流环P参数
#define M_CURRENT_I           0x62    //电流环I参数
#define M_CURRENT_D           0x63    //电流环D参数
#define M_SPEED_P             0x64    //速度环P参数
#define M_SPEED_I             0x65    //速度环I参数
#define M_SPEED_D             0x66    //速度环D参数
#define M_SPEED_DS            0x67    //速度P死区
#define M_POSITION_P          0x68    //位置环P参数
#define M_POSITION_I          0x69    //位置环I参数
#define M_POSITION_D          0x6A    //位置环D参数
#define M_POSITION_DS         0x6B    //位置P死区

#define L_CURRENT_P           0x71    //电流环P参数
#define L_CURRENT_I           0x72    //电流环I参数
#define L_CURRENT_D           0x73    //电流环D参数
#define L_SPEED_P             0x74    //速度环P参数
#define L_SPEED_I             0x75    //速度环I参数
#define L_SPEED_D             0x76    //速度环D参数
#define L_SPEED_DS            0x77    //速度P死区
#define L_POSITION_P          0x78    //位置环P参数
#define L_POSITION_I          0x79    //位置环I参数
#define L_POSITION_D          0x7A    //位置环D参数
#define L_POSITION_DS         0x7B    //位置P死区

//刹车控制命令
#define BRAKE_RELEASE_CMD     0x80    //刹车释放命令，0-保持制动，1-释放刹车
#define BRAKE_STATE           0x81    //刹车状态，0-保持制动，1-释放刹车

//示波器模块子索引地址定义
#define SCP_MASK              0x90    //记录对象标志MASK
#define SCP_REC_TIM           0x91    //记录时间间隔（对10kHZ的分频值）

#define SCP_TAGCUR_L          0x92    //目标电流数据集
#define SCP_TAGCUR_H          0x93    //目标电流数据集
#define SCP_MEACUR_L          0x94    //实际电流数据集
#define SCP_MEACUR_H          0x95    //实际电流数据集
#define SCP_TAGSPD_L          0x96    //目标速度数据集
#define SCP_TAGSPD_H          0x97    //目标速度数据集
#define SCP_MEASPD_L          0x98    //实际速度数据集
#define SCP_MEASPD_H          0x99    //实际速度数据集
#define SCP_TAGPOS_L          0x9A    //目标位置数据集
#define SCP_TAGPOS_H          0x9B    //目标位置数据集
#define SCP_MEAPOS_L          0x9C    //实际位置数据集
#define SCP_MEAPOS_H          0x9D    //实际位置数据集

//示波器记录对象MASK定义
#define MASK_TAGCUR         0x0001    //记录目标电流MASK
#define MASK_MEACUR         0x0002    //记录实际电流MASK
#define MASK_TAGSPD         0x0004    //记录目标速度MASK
#define MASK_MEASPD         0x0008    //记录实际速度MASK
#define MASK_TAGPOS         0x0010    //记录目标位置MASK
#define MASK_MEAPOS         0x0020    //记录实际位置MASK

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
        RW,			//自动标定绝对位置标志（本版本已移除）
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

int32_t jointPush(JOINT_HANDLE h, int32_t* pos, int32_t* speed) {
	Joint* pJoint = (Joint*)h;
	int32_t buf[2];
    if (pJoint->txQueFront == (pJoint->txQueRear+1)%MAX_BUFS) { //full
        return MR_ERROR_QXMTFULL;
    }
	buf[0] = *pos;
	buf[1] = *speed;
    memcpy((void*)pJoint->txQue[pJoint->txQueRear], (void*)buf, 8);
    pJoint->txQueRear = (pJoint->txQueRear+1)%MAX_BUFS;
    return MR_ERROR_OK;
}

int32_t jointPoll(JOINT_HANDLE h, int32_t* pos, int32_t* speed) {
	Joint* pJoint = (Joint*)h;
    if (!pJoint)
        return MR_ERROR_ILLDATA;
	if (pos) memcpy(pos, &(pJoint->basicModule->memoryTable[SYS_POSITION_L]), 4);
	if (speed) memcpy(speed, &(pJoint->basicModule->memoryTable[SYS_SPEED_L]), 4);

	return MR_ERROR_OK;
}

int32_t _jointGetPVTSeq(JOINT_HANDLE h, uint8_t* buf) {
	Joint* pJoint = (Joint*)h;
	uint16_t len = (pJoint->txQueRear+MAX_BUFS - pJoint->txQueFront)%MAX_BUFS;
    if (len < WARNING_BUFS) {
        if (pJoint->jointBufUnderflowHandler)
            pJoint->jointBufUnderflowHandler(pJoint, len);
        else return -2; //Sevo stopped
    }
    if (len == 0) {//empty
        return -1;
    }
    memcpy((void*)buf, (void*)pJoint->txQue[pJoint->txQueFront], 8);
    pJoint->txQueFront = (pJoint->txQueFront+1)%MAX_BUFS;
    return 0;
}

int32_t _jointSendPVTSeq(Joint* pJoint) {
  uint8_t buf[8];
  int32_t ret = _jointGetPVTSeq(pJoint, buf);

  if (ret == 0)
      writeSyncMsg(pJoint->basicModule, 0x200, (void*)buf);
  else if (ret == -1) {
	  writeSyncMsg(pJoint->basicModule, 0x200, NULL);
  }
  return MR_ERROR_OK;
}

int32_t jointPeriodSend(void* tv) {
  for (int16_t i = 0; i < jointNbr; i++) {
    _jointSendPVTSeq(jointStack[i]);
  }
  return MR_ERROR_OK;
}

Joint* jointConstruct(uint16_t id, canSend_t canSend) {
  uint16_t indexMap[4] = {SYS_POSITION_L, SYS_POSITION_H, SYS_SPEED_L, SYS_SPEED_H};
  Joint* pJoint = (Joint*)malloc(sizeof(Joint));
  Module* pModule;
  pJoint->basicModule = (Module*)malloc(sizeof(Module));
  pModule = pJoint->basicModule;
  pModule->memoryLen = CMDMAP_LEN;
  pModule->memoryTable = (uint16_t*)calloc(CMDMAP_LEN, sizeof(uint16_t));
  pModule->readDoneCb = (mCallback_t*)calloc(CMDMAP_LEN, sizeof(mCallback_t));
  pModule->writeDoneCb = (mCallback_t*)calloc(CMDMAP_LEN, sizeof(mCallback_t));
  pModule->accessType = (uint8_t*)joint_accessType;

  pModule->memoryTable[SYS_ID] = id;
  pJoint->jointId = (uint16_t*)&(pModule->memoryTable[SYS_ID]);
  pModule->moduleId = pJoint->jointId;
  pModule->canSend = canSend;

  pJoint->jointType = (uint16_t*)&(pModule->memoryTable[SYS_MODEL_TYPE]);

  pJoint->isOnline = JOINT_OFFLINE;
  pJoint->txQueFront = 0;
  pJoint->txQueRear = 0;
  memset((void*)(pJoint->txQue), 0, sizeof(rec_t)*MAX_BUFS);
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

void jointStartServo(JOINT_HANDLE h, jQueShortHandler_t handler) {
	Joint* pJoint = (Joint*)h;
    if (pJoint)
        pJoint->jointBufUnderflowHandler = handler;

}

void jointStopServo(JOINT_HANDLE h) {
	Joint* pJoint = (Joint*)h;
	pJoint->jointBufUnderflowHandler = NULL;
}

JOINT_HANDLE jointSelect(uint16_t id) {
  uint16_t i;
  for (i = 0; i < jointNbr; i++) {
    if (id == *(jointStack[i]->jointId))
      return (JOINT_HANDLE)jointStack[i];
  }

  return NULL;
}

JOINT_HANDLE jointUp(uint16_t id, void* canSend) {
	int32_t res;
	Joint* pJoint = jointConstruct(id, (canSend_t)canSend);

	if (jointNbr >= MAX_JOINTS) {
		ELOG("Joint Stack Overflow");
		return NULL;
	}
	else {
		if (pJoint != jointSelect(*(pJoint->jointId)))
			jointStack[jointNbr++] = pJoint; // push into stack
		else return (JOINT_HANDLE)pJoint; // already in the stack
	}
	res = jointGetType(pJoint, NULL, 5000, NULL);
	if ((res == 0) && isJointType(*(pJoint->jointType))) {
		return (JOINT_HANDLE)pJoint;
	}
	jointStack[jointNbr--] = NULL; // delete from stack
	jointDestruct(pJoint);
	return NULL;
}

int32_t jointDown(JOINT_HANDLE h) {
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
  jointStack[jointNbr--] = NULL;
  jointDestruct(pJoint);
  return MR_ERROR_OK;
}

/// Get Information from Joints

/// waiting for n us, if return MR_ERROR_OK, id will be stored in pJoint
int32_t _jointGETTemplet(uint8_t index, uint8_t datLen, Joint* pJoint, void* data, int32_t timeout, jCallback_t callBack) { //us
	int16_t i;
	Module* pModule = (Module*)pJoint->basicModule;
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

int32_t _jointSETTemplet(uint8_t index, uint8_t datLen, Joint* pJoint, void* data, int32_t timeout, jCallback_t callBack) { //us
	int16_t i;
	int32_t ret;
	Module* pModule = (Module*)pJoint->basicModule;
    if (timeout == -1) { //INFINITE
		jointTxCb[index] = callBack;
		writeEntryCallback(pModule, index, data, datLen, _onCommonWriteEntry);
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
int32_t jointGetId(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) { //us
	return _jointGETTemplet(SYS_ID, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetType(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) { //us
	return _jointGETTemplet(SYS_MODEL_TYPE, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetVoltage(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(SYS_VOLTAGE, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetTemp(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(SYS_TEMP, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetBaudrate(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(SYS_BAUDRATE_CAN, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetCurrent(JOINT_HANDLE pJoint, uint32_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(SYS_CURRENT_L, 4, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetSpeed(JOINT_HANDLE pJoint, uint32_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(SYS_SPEED_L, 4, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetPosition(JOINT_HANDLE pJoint, uint32_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(SYS_POSITION_L, 4, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetMode(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(TAG_WORK_MODE, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetMaxSpeed(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(LIT_MAX_SPEED, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetMaxAcceleration(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(LIT_MAX_ACC, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGePositionLimit(JOINT_HANDLE pJoint, uint16_t* data, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(LIT_MIN_POSITION_L, 2, (Joint*)pJoint, data, timeout, callBack);
}

int32_t jointGetCurrP(JOINT_HANDLE pJoint, uint16_t* pValue, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(S_CURRENT_P, 2, (Joint*)pJoint, pValue, timeout, callBack);
}

int32_t jointGetCurrI(JOINT_HANDLE pJoint, uint16_t* iValue, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(S_CURRENT_I, 2, (Joint*)pJoint, iValue, timeout, callBack);
}

int32_t jointGetSpeedP(JOINT_HANDLE pJoint, uint16_t* pValue, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(S_SPEED_P, 2, (Joint*)pJoint, pValue, timeout, callBack);
}

int32_t jointGetSpeedI(JOINT_HANDLE pJoint, uint16_t* iValue, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(S_SPEED_I, 2, (Joint*)pJoint, iValue, timeout, callBack);
}

int32_t jointGetPositionP(JOINT_HANDLE pJoint, uint16_t* pValue, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(S_POSITION_P, 2, (Joint*)pJoint, pValue, timeout, callBack);
}

int32_t jointGetPositionDs(JOINT_HANDLE pJoint, uint16_t* dsValue, int32_t timeout, jCallback_t callBack) {
	return _jointGETTemplet(S_POSITION_DS, 2, (Joint*)pJoint, dsValue, timeout, callBack);
}

int32_t jointSetID(JOINT_HANDLE pJoint, uint16_t id, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(SYS_ID, 2, (Joint*)pJoint, (void*)&id, timeout, callBack);
}

int32_t jointSetBaudrate(JOINT_HANDLE pJoint, uint16_t baud, int32_t timeout, jCallback_t callBack) { //us
	if (isJointMode(baud)) {
		return _jointSETTemplet(SYS_BAUDRATE_CAN, 2, (Joint*)pJoint, (void*)&baud, timeout, callBack);
	}
	return MR_ERROR_ILLDATA;
}

int32_t jointSetEnable(JOINT_HANDLE pJoint, uint16_t isEnable, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(SYS_ENABLE_DRIVER, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t jointSetPowerOnStatus(JOINT_HANDLE pJoint, uint16_t isEnable, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(SYS_ENABLE_ON_POWER, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t jointSetSave2Flash(JOINT_HANDLE pJoint, int32_t timeout, jCallback_t callBack) {
	uint16_t isEnable = 1;
	return _jointSETTemplet(SYS_SAVE_TO_FLASH, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t jointSetZero(JOINT_HANDLE pJoint, int32_t timeout, jCallback_t callBack) {
	uint16_t isEnable = 1;
	return _jointSETTemplet(SYS_SET_ZERO_POS, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t jointSetClearError(JOINT_HANDLE pJoint, int32_t timeout, jCallback_t callBack) {
	uint16_t isEnable = 1;
	return _jointSETTemplet(SYS_CLEAR_ERROR, 2, (Joint*)pJoint, (void*)&isEnable, timeout, callBack);
}

int32_t jointSetMode(JOINT_HANDLE pJoint, uint16_t mode, int32_t timeout, jCallback_t callBack) { //us
	if (isJointMode(mode)) {
		return _jointSETTemplet(TAG_WORK_MODE, 2, (Joint*)pJoint, (void*)&mode, timeout, callBack);
	}
	return MR_ERROR_ILLDATA;
}

int32_t jointSetSpeed(JOINT_HANDLE pJoint, int32_t speed, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(TAG_SPEED_L, 4, (Joint*)pJoint, (void*)&speed, timeout, callBack);
}

int32_t jointSetPosition(JOINT_HANDLE pJoint, int32_t position, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(TAG_POSITION_L, 4, (Joint*)pJoint, (void*)&position, timeout, callBack);
}

int32_t jointSetMaxSpeed(JOINT_HANDLE pJoint, int32_t maxspeed, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(LIT_MAX_SPEED, 2, (Joint*)pJoint, (void*)&maxspeed, timeout, callBack);
}

int32_t jointSetMaxAcceleration(JOINT_HANDLE pJoint, int32_t maxacc, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(LIT_MAX_ACC, 2, (Joint*)pJoint, (void*)&maxacc, timeout, callBack);
}

int32_t jointSetPositionLimit(JOINT_HANDLE pJoint, int32_t position_min, int32_t position_max, int32_t timeout, jCallback_t callBack) {

	return _jointSETTemplet(LIT_MIN_POSITION_L, 8, (Joint*)pJoint, (void*)&position_min, timeout, callBack);
}

int32_t jointSetCurrP(JOINT_HANDLE pJoint, uint16_t pValue, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(S_CURRENT_P, 2, (Joint*)pJoint, (void*)&pValue, timeout, callBack);
}

int32_t jointSetCurrI(JOINT_HANDLE pJoint, uint16_t iValue, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(S_CURRENT_I, 2, (Joint*)pJoint, (void*)&iValue, timeout, callBack);
}

int32_t jointSetSpeedP(JOINT_HANDLE pJoint, uint16_t pValue, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(S_SPEED_P, 2, (Joint*)pJoint, (void*)&pValue, timeout, callBack);
}

int32_t jointSetSpeedI(JOINT_HANDLE pJoint, uint16_t iValue, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(S_SPEED_I, 2, (Joint*)pJoint, (void*)&iValue, timeout, callBack);
}

int32_t jointSetPositionP(JOINT_HANDLE pJoint, uint16_t pValue, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(S_POSITION_P, 2, (Joint*)pJoint, (void*)&pValue, timeout, callBack);
}

int32_t jointSetPositionDs(JOINT_HANDLE pJoint, uint16_t dsValue, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(S_POSITION_DS, 2, (Joint*)pJoint, (void*)&dsValue, timeout, callBack);
}

int32_t jointSetScpMask(JOINT_HANDLE pJoint, uint16_t mask, int32_t timeout, jCallback_t callBack) {
	return _jointSETTemplet(SCP_MASK, 2, (Joint*)pJoint, (void*)&mask, timeout, callBack);
}




