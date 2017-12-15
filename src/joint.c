#include <stdlib.h>
#include "joint.h"

#define CMDMAP_LEN            160    //ÄÚ´æ¿ØÖÆ±í×Ü³¤¶È£¨°ë×Öµ¥Î»16bits£©

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

//波特率宏定义
#define BAUD_CAN_250000     0x0000    //250K
#define BAUD_CAN_500000     0x0001    //500K
#define BAUD_CAN_1000000    0x0002    //1M

//示波器记录对象MASK定义
#define MASK_TAGCUR         0x0001    //记录目标电流MASK
#define MASK_MEACUR         0x0002    //记录实际电流MASK
#define MASK_TAGSPD         0x0004    //记录目标速度MASK
#define MASK_MEASPD         0x0008    //记录实际速度MASK
#define MASK_TAGPOS         0x0010    //记录目标位置MASK
#define MASK_MEAPOS         0x0020    //记录实际位置MASK

const uint8_t joint_accessType[10][16] = 
{
  {//0x0*×Ö¶Î
    RO,     //×ÖÍ·
    RW,     //Çý¶¯Æ÷ID
    RO,     //Çý¶¯Æ÷ÐÍºÅ
    RO,     //¹Ì¼þ°æ±¾
    RO,     //´íÎó´úÂë
    RO,     //ÏµÍ³µçÑ¹
    RO,     //ÏµÍ³ÎÂ¶È
    RO,     //Ä£¿é¼õËÙ±È
    RW,     //232¶Ë¿Ú²¨ÌØÂÊ£¨±¾°æ±¾ÒÑÒÆ³ý£©
    RW,     //CAN×ÜÏß²¨ÌØÂÊ
    RW,     //Ê¹ÄÜÇý¶¯Æ÷±êÖ¾
    RW,     //ÉÏµçÊ¹ÄÜÇý¶¯Æ÷±êÖ¾
    RW,     //±£´æÊý¾Ýµ½Flash±êÖ¾
    RW,     //×Ô¶¯±ê¶¨¾ø¶ÔÎ»ÖÃ±êÖ¾£¨±¾°æ±¾ÒÑÒÆ³ý£©
    RW,     //½«µ±Ç°Î»ÖÃÉèÖÃÎªÁãµã±êÖ¾
    RW,     //Çå³ý´íÎó±êÖ¾
  },
  {//0x1*×Ö¶Î
    RO,     //µ±Ç°µçÁ÷µÍ16Î»£¨mA£©
    RO,     //µ±Ç°µçÁ÷¸ß16Î»£¨mA£©
    RO,     //µ±Ç°ËÙ¶ÈµÍ16Î»£¨units/s£©
    RO,     //µ±Ç°ËÙ¶È¸ß16Î»£¨units/s£©
    RO,     //µ±Ç°Î»ÖÃµÍ16Î»£¨units£©
    RO,     //µ±Ç°Î»ÖÃ¸ß16Î»£¨units£©
    NO_ACCESS,      //Êý×ÖµçÎ»Æ÷Öµ£¨±¾°æ±¾ÒÑÒÆ³ý£©
    RO,     //ÁãµãÎ»ÖÃÆ«ÒÆÁ¿µÍ16Î»£¨units£©
    RO,     //ÁãµãÎ»ÖÃÆ«ÒÆÁ¿¸ß16Î»£¨units£©
  },
  {//0x2*×Ö¶Î
    RO,     //µç»úÄÚ×è
    RO,     //µç»úµç¸Ð
    RO,     //µç»ú¶î¶¨µçÑ¹
    RO,     //µç»ú¶î¶¨µçÁ÷
    NO_ACCESS,      //ÂëÅÌÏßÊý£¨±¾°æ±¾ÒÑÒÆ³ý£©
    NO_ACCESS,      //µ±Ç°»ô¶û×´Ì¬£¨±¾°æ±¾ÒÑÒÆ³ý£©
    RO,      //¾ø¶Ô±àÂëÆ÷µ¥È¦Êý¾Ý
    RO,      //¾ø¶Ô±àÂëÆ÷¶àÈ¦Êý¾Ý
    RO,      //¶àÈ¦×´Ì¬ÐÅÏ¢
    RO,      //µç³ØµçÑ¹
    RO,      //X
    RO,      //Y
    RO,      //Z
  },
  {//0x3*×Ö¶Î
    RW,     //¹¤×÷Ä£Ê½£¬0-¿ª»·£¬1-µçÁ÷Ä£Ê½£¬2-ËÙ¶ÈÄ£Ê½£¬3-Î»ÖÃÄ£Ê½
    RW,     //¿ª»·Ä£Ê½ÏÂÕ¼¿Õ±È£¨0~100£©
    RW,     //Ä¿±êµçÁ÷µÍ16Î»£¨mA£©
    RW,     //Ä¿±êµçÁ÷¸ß16Î»£¨mA£©
    RW,     //Ä¿±êËÙ¶ÈµÍ16Î»£¨units/s£©
    RW,     //Ä¿±êËÙ¶È¸ß16Î»£¨units/s£©
    RW,     //Ä¿±êÎ»ÖÃµÍ16Î»£¨units£©
    RW,     //Ä¿±êÎ»ÖÃ¸ß16Î»£¨units£©
  },
  {//0x4*×Ö¶Î
    RW,     //×î´óµçÁ÷£¨mA£©
    RW,     //×î´óËÙ¶È£¨rpm£©
    RW,     //×î´ó¼ÓËÙ¶È£¨rpm/s£©
    RW,     //×îÐ¡Î»ÖÃµÍ16Î»£¨units£©
    RW,     //×îÐ¡Î»ÖÃ¸ß16Î»£¨units£©
    RW,     //×î´óÎ»ÖÃµÍ16Î»£¨units£©
    RW,     //×î´óÎ»ÖÃ¸ß16Î»£¨units£©
  },
  {//0x5*×Ö¶Î
    RW,     //Èý±Õ»·²ÎÊýËø¶¨±êÖ¾
    RW,     //µçÁ÷»·P²ÎÊý
    RW,     //µçÁ÷»·I²ÎÊý
    RW,     //µçÁ÷»·D²ÎÊý
    RW,     //ËÙ¶È»·P²ÎÊý
    RW,     //ËÙ¶È»·I²ÎÊý
    RW,     //ËÙ¶È»·D²ÎÊý
    RW,     //ËÙ¶ÈËÀÇø
    RW,     //Î»ÖÃ»·P²ÎÊý
    RW,     //Î»ÖÃ»·I²ÎÊý
    RW,     //Î»ÖÃ»·D²ÎÊý
    RW,     //Î»ÖÃËÀÇø
    RW,     //µçÁ÷Ç°À¡
    RW,     //µçÁ÷Ç°À¡
    RW,     //µçÁ÷Ç°À¡
  },
  {//0x6*×Ö¶Î
    NO_ACCESS,      //
    RW,     //µçÁ÷»·P²ÎÊý
    RW,     //µçÁ÷»·I²ÎÊý
    RW,     //µçÁ÷»·D²ÎÊý
    RW,     //ËÙ¶È»·P²ÎÊý
    RW,     //ËÙ¶È»·I²ÎÊý
    RW,     //ËÙ¶È»·D²ÎÊý
    RW,     //ËÙ¶ÈËÀÇø
    RW,     //Î»ÖÃ»·P²ÎÊý
    RW,     //Î»ÖÃ»·I²ÎÊý
    RW,     //Î»ÖÃ»·D²ÎÊý
    RW,     //Î»ÖÃËÀÇø
  },
  {//0x7*×Ö¶Î
    NO_ACCESS,      //
    RW,     //µçÁ÷»·P²ÎÊý
    RW,     //µçÁ÷»·I²ÎÊý
    RW,     //µçÁ÷»·D²ÎÊý
    RW,     //ËÙ¶È»·P²ÎÊý
    RW,     //ËÙ¶È»·I²ÎÊý
    RW,     //ËÙ¶È»·D²ÎÊý
    RW,     //ËÙ¶ÈËÀÇø
    RW,     //Î»ÖÃ»·P²ÎÊý
    RW,     //Î»ÖÃ»·I²ÎÊý
    RW,     //Î»ÖÃ»·D²ÎÊý
    RW,     //Î»ÖÃËÀÇø
  },
  {//0x8*×Ö¶Î
    RW,     //É²³µÊÍ·ÅÃüÁî
    RO,     //É²³µ×´Ì¬
  },
  {//0x9*×Ö¶Î
    RW,     //¼ÇÂ¼¶ÔÏó±êÖ¾MASK
    RW,     //´¥·¢Ô´£¬0Îª¿ª»·´¥·¢£¬1ÎªµçÁ÷´¥·¢£¬2ÎªËÙ¶È´¥·¢£¬3ÎªÎ»ÖÃ´¥·¢£¬4ÎªÓÃ»§´¥·¢
    RW,     //´¥·¢·½Ê½£¬0ÎªÉÏÉýÑØ£¬1ÎªÏÂ½µÑØ£¬2ÎªÁ¬Ðø²ÉÑù
    RW,     //ÓÃ»§´¥·¢±êÖ¾
    RW,     //¼ÇÂ¼Ê±¼ä¼ä¸ô£¨¶Ô10kHZµÄ·ÖÆµÖµ£©
    RW,     //¼ÇÂ¼Ê±¼äÆ«ÖÃ£¨Ä¬ÈÏÒÔÐÅºÅ¹ýÁãµãÊ±¿Ì¡À50´ÎÊý¾Ý£©
    RO,     //Ä¿±êµçÁ÷Êý¾Ý¼¯
    RO,     //Êµ¼ÊµçÁ÷Êý¾Ý¼¯
    RO,     //Ä¿±êËÙ¶ÈÊý¾Ý¼¯
    RO,     //Êµ¼ÊËÙ¶ÈÊý¾Ý¼¯
    RO,     //Ä¿±êÎ»ÖÃÊý¾Ý¼¯
    RO,     //Êµ¼ÊÎ»ÖÃÊý¾Ý¼¯
  },
};

jCallback_t jointCb[CMDMAP_LEN];  // call back of read Joint ID


Joint* jointInit(uint16_t id, canSend_t canSend) {
  Joint* pJoint = (Joint*)malloc(sizeof(Joint));
  pJoint->basicModule->memoryTable = (uint16_t*)calloc(CMDMAP_LEN, sizeof(uint16_t));
  pJoint->basicModule->readDoneCb = (mCallback_t*)calloc(CMDMAP_LEN, sizeof(mCallback_t));
  pJoint->basicModule->writeDoneCb = (mCallback_t*)calloc(CMDMAP_LEN, sizeof(mCallback_t));
  pJoint->basicModule->accessType = (uint8_t*)joint_accessType;

  pJoint->basicModule->memoryTable[SYS_ID] = id;
  pJoint->jointId = (uint16_t*)&(pJoint->basicModule->memoryTable[SYS_ID]);
  pJoint->basicModule->moduleId = pJoint->jointId;
  pJoint->basicModule->canSend = canSend;

  pJoint->jointType = (uint16_t*)&(pJoint->basicModule->memoryTable[SYS_MODEL_TYPE]);

  return pJoint;
}

int32_t jointDestroy(Joint* pJoint) {
  if (pJoint) {
    if (pJoint->basicModule->memoryTable)
      free(pJoint->basicModule->memoryTable);
    if (pJoint->basicModule->readDoneCb)
      free(pJoint->basicModule->readDoneCb);
    if (pJoint->basicModule->writeDoneCb)
      free(pJoint->basicModule->writeDoneCb);

    free(pJoint);
    return 0;
  }
  return -1;
}

void jointMsgRoute(Joint* pJoint, Message* msg) {
  canDispatch(pJoint->basicModule, msg);
}

int32_t jointSetPVTSync(Joint* pJoint, uint32_t targetPos, uint32_t targetVel) {
  uint32_t buf[2];
  buf[0] = targetPos;
  buf[1] = targetVel;
  writeSyncMsg(pJoint->basicModule, 0x200, (void*)buf);
}


void jointStartServo(Joint* pJoint) {

}

void jointStopServo(Joint* pJoint) {

}

// callback of read command
int32_t _onReadEntry(void* handle, uint8_t index, void* args) {
  Module* d = (Module*)handle;
  if (jointCb[index]) {
    jointCb[index]((void*)d, args);
    jointCb[index] = 0;  // delete callback
  }

  return 0;
}

int32_t _onWriteEntry(void* handle, uint8_t index, void* args) {
  Module* d = (Module*)handle;
  if(index == TAG_WORK_MODE) {

  }
  return 0;
}

/// Get Information from Joints
int32_t jointGetId(Joint* pJoint, jCallback_t callBack) {
  readEntryCallback(pJoint->basicModule, SYS_ID, 2, _onReadEntry);
  jointCb[SYS_ID] = callBack;
  return 0;
}

int32_t jointGetType(Joint* pJoint, jCallback_t callBack) {
  readEntryCallback(pJoint->basicModule, SYS_MODEL_TYPE, 2, _onReadEntry);
  jointCb[SYS_MODEL_TYPE] = callBack;
  return 0;
}

int32_t jointGetVoltage(Joint* pJoint, jCallback_t callBack) {
  readEntryCallback(pJoint->basicModule, SYS_VOLTAGE, 2, _onReadEntry);
  jointCb[SYS_VOLTAGE] = callBack;
  return 0;
}

/// Set Value to Joints
int32_t jointSetMode(Joint* pJoint, uint16_t mode) {
  if (isJointMode(mode)) {
    writeEntryCallback(pJoint->basicModule, TAG_WORK_MODE, (void*)&mode, 2, _onReadEntry);
    return 0;
  }
  return -1;
}

