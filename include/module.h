#ifndef __MODULE_H__
#define __MODULE_H__
#include <stdio.h>
#include "mrapi.h"
#include "can_driver.h"

/** Each entry of the object dictionary can be READONLY (RO), READ/WRITE (RW),
 *  WRITE-ONLY (WO)
 */
#define RW     0x00  
#define WO     0x01
#define RO     0x02
#define NO_ACCESS 0x03

//指令类型宏定义
#define CMDTYPE_RD            0x01   //读控制表指令
#define CMDTYPE_WR            0x02   //写控制表指令
#define CMDTYPE_WR_NR         0x03   //写控制表指令（无返回）
#define CMDTYPE_SCP           0x05   //示波器数据返回指令（保留）

#define MODULE_OFFLINE 0
#define MODULE_ONLINE 1


#define getNodeId(cob_id) (cob_id&0x00FF)

//typedef int32_t (*mCallback_t)(uint16_t id, uint16_t index, void* args);

/************************ STRUCTURES ****************************/
typedef struct td_module
{
    uint16_t*  moduleId;
    canSend_t canSend;
    uint16_t* memoryTable;
    uint16_t memoryLen;
    uint8_t* accessType;
    Callback_t* readDoneCb;
    Callback_t* writeDoneCb;
    uint16_t syncReceiveIndex[4];
	uint16_t* readFlag;
	uint16_t* writeFlag;
	uint8_t isOnline;
}Module;

#ifdef __cplusplus
extern "C"
{
#endif

int32_t readEntryCallback(Module* d, uint8_t index, uint8_t dataType, Callback_t callBack);
int32_t writeEntryCallback(Module* d, uint8_t index, void* pSourceData, uint8_t dataType, Callback_t callBack);
int32_t writeSyncMsg(Module* d, uint16_t prefix, void* pSourceData);
int32_t writeEntryNR(Module* d, uint8_t index, void* pSourceData, uint8_t dataType);

int32_t registerReadCallback(Module* d, uint8_t index, Callback_t callBack);
int32_t setSyncReceiveMap(Module* d, uint16_t index[4]);

int32_t moduleGet(uint8_t index, uint8_t datLen, Module* pModule, void* data, int32_t timeout, Callback_t callBack);
int32_t moduleSet(uint8_t index, uint8_t datLen, Module* pModule, void* data, int32_t timeout, Callback_t callBack);

#ifdef __cplusplus
}
#endif
#endif
