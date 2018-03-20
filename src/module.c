#include <string.h>
#include "module.h"

#define CMD_IN_PROGRESS -1
#define CMD_ACK_OK      1
#define CMD_ACK_NOK      2
#define CMD_IDLE        0

int32_t registerReadCallback(Module* d, uint8_t index, Callback_t callBack) {
  d->readDoneCb[index] = callBack;
  return 0;
}

int32_t writeSyncMsg(Module* d, uint16_t prefix, void* pSourceData) {
  Message txMsg = Message_Initializer;
  if (pSourceData == NULL) {
	  txMsg.cob_id = *(d->moduleId) + prefix;
	  txMsg.rtr = 0;
	  txMsg.len = 0;
  } else {
	  txMsg.cob_id = *(d->moduleId) + prefix;
	  txMsg.rtr = 0;
	  txMsg.len = 8;
	  memcpy((void*)(txMsg.data), pSourceData, 8);
  }

  d->canSend(&txMsg);
  return 0;
}

///write entry without reply
int32_t writeEntryNR(Module* d, uint8_t index, void* pSourceData, uint8_t dataType) {
  if ((d->accessType[index] != WO) && (d->accessType[index]) != RW) {
    ELOG("Write Entry Pemission");
    return -1;
  }
  Message txMsg = Message_Initializer;
  txMsg.cob_id = *d->moduleId;
  txMsg.rtr = 0;
  txMsg.len = 2 + dataType;
  txMsg.data[0] = CMDTYPE_WR_NR;
  txMsg.data[1] = index;
  memcpy((void*)&(txMsg.data[2]), pSourceData, dataType);

  d->canSend(&txMsg);
  return 0;
}

int32_t writeEntryCallback(Module* d, uint8_t index, void* pSourceData, uint8_t dataType, Callback_t callBack) {
  if ((d->accessType[index] != WO) && (d->accessType[index]) != RW) {
    ELOG("Write Entry Pemission");
    return -1;
  }
  Message txMsg = Message_Initializer;
  txMsg.cob_id = *d->moduleId;
  txMsg.rtr = 0;
  txMsg.len = 2 + dataType;
  txMsg.data[0] = CMDTYPE_WR;
  txMsg.data[1] = index;
  memcpy((void*)&(txMsg.data[2]), pSourceData, dataType);

  // Last index callback is set	
  d->writeDoneCb[index] = callBack;

  d->canSend(&txMsg);
  return 0;
}

int32_t readEntryCallback(Module* d, uint8_t index, uint8_t dataType, Callback_t callBack) {
  if ((d->accessType[index] != RO) && (d->accessType[index]) != RW) {
    DLOG("d->accessType[index] = %d", d->accessType[index]);
    ELOG("Read Entry Pemission");
    return -1;
  }
  Message txMsg = Message_Initializer;
  txMsg.cob_id = *d->moduleId;
  txMsg.rtr = 0;
  txMsg.len = 3;
  txMsg.data[0] = CMDTYPE_RD;
  txMsg.data[1] = index;
  txMsg.data[2] = dataType;

  d->readDoneCb[index] = callBack;

  d->canSend(&txMsg);
  return 0;
}

/// waiting for n us, if return MR_ERROR_OK, id will be stored in pJoint
int32_t moduleGet(uint8_t index, uint8_t datLen, Module* pModule, void* data, int32_t timeout, Callback_t callBack) { //us
	int16_t i;
	if (pModule == NULL) {
		return MR_ERROR_ILLDATA;
	}
	if (timeout == -1) {
		readEntryCallback(pModule, index, datLen, callBack);
		return MR_ERROR_OK;
	}
	//timeout is not INFINITE
	if (callBack != NULL) WLOG("callback will not work");
	if (pModule->readFlag[index] == CMD_IN_PROGRESS) {
		//reading in process
		return MR_ERROR_BUSY;
	}
	pModule->readFlag[index] = CMD_IN_PROGRESS;
	readEntryCallback(pModule, index, datLen, callBack);
	for (i = 0; i < timeout; i++) {
		if (pModule->readFlag[index] == CMD_ACK_OK) {
			if (data)
				memcpy(data, (void*)&(pModule->memoryTable[index]), datLen);
			pModule->readFlag[index] = CMD_IDLE;
			return MR_ERROR_OK;
		}
		delay_us(1);
	}
	pModule->readFlag[index] = CMD_IDLE;
	return MR_ERROR_TIMEOUT;
}

/// 如果timeout为infinite且callBack为空，则调用无返回的写函数writeEntryNR
int32_t moduleSet(uint8_t index, uint8_t datLen, Module* pModule, void* data, int32_t timeout, Callback_t callBack) { //us
	int16_t i;
	int32_t ret = 0;
	if (pModule == NULL) {
		return MR_ERROR_ILLDATA;
	}
	if (timeout == -1) { //INFINITE
		if (callBack == NULL) {
			writeEntryNR(pModule, index, data, datLen);
		}
		else {
			writeEntryCallback(pModule, index, data, datLen, callBack);
		}
		return MR_ERROR_OK;
	}
	//timeout is not INFINITE
	if (callBack != NULL) ILOG("callback will not work");
	if (pModule->writeFlag[index] == CMD_IN_PROGRESS) {
		//reading in process
		return MR_ERROR_BUSY;
	}
	pModule->writeFlag[index] = CMD_IN_PROGRESS;
	writeEntryCallback(pModule, index, data, datLen, callBack);
	for (i = 0; i < timeout; i++) {
		if (pModule->writeFlag[index] != CMD_IN_PROGRESS) {
			if (pModule->writeFlag[index] == CMD_ACK_OK) ret = MR_ERROR_ACK1;
			else if (pModule->writeFlag[index] == CMD_ACK_NOK) ret = MR_ERROR_ACK0;
			pModule->writeFlag[index] = CMD_IDLE;
			return ret;
		}
		delay_us(1);
	}
	pModule->writeFlag[index] = CMD_IDLE;
	return MR_ERROR_TIMEOUT;
}

int32_t setSyncReceiveMap(Module* d, uint16_t index[4]) {
    memcpy(d->syncReceiveIndex, index, 4*sizeof(uint16_t));
    return 0;
}

int32_t _setLocalEntry(Module* d, uint16_t index, uint8_t dataType, void* pDestData)
{
  uint8_t* pData = pDestData;
  if (index >= d->memoryLen) {
      return -1;
  }
  if (d->accessType[index] == NO_ACCESS ) {
      return -1;
  }
  // Firstly copy the data to memoryTable
  memcpy((void*)&(d->memoryTable[index]), pDestData, dataType);
  while(dataType) {
    if (d->readDoneCb[index]) {
      d->readDoneCb[index](*(d->moduleId), index, pData);
    }
    dataType -= 2;
    pData += 2;
    index += 1;
  }
  return 0;
}

void canDispatch(Module *d, Message *msg)
{
  uint16_t cob_id = msg->cob_id;
  uint16_t nodeid = getNodeId(cob_id);
  if (msg->rtr) {
    WLOG("remote frame received");
    return;
  }
  if (nodeid != *d->moduleId) {
    WLOG("Dismatched id!");
    return;
  }
  switch(cob_id>>8){
    // case 0x0:
    // uint8_t cmd = msg->data[0];
    // uint8_t index = msg->data[1];
    // if (cmd == CMDTYPE_WR) {

    // }
    // break;
    case 0x1: {   //ack 0x100+nodeid
      uint8_t cmd = msg->data[0];
      uint8_t index = msg->data[1];
      uint8_t len = msg->len - 2;
      if (cmd == CMDTYPE_WR) {
        uint8_t ack = msg->data[2];
		d->writeFlag[index] = CMD_ACK_OK;
        if (d->writeDoneCb[index]) {
          d->writeDoneCb[index](*(d->moduleId), index, &ack);
        }
      }
      else if ((cmd == CMDTYPE_RD) || (cmd == CMDTYPE_SCP)) {
		  d->readFlag[index] = CMD_ACK_OK;
		_setLocalEntry(d, index, len, (void*)&(msg->data[2]));
      }
    }
    break;
    case 0x3:
      _setLocalEntry(d, d->syncReceiveIndex[0], 2, (void*)&(msg->data[0]));
      _setLocalEntry(d, d->syncReceiveIndex[1], 2, (void*)&(msg->data[2]));
      _setLocalEntry(d, d->syncReceiveIndex[2], 2, (void*)&(msg->data[4]));
      _setLocalEntry(d, d->syncReceiveIndex[3], 2, (void*)&(msg->data[6]));
    break;

    default:
    break;
  }
}
