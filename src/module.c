#include <string.h>
#include "module.h"


int32_t registerReadCallback(Module* d, uint8_t index, mCallback_t callBack) {
  d->readDoneCb[index] = callBack;
  return 0;
}

int32_t writeSyncMsg(Module* d, uint16_t prefix, void* pSourceData) {
  Message txMsg = Message_Initializer;
  if (pSourceData == NULL) {
	  txMsg.cob_id = *(d->moduleId) + prefix;
	  txMsg.rtr = 0;
	  txMsg.len = 0;
  }
  else {
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

int32_t writeEntryCallback(Module* d, uint8_t index, void* pSourceData, uint8_t dataType, mCallback_t callBack) {
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

int32_t readEntryCallback(Module* d, uint8_t index, uint8_t dataType, mCallback_t callBack) {
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
      d->readDoneCb[index](d, index, pData);
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
        if (d->writeDoneCb[index]) {
          d->writeDoneCb[index](d, index, &ack);
        }
      }
      else if ((cmd == CMDTYPE_RD) || (cmd == CMDTYPE_SCP)) {
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
