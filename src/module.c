#include <string.h>
#include "module.h"

int32_t registerSetEntryCallback(Module* d, uint8_t index, mCallback_t callBack) {
  d->readDoneCb[index] = callBack;
  return 0;
}

int32_t writeSyncMsg(Module* d, uint16_t prefix, void* pSourceData) {
  Message txMsg = Message_Initializer;
  txMsg.cob_id = *d->moduleId + prefix;
  txMsg.rtr = 0;
  txMsg.len = 8;
  memcpy((void*)(txMsg.data), pSourceData, 8);

  d->canSend(&txMsg);
  return 0;
}

///write entry without reply
int32_t writeEntryNR(Module* d, uint8_t index, void* pSourceData, uint8_t dataType) {
  if ((d->accessType[index] != WO) || (d->accessType[index]) != RW) {
    MSG_ERROR("Write Pemission");
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
  if ((d->accessType[index] != WO) || (d->accessType[index]) != RW) {
    MSG_ERROR("Write Pemission");
    return -1;
  }
  Message txMsg = Message_Initializer;
  txMsg.cob_id = *d->moduleId;
  txMsg.rtr = 0;
  txMsg.len = 2 + dataType;
  txMsg.data[0] = CMDTYPE_WR;
  txMsg.data[1] = index;
  memcpy((void*)&(txMsg.data[2]), pSourceData, dataType);

  d->writeDoneCb[index + dataType/2 - 1] = callBack;

  d->canSend(&txMsg);
  return 0;
}

int32_t readEntryCallback(Module* d, uint8_t index, uint8_t dataType, mCallback_t callBack) {
  if ((d->accessType[index] != RO) || (d->accessType[index]) != RW) {
    MSG_ERROR("Read Pemission");
    return -1;
  }
  Message txMsg = Message_Initializer;
  txMsg.cob_id = *d->moduleId;
  txMsg.rtr = 0;
  txMsg.len = 3;
  txMsg.data[0] = CMDTYPE_RD;
  txMsg.data[1] = index;
  txMsg.data[2] = dataType;

  d->readDoneCb[index + dataType/2 - 1] = callBack;

  d->canSend(&txMsg);
  return 0;
}


int32_t _setLocalEntry(Module* d, uint8_t index, uint8_t dataType, void* pDestData)
{
  uint16_t i;
  memcpy((void*)&(d->memoryTable[index]), pDestData, dataType);
  while(dataType) {
    if (d->readDoneCb[index]) {
      d->readDoneCb[index](d, index, pDestData);
    }
    dataType -= 2;
    pDestData += 2;
    index += 1;
  }
}

void canDispatch(Module *d, Message *msg)
{
  uint16_t cob_id = msg->cob_id;
  uint16_t nodeid = cob_id&0x000F;
  if (nodeid != *d->moduleId) {
    MSG_WARN("Dismatched id!");
    return;
  }
  switch(cob_id>>8){
    // case 0x00:
    // uint8_t cmd = msg->data[0];
    // uint8_t index = msg->data[1];
    // if (cmd == CMDTYPE_WR) {

    // }
    // break;
    case 0x10: {   //ack 0x100+nodeid
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
    case 0x20:
    break;
  }
}