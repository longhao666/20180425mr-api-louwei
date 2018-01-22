/*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>

/* driver pcan pci for Peak board */
#include "pcan_basic.h" // for CAN_HANDLE

// Define for rtr CAN message
#define CAN_INIT_TYPE_ST PCAN_MESSAGE_STANDARD
#define CAN_INIT_TYPE_ST_RTR PCAN_MESSAGE_STANDARD | PCAN_MESSAGE_RTR 

/***************************************************************************/
int TranslateBaudeRate(char* optarg) {
	if (!strcmp(optarg, "1M")) return PCAN_BAUD_1M;
	if (!strcmp(optarg, "800K")) return PCAN_BAUD_800K;
	if (!strcmp(optarg, "500K")) return PCAN_BAUD_500K;
	if (!strcmp(optarg, "250K")) return PCAN_BAUD_250K;
	if (!strcmp(optarg, "125K")) return PCAN_BAUD_125K;
	if (!strcmp(optarg, "100K")) return PCAN_BAUD_100K;
	if (!strcmp(optarg, "50K")) return PCAN_BAUD_50K;
	if (!strcmp(optarg, "20K")) return PCAN_BAUD_20K;
	if (!strcmp(optarg, "10K")) return PCAN_BAUD_10K;
	if (!strcmp(optarg, "5K")) return PCAN_BAUD_5K;
	if (!strcmp(optarg, "none")) return 0;
	return 0x0000;
}

CAN_HANDLE TranslateCANHandle(char* optarg) {
	if (!strcmp(optarg, "pcanusb1")) return PCAN_USBBUS1;
	if (!strcmp(optarg, "pcanusb2")) return PCAN_USBBUS2;
	if (!strcmp(optarg, "pcanusb3")) return PCAN_USBBUS3;
	if (!strcmp(optarg, "pcanusb4")) return PCAN_USBBUS4;
	if (!strcmp(optarg, "pcanusb5")) return PCAN_USBBUS5;
	if (!strcmp(optarg, "pcanusb6")) return PCAN_USBBUS6;
	if (!strcmp(optarg, "pcanusb7")) return PCAN_USBBUS7;
	if (!strcmp(optarg, "pcanusb8")) return PCAN_USBBUS8;
	if (!strcmp(optarg, "pcanusb9")) return PCAN_USBBUS9;
	if (!strcmp(optarg, "pcanusb10")) return PCAN_USBBUS10;
	if (!strcmp(optarg, "pcanusb11")) return PCAN_USBBUS11;
	if (!strcmp(optarg, "pcanusb12")) return PCAN_USBBUS12;
	if (!strcmp(optarg, "pcanusb13")) return PCAN_USBBUS13;
	if (!strcmp(optarg, "pcanusb14")) return PCAN_USBBUS14;
	if (!strcmp(optarg, "pcanusb15")) return PCAN_USBBUS15;
	if (!strcmp(optarg, "pcanusb16")) return PCAN_USBBUS16;
	if (!strcmp(optarg, "pcanpci1")) return PCAN_PCIBUS1;
	if (!strcmp(optarg, "pcanpci2")) return PCAN_PCIBUS2;
	if (!strcmp(optarg, "pcanpci3")) return PCAN_PCIBUS3;
	if (!strcmp(optarg, "pcanpci4")) return PCAN_PCIBUS4;
	if (!strcmp(optarg, "pcanpci5")) return PCAN_PCIBUS5;
	if (!strcmp(optarg, "none")) return 0;
	return 0x0000;
}

uint8_t canChangeBaudRate_driver(CAN_HANDLE fd, char* baud)
{
	ILOG("canChangeBaudRate not yet supported by this driver\n");
	return 0;
}

/***************************************************************************/
CAN_HANDLE canOpen_driver(char* busname, char* baud)
{
	CAN_HANDLE handle = 0;
	int baudrate;
    TPCANParameter iBuffer;
	char sBuffer[256] = "./log";

	TPCANStatus status;

	handle = TranslateCANHandle(busname);

	if (handle && (baudrate = TranslateBaudeRate(baud)))
	{
		status = CAN_Initialize(handle, baudrate, 0, 0, 0);
		if (status != PCAN_ERROR_OK) {
			char errText[256];
			CAN_GetErrorText(status, 0, errText);
			ELOG("%s@%s", errText, __FUNCTION__);
			handle = PCAN_NONEBUS;
		}
		iBuffer = PCAN_PARAMETER_ON;
		CAN_SetValue(handle, PCAN_BUSOFF_AUTORESET, (void*)&iBuffer, sizeof(iBuffer));
		//iBuffer = PCAN_PARAMETER_ON;
		//CAN_SetValue(PCAN_NONEBUS, PCAN_LOG_LOCATION, (void*)&sBuffer, sizeof(sBuffer));
		//iBuffer = LOG_FUNCTION_ALL;
		//CAN_SetValue(handle, PCAN_LOG_CONFIGURE, (void*)&iBuffer, sizeof(iBuffer));
		//iBuffer = TRACE_FILE_SINGLE;
		//CAN_SetValue(handle, PCAN_TRACE_CONFIGURE, (void*)&iBuffer, sizeof(iBuffer));
		iBuffer = 0;
		CAN_SetValue(handle, PCAN_TRACE_SIZE, (void*)&iBuffer, sizeof(iBuffer));
		iBuffer = PCAN_PARAMETER_ON;
		//CAN_SetValue(PCAN_NONEBUS, PCAN_LOG_STATUS, (void*)&iBuffer, sizeof(iBuffer));
		CAN_SetValue(handle, PCAN_TRACE_STATUS, (void*)&iBuffer, sizeof(iBuffer));
	}
	else {
		ELOG("cannot opening %s @canOpen_driver", busname);
		handle = PCAN_NONEBUS;
	}

	return (CAN_HANDLE)handle;
}


/*********functions which permit to communicate with the board****************/
void canReset_driver(CAN_HANDLE handle, char* baud)
{
	int baudrate;
	TPCANStatus status;
	status = CAN_Uninitialize(handle);
	if (status != PCAN_ERROR_OK) {
		char errText[256];
		CAN_GetErrorText(status, 0, errText);
		ELOG("%s@%s", errText, __FUNCTION__);
	}

	if (handle && (baudrate = TranslateBaudeRate(baud)))
	{
		status = CAN_Initialize(handle, baudrate, 0, 0, 0);
		if (status != PCAN_ERROR_OK) {
			char errText[256];
			CAN_GetErrorText(status, 0, errText);
			ELOG("%s@%s", errText, __FUNCTION__);
		}
	}
	else {
		ELOG("error opening @canOpen_driver");
	}
}

uint8_t canReceive_driver(CAN_HANDLE handle, Message *m)
{
  uint8_t data, ret = 0; 
  TPCANStatus status;
  TPCANMsg peakMsg;
  TPCANTimestamp timeStamp;

  memset(&peakMsg, 0, sizeof(TPCANMsg));
  status = CAN_Read(handle, &peakMsg, &timeStamp);		// Poll
  if (status == PCAN_ERROR_OK)
  {
    m->cob_id = (uint16_t)peakMsg.ID;   
    if (peakMsg.MSGTYPE == CAN_INIT_TYPE_ST)         	/* bits of MSGTYPE_*/
      m->rtr = 0;
    else 
      m->rtr = 1;
    m->len = peakMsg.LEN;					/* count of data bytes (0..8) */
    for(data = 0  ; data < peakMsg.LEN ; data++)             			
      m->data[data] = peakMsg.DATA[data];         	/* data bytes, up to 8 */

  #if defined LOG_MSG_ON
	RLOG("%.4f", (double)((uint64_t)timeStamp.micros + (uint64_t)1000 * timeStamp.millis + (uint64_t)0x100000000 * 1000 * timeStamp.millis_overflow)/1000000.0);
	RLOG(" | IN    | ");
	print_message(m);
  #endif
    
    ret =  1;
  }
  else if (status != PCAN_ERROR_QRCVEMPTY) {
	char errText[256];
	CAN_GetErrorText(status, 0, errText);
	ELOG("%s@%s", errText, __FUNCTION__);
	ret = 0;
	if (status == PCAN_ERROR_BUSHEAVY) {
		ret = 2;
	}
  }
  else {}
  
  return ret;
}

/***************************************************************************/
uint8_t canSend_driver(CAN_HANDLE handle, Message const *m)
{
  uint8_t data;
  TPCANStatus status;
  TPCANMsg peakMsg;
  peakMsg.ID = m->cob_id;              			/* 11/29 bit code */
  if(m->rtr == 0)	
    peakMsg.MSGTYPE = CAN_INIT_TYPE_ST;       /* bits of MSGTYPE_*/
  else {
    peakMsg.MSGTYPE = CAN_INIT_TYPE_ST_RTR;       /* bits of MSGTYPE_*/
  }
  peakMsg.LEN = m->len;   
          			/* count of data bytes (0..8) */
  for(data = 0 ; data <  m->len; data ++)
  	peakMsg.DATA[data] = m->data[data];         	/* data bytes, up to 8 */
  
  status = CAN_Write(handle, & peakMsg);
  if(status != PCAN_ERROR_OK) {
    char errText[256];
    CAN_GetErrorText(status, 0, errText);
	ELOG("%s@%s", errText, __FUNCTION__);
	return 1;
  }
#if defined LOG_MSG_ON
  LOG_TIME();
  RLOG(" | OUT   | ");
  print_message((Message*)m);
#endif
  return 0;

}

/***************************************************************************/
int canClose_driver(CAN_HANDLE handle)
{
  TPCANStatus status = CAN_Uninitialize(handle);

  if (status != PCAN_ERROR_OK) {
    char errText[256];
    CAN_GetErrorText(status, 0, errText);
	ELOG("%s@%s", errText, __FUNCTION__);
	return -1;
  }

  return 0;
}
