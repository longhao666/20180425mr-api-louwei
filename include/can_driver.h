#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#include <stdio.h>
#if defined Linux
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#define TASK_HANDLE pthread_t

#elif defined _WINDOWS
#include <Windows.h>
#define TASK_HANDLE HANDLE
#endif
#include "mrapi.h"  //should before logger.h
#include "logger.h"

#define delay_us(n) usleep(n)


/** 
 * @brief The CAN message structure 
 * @ingroup can
 */
typedef struct {
  uint16_t cob_id; /**< message's ID */
  uint8_t rtr;     /**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
  uint8_t len;     /**< message's length (0 to 8) */
  uint8_t data[8]; /**< message's datas */
} Message;

#define Message_Initializer {0,0,0,{0,0,0,0,0,0,0,0}}

typedef uint8_t (*canSend_t)(Message *);

static inline void print_message(Message* m)
{
    int i;
    char msg[256];
    sprintf(msg, "id:0x%02X rtr:%d len:%d data:", m->cob_id, m->rtr, m->len);

    for (i = 0 ; i < m->len ; i++)
        sprintf(msg, "%s 0x%02X",msg, m->data[i]);
    sprintf(msg, "%s\n", msg);
    RLOG("%s",msg);
}

#endif
