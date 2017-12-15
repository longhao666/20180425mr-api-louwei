#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#include <stdint.h>
#include <stdio.h>

//#define DEBUG_MSG_CONSOLE_ON
#define DEBUG_MSG_CONSOLE_OFF

#define MSG   printf
#define MSG_WARN(str) printf("Warning: %s\n", str)
#define MSG_ERROR(str) printf("Error: %s\n", str)

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
    uint8_t fc;
    MSG("id:%02x ", m->cob_id);

    MSG(" rtr:%d", m->rtr);
    MSG(" len:%d", m->len);
    for (i = 0 ; i < m->len ; i++)
        MSG(" %02x", m->data[i]);
    MSG("\n");
}

#endif