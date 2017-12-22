#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#include <stdint.h>
#include <stdio.h>
#if defined Linux
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#elif defined _WINDOWS
#include <Windows.h>
#endif

#define DEBUG_MSG_CONSOLE_ON
// #define DEBUG_MSG_CONSOLE_OFF

#define delay_us(n) usleep(n)
#define MSG(...)   printf(__VA_ARGS__)
#define MSG_WARN(...) do{MSG("Warning: ");MSG(__VA_ARGS__);}while(0);
#define MSG_ERROR(...) do{MSG("Error: ");MSG(__VA_ARGS__);}while(0);

#define TASK_HANDLE HANDLE

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
    MSG("id:%02x ", m->cob_id);

    MSG(" rtr:%d", m->rtr);
    MSG(" len:%d", m->len);
    for (i = 0 ; i < m->len ; i++)
        MSG(" %02x", m->data[i]);
    MSG("\n");
}

#endif
