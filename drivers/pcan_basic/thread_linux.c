#include <stdlib.h>
#include <sys/time.h>
#include <asm/types.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <string.h>

#include "pcan_basic.h"

int32_t recTaskInitFlag = 0;

static void (*canTxPeriodic)(struct timeval* tv) = NULL;

void canReceiveLoop_signal(int sig)
{
}
/* We assume that ReceiveLoop_task_proc is always the same */
static void (*canRxInterruptISR)(Message* msg) = NULL;
/**
 * Enter in realtime and start the CAN receiver loop
 * @param port
 */
void* canReceiveLoop(void* arg)
{
    int fd;
    int ret;
    TPCANStatus status;
    fd_set Fds;
    CAN_HANDLE handle = (CAN_HANDLE)arg;
    Message rxMsg = Message_Initializer;
    pthread_mutex_t CanThread_mutex;

    /*get signal*/
    if(signal(SIGTERM, canReceiveLoop_signal) == SIG_ERR) {
        ELOG("signal()");
    }
    ret = pthread_mutex_init(&CanThread_mutex, NULL);
    status = CAN_GetValue(handle, PCAN_RECEIVE_EVENT, &fd, sizeof(int));
   
    if (status != PCAN_ERROR_OK) {
      char errText[256];
      CAN_GetErrorText(status, 0, errText);
      fprintf(stderr, "CAN_GetValue (PCANBasic) : %s.\n", errText);
        return NULL;
    }
    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&Fds);
    FD_SET(fd, &Fds);
    recTaskInitFlag = 1;
    while (select(fd+1, &Fds, NULL, NULL, NULL) > 0) {
        // MSG("CAN reading loop\n");
        if (canReceive_driver(handle, &rxMsg) != 0) {
            pthread_mutex_lock(&CanThread_mutex);
            if (canRxInterruptISR)
                canRxInterruptISR(&rxMsg);
            pthread_mutex_unlock(&CanThread_mutex);
        }
        else {
            //MSG("No in Message\n");
        }
        
    }
    pthread_mutex_destroy(&CanThread_mutex);

    return NULL;
}

void CreateReceiveTask(CAN_HANDLE handle, TASK_HANDLE* Thread, void* ReceiveLoopPtr)
{
    canRxInterruptISR = ReceiveLoopPtr;
    if (canRxInterruptISR == NULL) {
        ELOG("canRxInterruptISR cannot be NULL.\n");
        return;
    }

    if(pthread_create(Thread, NULL, canReceiveLoop, (void*)handle)) {
        ELOG("pthread_create()\n");
    }
    while (recTaskInitFlag == 0) {sleep(0);}
    DLOG("pthread_create()\n");
}

void DestroyReceiveTask(TASK_HANDLE* Thread)
{
    if(pthread_kill(*Thread, SIGTERM)) {
        ELOG("pthread_kill()");
    }
}

void WaitReceiveTaskEnd(TASK_HANDLE* Thread) 
{
    if(pthread_join(*Thread, NULL)) {
        ELOG("pthread_join()");
    }
}

