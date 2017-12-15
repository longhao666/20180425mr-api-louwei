#include <stdlib.h>

#include <sys/time.h>

#include <signal.h>
// #include <time.h>
#include <string.h>

#include "can_peak_linux.h"

static pthread_mutex_t CanThread_mutex;

static struct timeval last_sig;

static timer_t timer;

TIMEVAL timerVal = MS_TO_TIMEVAL(5); //default 5ms

void setTimer(TIMEVAL value);

void EnterMutex(void)
{
    if(pthread_mutex_lock(&CanThread_mutex)) {
        perror("pthread_mutex_lock() failed\n");
    }
}

void LeaveMutex(void)
{
    if(pthread_mutex_unlock(&CanThread_mutex)) {
        perror("pthread_mutex_unlock() failed\n");
    }
}
static void (*Timer_1ms)(void*) = NULL;
static void (*Timer_5ms)(void*) = NULL;
static void (*Timer_10ms)(void*) = NULL;

void TimeDispatch(struct timeval* tv) {
    static uint16_t cnt5ms = 0;
    static uint16_t cnt10ms = 0;

    if (Timer_1ms) Timer_1ms(tv);

    if(cnt5ms++ >= 5) {
        cnt5ms = 0;
        if (Timer_5ms) Timer_5ms(tv);
    }
    if(cnt10ms++ >= 10) {
        cnt10ms = 0;
        if (Timer_10ms) Timer_10ms(tv);
    }
}

void setTimerInterval(uint32_t t) {
    timerVal = t;
}

int32_t setTimerCb_driver(uint8_t ms, void* timerPtr) {
    if (ms == 1) Timer_1ms = timerPtr;
    else if(ms == 5) Timer_5ms = timerPtr;
    else if(ms == 10) Timer_10ms = timerPtr;
    else return -1;
    return 0;
}

/// It's a cycle timer
void timer_notify(sigval_t val)
{
    setTimer(timerVal);
    if(gettimeofday(&last_sig,NULL)) {
        perror("gettimeofday()");
    }
    // EnterMutex();
    TimeDispatch(&last_sig);
    // LeaveMutex();
}

void StartTimerLoop(void)
{
    struct sigevent sigev;

    // Take first absolute time ref.
    if(gettimeofday(&last_sig,NULL)){
        perror("gettimeofday()");
    }

    memset (&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = 0;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = timer_notify;

    if(timer_create (CLOCK_REALTIME, &sigev, &timer)) {
        perror("timer_create()");
    }
    setTimer(timerVal);
}

void StopTimerLoop(void)//TimerCallback_t exitfunction)
{
    // EnterMutex();
    if(timer_delete (timer)) {
        perror("timer_delete()");
    }
    // exitfunction(NULL,0);
    // LeaveMutex();
}

#define maxval(a,b) ((a>b)?a:b)
void setTimer(TIMEVAL value)
{
//  printf("setTimer(TIMEVAL value=%d)\n", value);
    // TIMEVAL is us whereas setitimer wants ns...
    long tv_nsec = 1000 * (maxval(value,1)%1000000);
    time_t tv_sec = value/1000000;
    struct itimerspec timerValues;
    timerValues.it_value.tv_sec = tv_sec;
    timerValues.it_value.tv_nsec = tv_nsec;
    timerValues.it_interval.tv_sec = 0;
    timerValues.it_interval.tv_nsec = 0;

    timer_settime (timer, 0, &timerValues, NULL);
}


void canReceiveLoop_signal(int sig)
{
}
/* We assume that ReceiveLoop_task_proc is always the same */
static void (*canRxInterruptISR)(Message* msg) = NULL;
/**
 * Enter in realtime and start the CAN receiver loop
 * @param port
 */
void* canReceiveLoop(void* fd0)
{
    /*get signal*/
    //  if(signal(SIGTERM, canReceiveLoop_signal) == SIG_ERR) {
    //      perror("signal()");
    //}
    Message rxMsg;
    while (1) {
        MSG("CAN reading loop\n");
        if (canReceive_driver(fd0, &rxMsg) != 0) {
            EnterMutex();
            if (canRxInterruptISR)
                canRxInterruptISR(&rxMsg);
            LeaveMutex();
        }
    }

    return NULL;
}

void CreateReceiveTask(CAN_HANDLE fd0, TASK_HANDLE* Thread, void* ReceiveLoopPtr)
{
    int ret;

    canRxInterruptISR = ReceiveLoopPtr;
    if (canRxInterruptISR == NULL) {
        perror("canRxInterruptISR cannot be NULL.\n");
        return;
    }

    ret = pthread_mutex_init(&CanThread_mutex, NULL);
    if(pthread_create(Thread, NULL, canReceiveLoop, (void*)fd0)) {
        perror("pthread_create()\n");
    }
    MSG("pthread_create()\n");
}

void WaitReceiveTaskEnd(TASK_HANDLE *Thread)
{
    if(pthread_kill(*Thread, SIGTERM)) {
        perror("pthread_kill()");
    }
    if(pthread_join(*Thread, NULL)) {
        perror("pthread_join()");
    }
}

