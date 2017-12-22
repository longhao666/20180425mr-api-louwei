#include <stdlib.h>
//#include <asm/types.h>
#include <signal.h>
#include <math.h>
#include <string.h>

#include "pcan_basic.h"

static CRITICAL_SECTION CanThread_mutex;

static struct timeval last_sig;

HANDLE timer = NULL;
DWORD timebuffer;
HANDLE timer_thread = NULL;
volatile int stop_timer=0;

TIMEVAL timerVal = MS_TO_TIMEVAL(1); //default 5ms

void setTimer(TIMEVAL value);

void usleep(__int64 usec)
{
    HANDLE timer;
    LARGE_INTEGER ft;

    ft.QuadPart = -(10*usec); // Convert to 100 nanosecond interval, negative value indicates relative time

    timer = CreateWaitableTimer(NULL, TRUE, NULL);
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
    WaitForSingleObject(timer, INFINITE);
    CloseHandle(timer);
}

void EnterMutex(void)
{
    EnterCriticalSection(&CanThread_mutex);
}

void LeaveMutex(void)
{
    LeaveCriticalSection(&CanThread_mutex);
}

void setTimerInterval(uint32_t t) {
    timerVal = t;
    setTimer(timerVal);
}

static void (*canTxPeriodic)(DWORD* tv) = NULL;
/// It's a cycle timer
int TimerThreadLoop(LPVOID arg)
{
	MSG("Go into TimerThreadLoop\n");
    while(!stop_timer)
    {
        WaitForSingleObject(timer, INFINITE);
		setTimer(timerVal);
		if(stop_timer)
            break;
		//        EnterMutex();
        timebuffer = GetTickCount();
        canTxPeriodic(&timebuffer);
//        LeaveMutex();
    }
	MSG("Go out of TimerThreadLoop\n");
	return 0;
}

void StartTimerLoop(int32_t hz, void* periodCall)
{
	unsigned long timer_thread_id;
	LARGE_INTEGER liDueTime;
    liDueTime.QuadPart = 0;

	stop_timer = 0;
	canTxPeriodic = periodCall;
    timer = CreateWaitableTimer(NULL, FALSE, NULL);
    if(NULL == timer)
    {
        MSG("CreateWaitableTimer failed (%d)\n", GetLastError());
    }

    // Take first absolute time ref in milliseconds.
    timebuffer = GetTickCount();

    timer_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)TimerThreadLoop, NULL, 0, &timer_thread_id);
}

void StopTimerLoop(void)//TimerCallback_t exitfunction)
{
    stop_timer = 1;
    setTimer(0);
    if(WaitForSingleObject(timer_thread, 1000) == WAIT_TIMEOUT)
    {
        TerminateThread(timer_thread, -1);
    }
    CloseHandle(timer);
    CloseHandle(timer_thread);
}

#define maxval(a,b) ((a>b)?a:b)
void setTimer(TIMEVAL value) //us
{
    if(value == TIMEVAL_MAX)
        CancelWaitableTimer(timer);
    else
    {
        LARGE_INTEGER liDueTime;

        /* arg 2 of SetWaitableTimer take 100 ns interval */
        liDueTime.QuadPart = ((long long) (-1) * value * 10);
        //printf("SetTimer(%llu)\n", value);

        if (!SetWaitableTimer(timer, &liDueTime, 0, NULL, NULL, FALSE))
        {
            MSG("SetWaitableTimer failed (%d)\n", GetLastError());
        }
    }
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
void* canReceiveLoop(void* arg)
{
    TPCANStatus status;
    CAN_HANDLE handle = (CAN_HANDLE)arg;
    HANDLE hEvent = NULL;
    Message rxMsg = Message_Initializer;

    hEvent = CreateEvent(NULL, FALSE, FALSE, L"ReceiveEvent");

    status = CAN_SetValue(handle, PCAN_RECEIVE_EVENT, &hEvent, sizeof(HANDLE));
    if (status != PCAN_ERROR_OK) {
        char strMsg[256];
        CAN_GetErrorText(status, 0, strMsg);
        MSG_ERROR("%s",strMsg);
        return NULL;
    }

    while (1) {
        if ( WAIT_OBJECT_0 == WaitForSingleObject(hEvent, INFINITE)) {
			ResetEvent(hEvent);
            if (canReceive_driver(handle, &rxMsg) == 1) {
                EnterMutex();
                if (canRxInterruptISR)
                    canRxInterruptISR(&rxMsg);
                LeaveMutex();
            }
        }
    }

    return NULL;
}

void CreateReceiveTask(CAN_HANDLE handle, TASK_HANDLE* Thread, void* ReceiveLoopPtr)
{
    unsigned long thread_id = 0;
    TPCANStatus status;

    InitializeCriticalSection(&CanThread_mutex);
	canRxInterruptISR = ReceiveLoopPtr;
	if (canRxInterruptISR == NULL) {
        MSG_ERROR("canRxInterruptISR cannot be NULL.\n");
        return;
    }

    *Thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)canReceiveLoop, (void*)handle, 0, &thread_id);

    MSG("pthread_create()\n");
}

void DestroyReceiveTask(TASK_HANDLE* Thread)
{
    DeleteCriticalSection(&CanThread_mutex);
	if (WaitForSingleObject(*Thread, 1000) == WAIT_TIMEOUT)
	{
		TerminateThread(*Thread, -1);
	}
	CloseHandle(*Thread);
}

void WaitReceiveTaskEnd(TASK_HANDLE* Thread) 
{
	WaitForSingleObject(*Thread, INFINITE);
}

