#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <string.h>
#include <Windows.h>

#include "pcan_basic.h"


//// Timer is the same when different can device is open
static struct timeval last_sig;

HANDLE timer = NULL;
DWORD timebuffer;
HANDLE timer_thread = NULL;
volatile int stop_timer = 1;

int32_t recTaskInitFlag = 0;

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

void setTimerInterval(uint32_t t) {
    timerVal = t;
    setTimer(timerVal);
}

static void (*canTxPeriodic)(DWORD* tv) = NULL;
/// It's a cycle timer
int TimerThreadLoop(LPVOID arg)
{
	ILOG("Go into TimerThreadLoop");
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
	ILOG("Go out of TimerThreadLoop");
	return 0;
}

void StartTimerLoop(int32_t hz, void* periodCall)
{
	unsigned long timer_thread_id;
	LARGE_INTEGER liDueTime;
	float val = 0;

	if (stop_timer == 0) {
		ILOG("Timer has already been started");
		return;
	}
	
	liDueTime.QuadPart = 0;

	stop_timer = 0;
	canTxPeriodic = periodCall;
    timer = CreateWaitableTimer(NULL, FALSE, NULL);
    if(NULL == timer)
    {
        ILOG("CreateWaitableTimer failed (%d)", GetLastError());
    }

    // Take first absolute time ref in milliseconds.
    timebuffer = GetTickCount();

    timer_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)TimerThreadLoop, NULL, 0, &timer_thread_id);
	if (hz != -1) {
		val = 1000000.0f / (float)hz;
	}
	//setTimerInterval(round(val));
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
        //MSG("SetTimer(%llu)\n", value);

        if (!SetWaitableTimer(timer, &liDueTime, 0, NULL, NULL, FALSE))
        {
            ILOG("SetWaitableTimer failed (%d)", GetLastError());
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
	uint8_t recRet;
    TPCANStatus status;
    CAN_HANDLE handle = (CAN_HANDLE)arg;
    HANDLE hEvent = NULL;
    Message rxMsg = Message_Initializer;
	CRITICAL_SECTION CanThread_mutex;

	InitializeCriticalSection(&CanThread_mutex);
	hEvent = CreateEvent(NULL, FALSE, FALSE, L"ReceiveEvent");

    status = CAN_SetValue(handle, PCAN_RECEIVE_EVENT, &hEvent, sizeof(HANDLE));
    if (status != PCAN_ERROR_OK) {
        char strMsg[256];
        CAN_GetErrorText(status, 0, strMsg);
        ELOG("CAN_SetValue : %s",strMsg);
        return NULL;
    }
	//Init FLAG is set to avoid message coming while event hasn't set
	recTaskInitFlag = 1;
    while (1) {
        if ( WAIT_OBJECT_0 == WaitForSingleObject(hEvent, INFINITE)) {
			//first of all, you are creating an auto-reset event, so you don't need to call ResetEvent() on your handle.
			//ResetEvent(hEvent);
            do {
				recRet = canReceive_driver(handle, &rxMsg);
				EnterCriticalSection(&CanThread_mutex);
                if (canRxInterruptISR)
                    canRxInterruptISR(&rxMsg);
				LeaveCriticalSection(&CanThread_mutex);
			} while (recRet == 1);
			if (recRet == 2) {
				ILOG("Please check if there is any module on the bus.CAN bus 0x%x will be reset with baudrate of 1M", handle);
				CAN_Reset(handle);
				canReset_driver(handle, "1M");
				CAN_SetValue(handle, PCAN_RECEIVE_EVENT, &hEvent, sizeof(HANDLE));
			}
        }
    }

    return NULL;
}

void CreateReceiveTask(CAN_HANDLE handle, TASK_HANDLE* Thread, void* ReceiveLoopPtr)
{
    unsigned long thread_id = 0;

	canRxInterruptISR = ReceiveLoopPtr;
	if (canRxInterruptISR == NULL) {
        ELOG("canRxInterruptISR cannot be NULL.\n");
        return;
    }

    *Thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)canReceiveLoop, (void*)handle, 0, &thread_id);

	while (recTaskInitFlag != 1) { Sleep(0); }

    ILOG("Receive Task created");
}

void DestroyReceiveTask(TASK_HANDLE* Thread)
{
 //   DeleteCriticalSection(&CanThread_mutex);
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

