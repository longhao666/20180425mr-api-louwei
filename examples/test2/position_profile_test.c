// position profile test.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "MC_PositionProfile.h"
#include "mrapi.h"
#if defined Linux
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#elif defined _WINDOWS
#include <Windows.h>
#include <math.h>
#endif

position_profile_t position_pro_strut;
volatile int stop_timer = 1;

float Pos_LinearRampPro_Init(void)
{
	int32_t initPos = 0;
	float initPos_f = 0.0f;
	float goalPos = 0;
	jointGetPosition(jointSelect(1), &initPos, 1000, NULL);
	initPos_f = (float)initPos / 65536.0f*360.0f;
	printf("\nCurrent position is %f, Please enter goal position : ", initPos_f);
#if defined Linux
    scanf("%f", &goalPos);
#endif
#if defined _WINDOWS
    scanf_s("%f", &goalPos);
#endif
    position_profile_init(&position_pro_strut, goalPos, initPos_f,
		60.0f, 180.0f);
	return goalPos;
}

int TimerThreadLoop(LPVOID arg)
{
	int32_t ret;
	static int j = 0;
	HANDLE timer = NULL;
	LARGE_INTEGER liDueTime;
	JOINT_HANDLE handle = arg;
	float realPos = 0.0f;
	LARGE_INTEGER m_nFreq;
	LARGE_INTEGER m_nTime;
	float q, v;

	timer = CreateWaitableTimer(NULL, FALSE, NULL);
	if (NULL == timer)
	{
		printf("CreateWaitableTimer failed (%d)", GetLastError());
	}

	/* arg 2 of SetWaitableTimer take 100 ns interval */
	liDueTime.QuadPart = ((long long)(-1) * 5000 * 10);  // 5ms

	if (!SetWaitableTimer(timer, &liDueTime, 0, NULL, NULL, FALSE))
	{
		printf("SetWaitableTimer failed (%d)", GetLastError());
		stop_timer = 1;
	}
	else {
		stop_timer = 0;
	}

	printf("Go into TimerThreadLoop\n");
	while (!stop_timer)
	{
		WaitForSingleObject(timer, INFINITE);
		QueryPerformanceFrequency(&m_nFreq); // 获取时钟周期 
		QueryPerformanceCounter(&m_nTime);   // 获取当前时间
		printf("time: %.4f\t", ((double)m_nTime.QuadPart / m_nFreq.QuadPart));

		if (!SetWaitableTimer(timer, &liDueTime, 0, NULL, NULL, FALSE))
		{
			printf("SetWaitableTimer failed (%d)", GetLastError());
			stop_timer = 1;
		}
		if (j < position_pro_strut.steps) {
			jointPoll(handle, &realPos, NULL, NULL);
			printf("real position: %3.3f\t", realPos);

			q = position_profile_caculate(&position_pro_strut, j++);
			v = position_pro_strut.v;

			ret = jointPush(handle, q, v, 0);
			printf("target position: %3.3f\n", q);
		}
		else {
			jointPoll(handle, &realPos, NULL, NULL);
			printf("real position: %3.3f\t", realPos);
			if (fabs(realPos - position_pro_strut.qf) > 0.006) {
				ret = jointPush(handle, position_pro_strut.qf, 0, 0);
				printf("target position: %3.3f\n", q);
			}
			else {
				Pos_LinearRampPro_Init();
				j = 0;
			}
		}
	}
	printf("Go out of TimerThreadLoop\n");
	return 0;
}


int main()
{
	HANDLE timer_thread = NULL;
	unsigned long timer_thread_id;
	JOINT_HANDLE joint1;
	GRIPPER_HANDLE gripper1;
	float goalPos = 0;

	startMaster("pcanusb1", MASTER(0));
	printf("Master Started\n");

	joint1 = jointUp(0x01, MASTER(0));
	if (joint1) {
		if (jointSetMode(joint1, joint_cyclesync, 1000, NULL) == MR_ERROR_ACK1) {
			printf("Set mode to MODE_CYCLESYNC.\n");
		}
		goalPos = Pos_LinearRampPro_Init();
		timer_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)TimerThreadLoop, joint1, 0, &timer_thread_id);
	}
	gripper1 = gripperUp(8, MASTER(0));

	if (gripper1) {
		if (gripperSetMode(gripper1, gripper_servo, 1000, NULL) == MR_ERROR_OK) {
			gripperPush(gripper1, 0, 0);
		}

	}

	joinMaster(MASTER(0));
#ifdef _WINDOWS
	system("pause");
#endif

    return 0;
}

