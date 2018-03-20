// position profile test.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "MC_PositionProfile.h"
#include "mrapi.h"
#include <signal.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

JOINT_HANDLE joint1;
position_profile_t position_pro_strut;
volatile int stop_timer = 1;

float Pos_LinearRampPro_Init(void)
{
	int32_t initPos = 0;
	float initPos_f = 0.0f;
	float goalPos = 0;
    jointGetPosition(joint1, &initPos, 1000, NULL);
	initPos_f = (float)initPos / 65536.0f*360.0f;
	printf("\nCurrent position is %f, Please enter goal position : ", initPos_f);
    scanf("%f", &goalPos);
    position_profile_init(&position_pro_strut, goalPos, initPos_f,
		60.0f, 180.0f);
	return goalPos;
}

int TimerThreadLoop()
{
	int32_t ret;
	static int j = 0;
    JOINT_HANDLE handle = joint1;
	float realPos = 0.0f;
	float q, v;

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

#define maxval(a,b) ((a>b)?a:b)
void StartTimerLoop(int32_t hz, void* timer_notify)
{
    timer_t timerid;
    static struct timeval last_sig;
    struct itimerspec timerValues;
    struct sigevent sigev;
    float val;
    if (hz == 0) val = 0.0f;
    else val = 1000000.0/(float)hz;
    long tv_nsec = 1000 * (maxval((int)val,1)%1000000);
    time_t tv_sec = val/1000000;

    // Take first absolute time ref.
    if(gettimeofday(&last_sig,NULL)){
    }

    memset (&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = 0;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = timer_notify;

    if(timer_create (CLOCK_REALTIME, &sigev, &timerid)) {
        printf("timer created.\n");
    }

    timerValues.it_value.tv_sec = 1;
    timerValues.it_value.tv_nsec = 0;
    timerValues.it_interval.tv_sec = 0;
    timerValues.it_interval.tv_nsec = 5000000;

    if(timer_settime (timerid, 0, &timerValues, NULL) == -1) {
        perror("fail to timer_settime");
        exit(-1);
    }
}

int main()
{
	float goalPos = 0;

	startMaster("pcanusb1", MASTER(0));
	printf("Master Started\n");

    joint1 = jointUp(0x07, MASTER(0));
	if (joint1) {
		if (jointSetMode(joint1, MODE_CYCLESYNC, 1000, NULL) == MR_ERROR_ACK1) {
			printf("Set mode to MODE_CYCLESYNC.\n");
		}
		goalPos = Pos_LinearRampPro_Init();
	}
    StartTimerLoop(200, TimerThreadLoop);

	joinMaster(MASTER(0));

    return 0;
}

