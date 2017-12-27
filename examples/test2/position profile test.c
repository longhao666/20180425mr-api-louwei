// position profile test.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "MC_PositionProfile.h"
#include "mrapi.h"
#include <math.h>

position_profile_t position_pro_strut;

int32_t Pos_LinearRampPro_Init(void)
{
	int32_t initPos = 0;
	int32_t goalPos = 0;
	jointGetPosition(jointSelect(1), &initPos, 1000, NULL);
	printf("Current position is %d, Please enter goal position : ", initPos);
	scanf_s("%d", &goalPos);
	position_profile_init(&position_pro_strut, goalPos, initPos,
		32768, 32768);
	return goalPos;
}

int32_t fillbuf(JOINT_HANDLE handle, uint16_t len) {
	int32_t buf[2];
	uint16_t i;
	static int j = 0;
	for (i = len; i < MAX_BUFS; i++) {
		if (j < position_pro_strut.steps) {
			float q = position_profile_caculate(&position_pro_strut, j++);
			float v = position_pro_strut.v;
			buf[0] = lroundf(q);
			buf[1] = lroundf(v);
			jointPush(handle, &buf[0], &buf[1]);
		}
	}
	return 0;
}

int main()
{
	JOINT_HANDLE joint1;
	int32_t goalPos = 0;
	int32_t realPos = 0;

	startMaster(0);
	printf("Master Started\n");

	setControlLoopFreq(200);
	joint1 = jointUp(0x01, masterLoadSendFunction(0));
	if (joint1) {
		goalPos = Pos_LinearRampPro_Init();
		if (jointSetMode(joint1, MODE_CYCLESYNC, 1000, NULL) == MR_ERROR_ACK1) {
			printf("Set mode to MODE_CYCLESYNC.\n");
		}
		jointStartServo(joint1, fillbuf);
		//jointSetPosition(joint1, 65536, 1000, NULL);
		//jointGetPosition(joint1, NULL, 1000, NULL);
	}
	do {
		jointPoll(joint1, &realPos, NULL);
		Sleep(1);
	} while (abs(realPos-goalPos) > 10);
	jointStopServo(joint1);


	joinMaster(0);
	system("pause");

    return 0;
}

