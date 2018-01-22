//#include <time.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include "mrapi.h"

int32_t fillbuf(JOINT_HANDLE handle, uint16_t len) {
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int32_t ret;
	int32_t pos, speed;
	do {
		ret = jointPush(handle, &pos, &speed, NULL);
	} while (ret == MR_ERROR_OK);
	return 0;
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
    stopMaster(0);
    exit(0);
}

int main(int argc, char const *argv[])
{
    /* code */
    JOINT_HANDLE joint1 = NULL;
    /* install signal handlers */
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    startMaster("pcanusb1", 0);
    printf("Master Started\n");
    
    setControlLoopFreq(200);
    joint1 = jointUp(0x01, MASTER(0));
    if (joint1) {
        if (jointSetMode(joint1, MODE_CYCLESYNC, 1000, NULL) == 0){
			printf("Set mode to speed loop\n");
        }
        jointStartServo(joint1, fillbuf);
		//jointSetPosition(joint1, 65536, 1000, NULL);
		jointGetPosition(joint1, NULL, 1000, NULL);
    }
//    jointStopServo(joint1);

    joinMaster(0);
	system("pause");
    return 0;
}
