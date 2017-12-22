//#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include "master.h"

int32_t fillbuf(void* handle, uint16_t len) {
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t i;
    Joint* p = (Joint*)handle;
    for (i = len; i < MAX_BUFS; i++)
        jointPush(p, buf);
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
    Joint* joint1 = NULL;
    /* install signal handlers */
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    startMaster(0);
    MSG("Master Started.\n");
    
    setControlLoopFreq(200);
    joint1 = jointUp(0x01, masterLoadSendFunction(0));
    if (joint1) {
        if (jointSetModeTimeout(joint1, MODE_CYCLESYNC, 1000, NULL) == 0){
            MSG("Set mode to speed loop.\n");
        }
        jointStartServo(joint1, fillbuf);
    }
//    jointStopServo(joint1);

    joinMaster(0);
	system("pause");
    return 0;
}
