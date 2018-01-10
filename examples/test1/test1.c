//#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include "mrapi.h"

int32_t fillbuf(void* handle, uint16_t len) {
    uint32_t pos, spd;
    uint16_t i;
    for (i = len; i < MAX_BUFS; i++)
        jointPush(handle, &pos, &spd);
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

    startMaster(0);
    printf("Master Started.\n");
    
    setControlLoopFreq(200);
    joint1 = jointUp(0x01, masterLoadSendFunction(0));
    if (joint1) {
        if (jointSetMode(joint1, MODE_CYCLESYNC, 1000, NULL) == 0){
            printf("Set mode to speed loop.\n");
        }
        jointStartServo(joint1, fillbuf);
        jointSetPosition(joint1,0,5000,NULL);
    }
//    jointStopServo(joint1);
    joinMaster(0);
    return 0;
}
