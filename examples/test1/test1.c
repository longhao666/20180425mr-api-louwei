//#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include "master.h"

void timer1ms(void* curTime) {
    static int cnt = 0;
    struct timeval* tv = (struct timeval*) curTime;
    // MSG("1ms");
    if (tv) {
        MSG("cnt:%5d\t", cnt++);
        MSG("tv_sec:%ld\t", tv->tv_sec);
        MSG("tv_usec:%ld\n", tv->tv_usec);
    }
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
    stopMaster();
    exit(0);
}

int main(int argc, char const *argv[])
{
    /* code */
    /* install signal handlers */
    // mlockall(MCL_CURRENT | MCL_FUTURE);
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    startMaster();
    MSG("Master Started.\n");
    // setTimerCallback(10, timer1ms);
    
    joinMaster();
    // while(1) {
    //     sleep(1);
    // }


    return 0;
}