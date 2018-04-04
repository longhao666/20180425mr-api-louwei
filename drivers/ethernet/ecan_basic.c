#include <pcap/pcap.h>
#include <stdio.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <malloc.h>
#include <unistd.h>
#include "ecan_basic.h"

pcap_t* device = NULL;
pthread_t thread = (pthread_t)NULL;
static void (*canRxInterruptISR)(Message* msg) = NULL;
int loopFlag = 0;

void getPacket(u_char * arg, const struct pcap_pkthdr * pkthdr, const u_char * packet);

void canReceiveLoop_signal(int sig)
{
    if (sig == SIGTERM){
       pthread_exit(NULL);
    }
}

void *canReceiveLoop(void *handle)
{
    signal(SIGTERM, canReceiveLoop_signal);
    /* wait loop forever */

    loopFlag = 1;
    pcap_loop(device, -1, getPacket, NULL);
    pthread_exit(NULL);
    return NULL;
}

void CreateReceiveTask(CAN_HANDLE fd0, TASK_HANDLE* Thread, void* ReceiveLoopPtr)
{
    if (thread != (pthread_t)NULL) return;
    canRxInterruptISR = ReceiveLoopPtr;
    pthread_create(Thread, NULL, (void *(*)(void *))canReceiveLoop, NULL);
    while(!loopFlag){sleep(0);}
    thread = *Thread;
}

void DestroyReceiveTask(TASK_HANDLE* Thread)
{
    pthread_kill(thread, SIGTERM);
}

void WaitReceiveTaskEnd(TASK_HANDLE* Thread)
{
    pthread_join(thread, NULL);
}

uint8_t canChangeBaudRate_driver(CAN_HANDLE fd, char* baud)
{}

CAN_HANDLE canOpen_driver(const char* busno, const char* baud)
{
    char errBuf[PCAP_ERRBUF_SIZE];
    char devStr[256];
    int canId;

    (void)baud;

    sscanf(busno, "%[^-]-%d", devStr, &canId);

    // ethernet has been initiallized
    if (device != NULL) return canId;
    device = pcap_create(devStr, errBuf);
    if (!device) {
        printf("error: %s\n", errBuf);
        return 0;
    }

    pcap_set_immediate_mode(device, 1);
    pcap_set_buffer_size(device, 10);
    pcap_activate(device);

    return (char)canId;
}

void canReset_driver(CAN_HANDLE handle, char* baud){}

uint8_t canSend_driver(CAN_HANDLE fd0, Message const *m)
{
    char errBuf[PCAP_ERRBUF_SIZE];
    char *sendBuf;

//    printf("sending : %d\n", fd0);
    sendBuf = malloc(15 + sizeof(Message));
    for (int i = 0; i < 12; i++) sendBuf[i] = 0xFF;
    sendBuf[12] = 0x12;
    sendBuf[13] = 0x34;
    sendBuf[14] = fd0;
    memcpy(sendBuf+15, (void*)m, sizeof(Message));
    if (pcap_inject(device, (const void*)sendBuf, 15 + sizeof(Message)) == -1) {
        pcap_perror(device, errBuf);
        printf("Couldn't send frame: %s\n", errBuf);
        return 2;
    }
}

uint8_t canReceive_driver(CAN_HANDLE fd0, Message *m){}

int canClose_driver(CAN_HANDLE handle)
{
    pcap_close(device);
}

void getPacket(u_char * arg, const struct pcap_pkthdr * pkthdr, const u_char * packet)
{
    Message m;

    if((packet[12] == 0x12) && (packet[13] == 0x34)) {
        memcpy(&m, packet+15, sizeof(Message));
//        printf("id: %d, len: %d, rtr: %d\ndata: ", m.cob_id, m.len, m.rtr);
//        for (int i = 0; i < m.len; i++) {
//            printf("0x%X ", m.data[i]);
//        }
//        printf("\n");
        if(canRxInterruptISR) canRxInterruptISR(&m);
    }
}

