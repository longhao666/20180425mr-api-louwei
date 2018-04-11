#include <string.h>
#include <stdlib.h>
#include "can_stm32.h"
#include "stm32f4xx.h"

void CAN1_RX0_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);

////CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1TQ~CAN_SJW_4TQ
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1TQ~CAN_BS2_8TQ;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1TQ~CAN_BS1_16TQ
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+tbs2+1)*brp); 其中tbs1和tbs2我们只用关注标识符上标志的序号，例如CAN_BS2_1TQ，我们就认为tbs2=1来计算即可。
//mode:CAN_MODE_NORMAL,普通模式;CAN_MODE_LOOPBACK,回环模式;
//Fpclk1的时钟在初始化的时候设置为45M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_8tq,6,CAN_MODE_LOOPBACK);
//则波特率为:45M/((6+8+1)*6)=500Kbps
//则波特率为:45M/((3+5+1)*5)=1Mbps
//返回值:0,初始化OK;
//    其他,初始化失败; 

CAN_HANDLE CAN_Mode_Init(uint8_t canId)
{
  CAN_FilterConfTypeDef  CAN_FilerConf;
  CAN_HANDLE fd = malloc(sizeof(CAN_HandleTypeDef));
  CanTxMsgTypeDef* TxMessage = malloc(sizeof(CanTxMsgTypeDef));
  CanRxMsgTypeDef* RxMessage = malloc(sizeof(CanRxMsgTypeDef));
  
  if (canId == 1) fd->Instance = CAN1; 
  else if (canId == 2) fd->Instance = CAN2; 
  else {
    free(TxMessage);
    free(RxMessage);
    free(fd);
    return NULL;   //初始化
  }

// 	CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS2_3TQ,CAN_BS1_5TQ,5,CAN_MODE_NORMAL); //CAN初始化,波特率1Mbps 
  fd->pTxMsg = TxMessage;      // 发送消息
  fd->pRxMsg = RxMessage;      // 接收消息
  fd->Init.Prescaler = 5;    // 分频系数(Fdiv)为brp+1
  fd->Init.Mode = CAN_MODE_NORMAL;        // 模式设置 
  fd->Init.SJW = CAN_SJW_1TQ;         // 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1TQ~CAN_SJW_4TQ
  fd->Init.BS1 = CAN_BS1_5TQ;         // tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ
  fd->Init.BS2 = CAN_BS2_3TQ;         // tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ
  fd->Init.TTCM = DISABLE;     // 非时间触发通信模式 
  fd->Init.ABOM = DISABLE;     // 软件自动离线管理
  fd->Init.AWUM = DISABLE;     // 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  fd->Init.NART = ENABLE;      // 禁止报文自动传送 
  fd->Init.RFLM = DISABLE;     // 报文不锁定,新的覆盖旧的 
  fd->Init.TXFP = DISABLE;     // 优先级由报文标识符决定 

  if(HAL_CAN_Init(fd)!=HAL_OK) {
    free(TxMessage);
    free(RxMessage);
    free(fd);
    return NULL;   //初始化
  }
    
  CAN_FilerConf.FilterIdHigh = 0X0000;     //32位ID
  CAN_FilerConf.FilterIdLow = 0X0000;
  CAN_FilerConf.FilterMaskIdHigh = 0X0000; //32位MASK
  CAN_FilerConf.FilterMaskIdLow = 0X0000;  
  CAN_FilerConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  CAN_FilerConf.FilterNumber = 0;          //过滤器0
  CAN_FilerConf.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilerConf.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilerConf.FilterActivation = ENABLE; //激活滤波器0
  CAN_FilerConf.BankNumber = 14;

  if(HAL_CAN_ConfigFilter(fd, &CAN_FilerConf) != HAL_OK) {
    free(TxMessage);
    free(RxMessage);
    free(fd);
    return NULL;//滤波器初始化
  }

  return fd;
}

//CAN底层驱动，引脚配置，时钟配置，中断配置
//此函数会被HAL_CAN_Init()调用
//hcan:CAN句柄
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_Initure;
  
  __HAL_RCC_CAN1_CLK_ENABLE();                //使能CAN1时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();			    //开启GPIOA时钟

  GPIO_Initure.Pin = GPIO_PIN_11|GPIO_PIN_12;   //PA11,12
  GPIO_Initure.Mode = GPIO_MODE_AF_PP;          //推挽复用
  GPIO_Initure.Pull = GPIO_PULLUP;              //上拉
  GPIO_Initure.Speed = GPIO_SPEED_FAST;         //快速
  GPIO_Initure.Alternate = GPIO_AF9_CAN1;       //复用为CAN1
  HAL_GPIO_Init(GPIOA, &GPIO_Initure);         //初始化
  
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);//FIFO0消息挂起中断允许.	  
  //CAN1->IER|=1<<1;		//FIFO0消息挂起中断允许.	
//    HAL_NVIC_SetPriority(CAN1_RX0_IRQn,1,2);    //抢占优先级1，子优先级2
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);          //使能中断
}

uint8_t CAN_hw_rd(CAN_TypeDef* device, Message *msg, int32_t fifo_id)
{
  msg->cob_id = 0x000007FFUL & (device->sFIFOMailBox[fifo_id].RIR >> 21);
	//Read type information
	if ((device->sFIFOMailBox[fifo_id].RIR & (1 << 1)) == 0) msg->rtr = 0;		//DATA   FRAME
	else msg->rtr = 1;													//REMOTE FRAME
	//Read length (number of received bytes)
	msg->len = (uint32_t)0x0000000F & device->sFIFOMailBox[fifo_id].RDTR;
	//Read data bytes
	memcpy((void*)&(msg->data[0]), (const void*)&(device->sFIFOMailBox[fifo_id].RDLR), 4);
	memcpy((void*)&(msg->data[4]), (const void*)&(device->sFIFOMailBox[fifo_id].RDHR), 4);
  
  return 0;
}

/* We assume that ReceiveLoop_task_proc is always the same */
void (*canRxInterruptISR)(Message* msg) = NULL;

void usleep(int usec) {}

int TranslateBaudeRate(const char* optarg) {
	if (!strcmp(optarg, "1M")) return 1000000;
	if (!strcmp(optarg, "800K")) return 800000;
	if (!strcmp(optarg, "500K")) return 500000;
	if (!strcmp(optarg, "250K")) return 250000;
	if (!strcmp(optarg, "125K")) return 125000;
	if (!strcmp(optarg, "100K")) return 100000;
	if (!strcmp(optarg, "50K")) return 50000;
	if (!strcmp(optarg, "20K")) return 20000;
	if (!strcmp(optarg, "10K")) return 10000;
	if (!strcmp(optarg, "5K")) return 5000;
	if (!strcmp(optarg, "none")) return 0;
	return 0x0000;
}

//设置CAN网的位时序
static void CAN_set_timing (CAN_HANDLE fd, uint32_t tseg1, uint32_t tseg2, uint32_t sjw, uint32_t brp)
{
	fd->Instance->BTR &= ~(((          0x03) << 24) | ((            0x07) << 20) | ((            0x0F) << 16) | (          0x3FF));
	fd->Instance->BTR |=  ((((sjw-1) & 0x03) << 24) | (((tseg2-1) & 0x07) << 20) | (((tseg1-1) & 0x0F) << 16) | ((brp-1) & 0x3FF));
}

void CreateReceiveTask(CAN_HANDLE fd, TASK_HANDLE* Thread, void* ReceiveLoopPtr) 
{
  if (fd == NULL) return;
  canRxInterruptISR = (void (*)(Message*))ReceiveLoopPtr;
  if ((fd->Instance == CAN1) && Thread) *Thread = (int)CAN1_RX0_IRQHandler;
  if((fd->Instance == CAN2) && Thread) *Thread = (int)CAN2_RX0_IRQHandler;
}

void DestroyReceiveTask(TASK_HANDLE* Thread) 
{
  if (*Thread == (int)CAN1_RX0_IRQHandler)
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);          // 关闭中断
  else if (*Thread == (int)CAN2_RX0_IRQHandler)
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);          // 关闭中断
}
  
void WaitReceiveTaskEnd(TASK_HANDLE* Thread) 
{
  
}
  

uint8_t canChangeBaudRate_driver(CAN_HANDLE fd, char* baud) {
#define CAN_CLK 45000000
  int baudrate = TranslateBaudeRate(baud);
  uint32_t brp;
  
	if (baudrate <= 500000)
	{
		brp  = (CAN_CLK / 18) / baudrate;
		CAN_set_timing(fd, 12, 5, 1, brp);
	}
	else if (baudrate <= 1000000)
	{
		brp  = (CAN_CLK / 9) / baudrate;
		CAN_set_timing(fd, 5, 3, 1, brp);
	}
}
  
// CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS2_6TQ,CAN_BS1_8TQ,6,CAN_MODE_NORMAL); //CAN初始化,波特率500Kbps      
  
CAN_HANDLE canOpen_driver(const char* busno, const char* baud) {
  uint8_t canId = 0;
  CAN_HANDLE fd;
  
  if (strcmp(busno, "can1") == 0) canId = 1;
  else if (strcmp(busno, "can2") == 0) canId = 2;
  
  fd = CAN_Mode_Init(canId);
//  canChangeBaudRate_driver(fd, (char*)baud);
  return fd;
}
  
void canReset_driver(CAN_HANDLE handle, char* baud) {}
  
uint8_t canSend_driver(CAN_HANDLE fd, Message const *m) {
  uint8_t MailBox = 0;
  /* Select one empty transmit mailbox */
  if ((fd->Instance->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)  MailBox = 0;
  else if ((fd->Instance->TSR&CAN_TSR_TME1) == CAN_TSR_TME1) MailBox = 1;
  else if ((fd->Instance->TSR&CAN_TSR_TME2) == CAN_TSR_TME2) MailBox = 2;
  else return 1; // 
  
	// Reset TIR register
	fd->Instance->sTxMailBox[MailBox].TIR  = 0;          			//reset TXRQ bit
	//Setup the identifier information
  fd->Instance->sTxMailBox[MailBox].TIR |= (uint32_t)(m->cob_id << 21);
	//Setup type information 
	if (m->rtr == 1)
	{                        								//REMOTE FRAME
		fd->Instance->sTxMailBox[MailBox].TIR |= (1 << 1);
	}
	//Setup data bytes
	fd->Instance->sTxMailBox[MailBox].TDLR = *(uint32_t*)&m->data[0];
	fd->Instance->sTxMailBox[MailBox].TDHR = *(uint32_t*)&m->data[4];
	//Setup length
	fd->Instance->sTxMailBox[MailBox].TDTR &= ~0x0000000F;
	fd->Instance->sTxMailBox[MailBox].TDTR |=  (m->len & 0x0000000F);
	fd->Instance->IER |= (1 << 0);                           		//enable  TME s32errupt
	//transmit message
	fd->Instance->sTxMailBox[MailBox].TIR  |=  (1UL << 0);      //??????
	return 0;
}

uint8_t canReceive_driver(CAN_HANDLE fd, Message *m) {
  return CAN_hw_rd(fd->Instance, m, 0);
}
int canClose_driver(CAN_HANDLE fd) {
  if (fd == NULL) return 0;
  if (fd->Instance == CAN1)
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);          // 关闭中断
  else if (fd->Instance == CAN2)
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);          // 关闭中断
  HAL_CAN_DeInit(fd);
  free(fd->pTxMsg);
  free(fd->pRxMsg);
  free(fd);
  return 0;
}

//CAN中断服务函数
void CAN1_RX0_IRQHandler(void)
{
  Message m;
  CAN_hw_rd(CAN1, &m, 0);
  if (canRxInterruptISR) canRxInterruptISR(&m);
  // Release FIFO 0/1 output mailbox
  CAN1->RF0R = CAN_RF0R_RFOM0;
}

//CAN中断服务函数
void CAN2_RX0_IRQHandler(void)
{
  Message m;
  CAN_hw_rd(CAN2, &m, 0);
  if (canRxInterruptISR) canRxInterruptISR(&m);
  // Release FIFO 0/1 output mailbox
  CAN2->RF0R = CAN_RF0R_RFOM0;
}
