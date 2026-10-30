/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           maze.c
** Last modified Date:  2007/09/24
** Last Version:        V1.0
** Description:         根据底层程序取得的迷宫信息，经过该智能算法控制电脑鼠的下一状态，并送往底层驱动程
**                      序执行。
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          Liao Maogang
** Created date:        2007/09/08
** Version:             V1.0
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**                                  在中断中stop
*********************************************************************************************************/


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Maze.h"
#include "Maze_Algo.h"
#include "stm32f10x.h"
#include "zlg7289.h"
#include "Mouse_Drive.h"
#include "Mouse_Nav.h"
#include "Mouse_Fan.h"
#include "Mouse_Led.h"
#include "stmflash.h"
#include "stdio.h"
/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/

static uint8 GucMouseTask = WAIT;                                 /*  状态机，初始状态为等待      */

uint8 Rece = 0;                                                  /*  USART3 收到的字节           */
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delay(uint32 uiD)
{
    volatile uint32 i;
    for (i = uiD; i; i--);
}

/*********************************************************************************************************
** Function name:       __ir_Get
** Descriptions:        读取E2PROM中的红外频率
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void RCC_Init(void)
{    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
}

// mode:JTAG,SWD模式设置；00，全使能，01使能SWD，10全关闭
void JTAG_Set(u8 mode)
{
    u32 temp;
    temp=mode;
    temp<<=25;
    RCC->APB2ENR|=1<<0;
    AFIO->MAPR&=0XF8FFFFFF;
    AFIO->MAPR|=temp;
}

void StartSave()
{
    Test_Write(START_SAVE_ADDRESS,GucXStart);
}

void wallsave(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock4[x][y] =GucMapBlock[x][y]; 
      }
   } 
  STMFLASH_Write(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,TUNE_FLASH_MAP_WORDS);

}


void wallget(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   STMFLASH_Read(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,TUNE_FLASH_MAP_WORDS);
   delay(TUNE_DELAY_FLASH_READ);
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock[x][y] =(uint8)GucMapBlock4[x][y]; 
      }
   }   
}

void StartGet(void)
{
  uint16 start = 0;
 
 // read_fm24LC16(&start,0x80,0xa1,1);
  STMFLASH_Read(START_SAVE_ADDRESS,(u16*)&start,1);
  if(start == TUNE_START_MARK)
  {
    GucXStart   = MAZETYPE - 1;                             /*  修改电脑鼠起点的横坐标      */
    GmcMouse.cX = MAZETYPE - 1; 
  }
}


void __ir_Get(void)
{    
      uint16    DIS1[10] = {0};
      STMFLASH_Read(IR_SAVE_ADDRESS,(u16 *)DIS1,10);
      delay(TUNE_DELAY_POST_ACTION);
      GusDistance_L_Near=DIS1[0];
      GusDistance_R_Near=DIS1[1];
      GusDistance_L_Mid=DIS1[2];
      GusDistance_R_Mid=DIS1[3]; 
      GusDistance_L_Far=DIS1[4];
      GusDistance_R_Far=DIS1[5];
      GusDistance_FL_Far=DIS1[6];
      GusDistance_FR_Far=DIS1[7];
      GusDistance_FL_Near=DIS1[8];
      GusDistance_FR_Near=DIS1[9];
}
void USART3_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);		   

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}	




void USART3_Config(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = TUNE_UART_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure); 
    USART_Cmd(USART3, ENABLE);
}

void USART3_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		   
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  	           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TUNE_UART_IRQ_PREEMPT; 	    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TUNE_UART_IRQ_SUB; 		    
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);		
    USART_Cmd(USART3, ENABLE);
}

void uart_sendchar(unsigned char ch)
{
  while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
  USART_SendData(USART3,ch);
}


int fputc(int ch,FILE *f)
{
  USART_SendData(USART3,(unsigned char)ch);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  return (ch);
}


void USART3_IRQHandler(void)
{
    int RX_status;
    RX_status = USART_GetFlagStatus(USART3, USART_FLAG_RXNE);
    if(RX_status == SET) 
    {
        //USART_SendData(USART3 , USART_ReceiveData(USART3));
        Rece=USART_ReceiveData(USART3);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
    }
}
void SendBlook()
{
    uint8 String[]="0123456789abcdef";
    //uint8 shi,ge;
    SendStr("GucMapBlock[");
    uart_sendchar(String[GmcMouse.cY]);
    SendStr("]= ");
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X08) >> 3]);
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X04) >> 2]);
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X02) >> 1]);
    uart_sendchar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X01]);
    SendStr("         ");
}
void Sendmap()
{
    uint8 String[]="0123456789abcdef";
  /*  if(map == 0)
       uart_sendchar(String[15]);
    else*/
    uart_sendchar(String[GmcMouse.cX]);
    SendStr("  ");
    uart_sendchar(String[GmcMouse.cY]);
    SendStr("  ");
    uart_sendchar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY]]);
    SendStr("\r\n");
}
void SendStr(const char *s)
{
  while(*s){    
  // USART_SendData(USART3, *s++);
    uart_sendchar(*s++);
  }
}
uint8 zijiaozheng0 = 0;                                         /*  陀螺零漂校准标志（SysTick 使用）*/

int main(void)
{
    uint8 ucTemp = 0;                                           /*  START 状态中坐标转换         */
    uint8 start  = 0;                                           /*  START 按键次数               */
#if TUNE_8DIR_ENABLE
    uint8 nav    = NAV_NO_PATH;                                 /*  8 向冲刺结果                 */
#endif

    SystemInit();
    RCC_Init();
    JTAG_Set(1);
    MouseInit();                                                /*  电脑鼠初始化                 */
    PIDInit();
    ZLG7289Init();
    delay(TUNE_DELAY_BOOT);
    //__ir_Get();                                               /*  需要时打开：从 Flash 载入红外阈值 */
    delay(TUNE_DELAY_BOOT);
    USART3_GPIO_Config();
    USART3_Config();
    USART3_NVIC_Config();
    floodwall();
    Fan_Off();
    Led_SetMode(LED_MODE_OFF);

    while (1) {
        switch (GucMouseTask) {                                 /*  状态机处理                   */
        case WAIT:
            sensorDebug();                                      /*  检查红外                     */
            delay(TUNE_DELAY_SENSOR_REFRESH);

            if (startCheck() == true) {
                start++;
            }
            if ((start == TUNE_START_PRESS_SEARCH) && GucGoHead) {
                start = 0;
                zijiaozheng0 = 1;
                Led_SetMode(LED_MODE_SEARCH);
                delay(TUNE_DELAY_SETTLE);
                GucMouseTask = START;
                delay(TUNE_DELAY_POST_ACTION);
                zijiaozheng0 = 1;
            }
            if (start == TUNE_START_PRESS_CALIB) {
                start = 0;
                zijiaozheng0 = 1;
                Led_SetMode(LED_MODE_SEARCH);
                delay(TUNE_DELAY_SETTLE);
                delay(TUNE_DELAY_POST_ACTION);
                Led_SetMode(LED_MODE_OFF);
                zijiaozheng0 = 1;
            }
            if ((start >= TUNE_START_PRESS_SPURT_MIN) &&
                (start < TUNE_START_PRESS_SPURT_MAX) && GucGoHead) {
                Fan_Off();
                Led_SetMode(LED_MODE_OFF);
                zijiaozheng0 = 1;
                start = 0;
                wallget();
                delay(TUNE_DELAY_SETTLE);
                StartGet();
                delay(TUNE_DELAY_SETTLE);
                delay(TUNE_DELAY_POST_ACTION);
                delay(TUNE_DELAY_POST_ACTION);
                delay(TUNE_DELAY_POST_ACTION);
                delay(TUNE_DELAY_POST_ACTION);
                GucMouseTask = SPURTL;
                delay(TUNE_DELAY_POST_ACTION);
            }
            break;

        case START:                                             /*  判断电脑鼠起点的横坐标       */
            Led_SetMode(LED_MODE_SEARCH);
            delay(TUNE_DELAY_SETTLE);
            mazeSearch();                                       /*  向前搜索                     */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {
                if (MAZETYPE == 16) {
                    GucXGoal0 = TUNE_GOAL_X0;
                    GucXGoal1 = TUNE_GOAL_X1;
                }
                GucXStart   = MAZETYPE - 1;
                GmcMouse.cX = MAZETYPE - 1;
                ucTemp = GmcMouse.cY;
                do {
                    GucMapBlock[MAZETYPE - 1][ucTemp]  = GucMapBlock[0][ucTemp];
                    GucMapBlock0[MAZETYPE - 1][ucTemp] = GucMapBlock0[0][ucTemp];
                    GucMapBlock0[0][ucTemp] = 0;
                    GucMapBlock1[MAZETYPE - 1][ucTemp] = GucMapBlock1[0][ucTemp];
                    if (ucTemp > 0) {
                        GucMapBlock1[MAZETYPE - 2][ucTemp - 1] = 0x1d;
                    }
                    GucMapBlock1[0][ucTemp + 1] = 0x17;
                    GucMapBlock1[1][ucTemp]     = 0x1f;
                    GucMapBlock[0][ucTemp]      = 0x10;
                    GucMapBlock[1][ucTemp]      = 0x10;
                } while (ucTemp--);
                GucMapBlock1[0][0]             = 0x13;
                GucMapBlock1[1][0]             = 0x1b;
                GucMapBlock1[MAZETYPE - 2][0]  = 0x19;
                GucMouseTask = MAZESEARCH;
            }
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {
                GucMouseTask = MAZESEARCH;
            }
            break;

        case MAZESEARCH:
            Led_SetMode(LED_MODE_SEARCH);
            centralMethodnew();
            goalwall();
            StartSave();
            wallsave();
            mouseTurnback();
            objectGoTo1(GucXStart, GucYStart);
            mouseTurnbackqi();
            delay(TUNE_DELAY_SETTLE);
            GucMouseTask = SPURTL;
            break;

        case SPURTL:                                            /*  一冲（普通冲刺 / 8 向冲刺）  */
            Led_SetMode(LED_MODE_SPRINT);
            Fan_On();
            StartGet();
            wallget();
            goalwall();

#if TUNE_8DIR_ENABLE
            nav = mouseSpurt_8_to_goal(GucXGoal0, GucXGoal1, GucYGoal0, GucYGoal1);
            if (nav == NAV_NO_PATH) {
                mouseSpurt_CC();                                /*  地图没有斜向通路，回退 4 向  */
            } else if (nav != NAV_OK) {
                Fan_Off();
                Led_SetMode(LED_MODE_OFF);
                mouseStop();                                    /*  运动超时/中止，停车待查       */
                GucMouseTask = WAIT;
                delay(TUNE_DELAY_SETTLE);
                break;
            }
#else
            mouseSpurt_CC();
#endif

            mouseTurnback();
            objectGoTo1(GucXStart, GucYStart);
            onestep3();
            mouseTurnbackqidian();
            Fan_Off();
            Led_SetMode(LED_MODE_OFF);
            GucMouseTask = WAIT;
            delay(TUNE_DELAY_SETTLE);
            break;

        default:
            break;
        }
    }
}

     
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
