/****************************************Copyright (c)****************************************************
** File Name:           serial_debug.c
** Description:         USART3 调试串口与地图发送
*********************************************************************************************************/

#include "serial_debug.h"
#include "maze.h"
#include "mouse_config.h"
#include "stm32f10x.h"
#include <stdio.h>

uint8 Rece = 0;

static void Serial_GpioInit(void)
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

static void Serial_PortInit(void)
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



static void Serial_NvicInit(void)
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



void Serial_SendChar(uint8 ch)
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


void Serial_SendBlock(void)
{
    uint8 String[]="0123456789abcdef";
    //uint8 shi,ge;
    Serial_SendStr("GucMapBlock[");
    Serial_SendChar(String[GmcMouse.cY]);
    Serial_SendStr("]= ");
    Serial_SendChar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X08) >> 3]);
    Serial_SendChar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X04) >> 2]);
    Serial_SendChar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X02) >> 1]);
    Serial_SendChar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X01]);
    Serial_SendStr("         ");
}


void Serial_SendMap(void)
{
    uint8 String[]="0123456789abcdef";
  /*  if(map == 0)
       Serial_SendChar(String[15]);
    else*/
    Serial_SendChar(String[GmcMouse.cX]);
    Serial_SendStr("  ");
    Serial_SendChar(String[GmcMouse.cY]);
    Serial_SendStr("  ");
    Serial_SendChar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY]]);
    Serial_SendStr("\r\n");
}


void Serial_SendStr(const char *s)
{
  while(*s){    
  // USART_SendData(USART3, *s++);
    Serial_SendChar(*s++);
  }
}


void Serial_Init(void)
{
    Serial_GpioInit();
    Serial_PortInit();
    Serial_NvicInit();
}

