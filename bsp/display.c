#include "display.h"
#include "types.h"

/*********************************************************************************************************
** Function name:       ZLG7289_Delay
** Descriptions:        延时N个微秒
** input parameters:    x: 延时时间
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ZLG7289_Delay(uint16 x)
{	
	x += x;
	while(x--);
}

/*********************************************************************************************************
** Function name:       ZLG7289_DIO_OUT
** Descriptions:        将ZLG728_DI0设置成输出模式。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ZLG7289_DIO_OUT(void)
{
	//DIO=1;
    GPIOB->CRL&=0XFF0FFFFF; 
    GPIOB->CRL|=0X00300000;    //PD12推挽输出 
   // GPIOD->ODR|=1<<12;          //PD12输出高	
}
/*********************************************************************************************************
** Function name:       ZLG7289_DIO_IN
** Descriptions:        将ZLG728_DI0设置成输入模式。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ZLG7289_DIO_IN(void)
{
    GPIOB->CRL&=0XFF0FFFFF;
    GPIOB->CRL|=0X00400000;    //PD12浮空输入	
}

/*********************************************************************************************************
** Function name:       __displaySPIWrite
** Descriptions:        向SPI 总线写入1 个字节的数据。
** input parameters:    cDat：要写入的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __displaySPIWrite (char cDat)
{
      char CT=8;
      ZLG7289_DIO_OUT();	 //DIO设置成输出模式
      do{
           if((cDat & 0x80) == 0x80)
            {
                 ZLG7289_DIO=1;	
            }
            else
            {
                 ZLG7289_DIO=0;
            }
            cDat <<= 1;
            ZLG7289_CLK=1;
            ZLG7289_Delay(50);
            ZLG7289_CLK=0;
            ZLG7289_Delay(50);
        }while(--CT!=0);
}
/*********************************************************************************************************
** Function name:       __displaySPIRead
** Descriptions:        从SPI 总线读取1 个字节的数据。
** input parameters:    无
** output parameters:   无
** Returned value:      读取到的数据
*********************************************************************************************************/
 

char __displaySPIRead (void)
{
    char cDat = 0;
    char cT   = 8;
    ZLG7289_DIO_IN();   
  
   do
	{
	    ZLG7289_CLK=1;
	    ZLG7289_Delay(50);
	    cDat<<=1;  
	    if(PBin(5)==1)  cDat++;
	    ZLG7289_CLK=0;
	    ZLG7289_Delay(50);

	} while ( --cT != 0 );
	return 	cDat;
} 

/*********************************************************************************************************
** Function name:       displayCmd
** Descriptions:        执行ZLG7289 纯指令。
** input parameters:    cCmd：命令字
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void displayCmd (char  cCmd)
{
    ZLG7289_CS=0;
    ZLG7289_Delay(200);
    __displaySPIWrite (cCmd);
    ZLG7289_CS=1;
    ZLG7289_Delay(50);	
} 
/*********************************************************************************************************
** Function name:       displayCmdDat
** Descriptions:        执行ZLG7289 带数据指令。
** input parameters:    cCmd：命令字
**                      cDat：数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void displayCmdDat (uint8  cCmd, char  cDat)
{
    ZLG7289_CS=0;
    ZLG7289_Delay(200);
    __displaySPIWrite (cCmd);
    ZLG7289_Delay(80);
    __displaySPIWrite (cDat);
    ZLG7289_CS=1;
    ZLG7289_Delay(50);
}
/*********************************************************************************************************
** Function name:       displayDownload
** Descriptions:        下载数据。
** input parameters:    ucMod=0： 下载数据且按方式0 译码
**                      ucMod=1： 下载数据且按方式1 译码
**                      ucMod=2： 下载数据但不译码
**                      cX：      数码管编号（横坐标），取值0～7
**                      cDp=0：   小数点不亮
**                      cDp=1：   小数点亮
**                      cDat：    要显示的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void displayDownload (u8  ucMod, char  cX, char  cDp, char  cDat)
{
    unsigned char ucModDat[3] = {0x80,0xC8,0x90};
    unsigned char ucD1;
    unsigned char ucD2;
    
    if (ucMod > 2) {
        ucMod = 2;
    }
    
    ucD1  = ucModDat[ucMod];
    cX   &= 0x07;
    ucD1 |= cX;
    ucD2  = cDat & 0x7F;
    
    if (cDp  == 1) {
        ucD2 |= 0x80;
    }
    displayCmdDat(ucD1, ucD2);
}
/*********************************************************************************************************
** Function name:       displayKey
** Descriptions:        执行ZLG7289 键盘命令。
** input parameters:    无
** output parameters:   无
** Returned value:      返回读到的按键值：0～63。如果返回0xFF 则表示没有键按下
*********************************************************************************************************/
char displayKey (void)
{
   char key;
   
   ZLG7289_CS=0;
   ZLG7289_Delay(200);
   __displaySPIWrite(0x15);
   ZLG7289_Delay(100);
   key=__displaySPIRead();
   ZLG7289_CS=1;
   ZLG7289_Delay(50);

   return key;
}
/*********************************************************************************************************
** Function name:       __displayKeyInit
** Descriptions:        对ZLG7289按键中断初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __displayKeyInit (void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //用于中断引脚：上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		  //配置2位高优先级（抢占优先级），2位子优先级（响应优先级）
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;             
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	  //引脚中断抢占优先等级为第二级。
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 	          //引脚中断子优先等级为第二级。
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);    
 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource3);  //选择中断引脚
    EXTI_InitStructure.EXTI_Line = EXTI_Line3 ;		
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//中断模式，非事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);    
    
}
/*********************************************************************************************************
** Function name:       ZLG7289Init
** Descriptions:        对ZLG7289进行初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ZLG7289Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5 |GPIO_Pin_4;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    ZLG7289_DIO=1;
    ZLG7289_CLK=0;	
    ZLG7289_CS=1;
    __displayKeyInit ();
    displayReset();	
}




