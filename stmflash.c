#include "stmflash.h"


volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;   //Flash操作状态变量

u16 STMFLASH_ReadHalfWord(u32 addr)
{
	return *(u16 *)addr;
}


#if STM32_FLASH_WREN
	
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	u16 i;
	
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
		WriteAddr+=2;
	}
}

#if STM32_FLASH_SIZE<256
	#define STM_SECTOR_SIZE 1024
#else
	#define STM_SECTOR_SIZE 2048
#endif

u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	u32 secpos;     //扇区地址
	u16 secoff;     //扇区内偏移地址（16位字计算）
	u16 secremain;   //扇区内剩余地址（16位字计算）
	
	u16 i;
	u32 offaddr;   //去掉0x08000000后的地址
	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr>=STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)
		return ;  //非法地址
	
	FLASH_Unlock();     //解锁
	
	offaddr=WriteAddr-STM32_FLASH_BASE;
	secpos=offaddr/STM_SECTOR_SIZE;
	secoff=(offaddr%STM_SECTOR_SIZE)/2;
	secremain=STM_SECTOR_SIZE/2-secoff;
	
	if(NumToWrite<=secremain)
		secremain=NumToWrite;
	
	while(1)
	{
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
		
		for(i=0;i<secremain;i++)
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)
				break;
		}
		
		if(i<secremain)
		{
				FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
				for(i=0;i<secremain;i++)//复制
				{
					STMFLASH_BUF[i+secoff]=pBuffer[i];	  
				}
				 STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
		}
		else 
				STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间 	
			
		if(NumToWrite==secremain)
				break;//写入结束了
		else
		{
				secpos++;				//扇区地址增1
				secoff=0;				//偏移位置为0 	 
				pBuffer+=secremain;  	//指针偏移
				WriteAddr+=secremain;	//写地址偏移	   
				NumToWrite-=secremain;	//字节(16位)数递减
				if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
				else secremain=NumToWrite;//下一个扇区可以写完了
		}				
	}	
	FLASH_Lock();
}

#endif


void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);
		
		ReadAddr+=2;
		
	}
}
	

void Test_Write(u32 WriteAddr,u16 WriteData)
{
	STMFLASH_Write(WriteAddr,&WriteData,1);
}



void STMFLASH_WriteOneWord(u32 WriteAddr,u32 WriteData)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	
	FLASHStatus=FLASH_ErasePage(STARTADDR);
	
	if(FLASHStatus)
	{
		FLASHStatus=FLASH_ProgramWord(STARTADDR+WriteAddr,WriteData);
	}
	FLASH_Lock();
}

/****************************************************************
*Name:		ReadFlashNBtye
*Function:	从内部Flash读取N字节数据
*Input:		ReadAddress：数据地址（偏移地址）ReadBuf：数据指针	ReadNum：读取字节数
*Output:	读取的字节数  
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		
****************************************************************/
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum) 
{
		int DataNum = 0;
		ReadAddress = (uint32_t)STARTADDR + ReadAddress; 
		while(DataNum < ReadNum) 
		{
			 *(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;
			 DataNum++;
		}
		return DataNum;
}
