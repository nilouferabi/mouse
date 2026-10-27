#ifndef __STMFLASH_H
#define __STMFLASH_H


#include "stm32f10x.h"

#define IR_SAVE_ADDRESS 0x08010000
#define MAPE_SAVE_ADDRESS 0x08018800
#define START_SAVE_ADDRESS 0x08019000

#define STM32_FLASH_WREN 1

#define STM32_FLASH_SIZE  128    //所选的STM32的FLASH的容量大小为128K
#define STM32_FLASH_BASE  0x08000000    //stm32起始地址


#define  STARTADDR  0x08010000 

u16 STMFLASH_ReadHalfWord(u32 addr);
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);
void Test_Write(u32 WriteAddr,u16 WriteData);
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);
void STMFLASH_WriteOneWord(u32 WriteAddr,u32 WriteData);
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum) ;




#endif
