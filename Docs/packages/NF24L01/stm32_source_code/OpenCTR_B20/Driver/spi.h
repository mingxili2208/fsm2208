#ifndef __SPI_H
#define __SPI_H
#include "ax_sys.h"


void SPI2_Init(void);           //Initialize SPI port
void SPI2_SetSpeed(u8 SpeedSet); //Set SPI speed
u8 SPI2_ReadWriteByte(u8 TxData);//SPI bus read and write a byte	

#endif

