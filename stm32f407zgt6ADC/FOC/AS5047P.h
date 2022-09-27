/*
 * as5047p.h
 *
 *  Created on: Jun 15, 2021
 *      Author: Administrator
 */

#ifndef __AS5047P_H_
#define __AS5047P_H_

#include "main.h"

extern SPI_HandleTypeDef hspi2;
//extern UART_HandleTypeDef huart2;


#define SPI_CS_ENABLE  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_RESET )
#define SPI_CS_DISABLE HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_SET )

#define AS5047P_CPR 16384

//spi command frame 格式
#define AS5047_COMMAND_READ 0x4000


//Volatil Registers 的 addrees mapping
#define	NOP_AS5047P_VOL_REG_ADD 0x0000
#define ERRFL_AS5047P_VOL_REG_ADD 0x0001
#define PROG_AS5047P_VOL_REG_ADD 0x0003
#define DIAAGC_AS5047P_VOL_REG_ADD 0x3ffc
#define MAG_AS5047P_VOL_REG_ADD 0x3ffd
#define ANGLEUNC_AS5047P_VOL_REG_ADD 0x3ffe
#define ANGLECOM_AS5047P_VOL_REG_ADD 0x3fff


//non-volatile-registers 的 addrees mapping
#define ZPOSM_AS5047P_nVOL_REG_ADD 0x0016
#define ZPOSL_AS5047P_nVOL_REG_ADD 0x0017
#define SETTINGS1_AS5047P_nVOL_REG_ADD 0x0018
#define SETTINGS2_AS5047P_nVOL_REG_ADD 0x0019
#define RED_AS5047P_VOL_nREG_ADD 0x001a

void AS5047_Init(void);

float getAngle_as5047p(void);
uint16_t AS5047_ReadData(uint16_t addr);
uint16_t AS5047_WriteData(uint16_t addr,uint16_t data);
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata);
uint16_t Parity_bit_Calculate(uint16_t data_2_cal);

void AS5047_SetZeroPosition(void);
uint16_t AS5047_Get_ZeroPosition(void);

uint16_t ReadRegister(uint16_t addr);
float getVelocity_AS5047P(void);

#endif /* INC_AS5047P_H_ */
