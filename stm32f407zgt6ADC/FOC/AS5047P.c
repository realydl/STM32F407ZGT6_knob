/*
 * as5047p.c
 *
 *  Created on: Jun 15, 2021
 *      Author: Administrator
 */

#include "AS5047P.h"

float full_rotation_offset_as5047p;
float angle_data_prev_as5047p;


uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value=0;
	//自低位开始统计奇偶
	while(data_2_cal != 0)
	{
		//(((⑤^④)^③)^②)^①——表达式1假设⑤和④不同，则⑤^④运算结果为1，表示有一个1。表达式1可以化成：((1^③)^②)^①——表达式2
		parity_bit_value ^= data_2_cal;
		data_2_cal >>=1;
	}
	return (parity_bit_value & 0x1);
}


uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata)
{
	SPI_CS_ENABLE;
	uint16_t rxdata;
	//waring不用管,传进去的是地址数据不会被截断，HAL库根据配置，按16字节的数据格式将16字节的发送出去
	if(HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)&_txdata,(uint8_t *)&rxdata,1,100) !=HAL_OK)
		rxdata=0;
	SPI_CS_DISABLE;
	return rxdata;
}


uint16_t AS5047_WriteData(uint16_t addr,uint16_t data)
{
	//发送地址指令
	// & 0x3fff 得到 13:0位 data数据 进行奇偶校验位计算
	if(Parity_bit_Calculate(addr & 0x3fff ) == 1 )
	addr |= 0x8000;  //将15bit 置1  偶校验
	SPI_CS_ENABLE;
	SPI_ReadWrite_OneByte(addr);
	SPI_CS_DISABLE;

	//发送数据指令
	if(Parity_bit_Calculate(data &0x3fff) ==1)
		data |=0x8000;

	uint16_t feedback;
	SPI_CS_ENABLE;
	feedback = SPI_ReadWrite_OneByte(data);
	SPI_CS_DISABLE;

	return feedback;
}


//uint16_t AS5047_ReadData(uint16_t addr)
//{
//	uint16_t data;
//	if(Parity_bit_Calculate(addr) ==0 )
//		addr |=0x8000; //1000 0000 0000 0000
//	addr |= AS5047_COMMAND_READ; //0100 0000 0000 0000
//  SPI_ReadWrite_OneByte(addr);
//	data=SPI_ReadWrite_OneByte(NOP_AS5047P_VOL_REG_ADD);  //ANGLECOM_AS5047P_VOL_REG_ADD=11 1111 1111
//	data &=0x3fff;

////	SPI_ReadWrite_OneByte(devidx,addr);
////	data = SPI_ReadWrite_OneByte(devidx,addr);
//	//此处可以做奇偶校验判断是否接收到正确数据，但是也可以不做，直接去掉15，14bit
//	//data &= 0x3fff;
//	return data;
//}

uint16_t EF;
uint16_t data;

uint16_t AS5047_ReadData(uint16_t addr)
{
//	if(Parity_bit_Calculate(addr) ==0 ) 
//	addr |=0x8000; //1000 0000 0000 0000

	addr |= AS5047_COMMAND_READ; //0100 0000 0000 0000
	SPI_ReadWrite_OneByte(addr);
	data = SPI_ReadWrite_OneByte(NOP_AS5047P_VOL_REG_ADD);  //ANGLECOM_AS5047P_VOL_REG_ADD=11 1111 1111
	EF = data & 0x4000;
	if(EF == 0x4000){
		data &= 0x3fff;
	}
	return data;
}


void AS5047_Init(void)
{
  //设置ABI模式，输出分辨1024.
    AS5047_WriteData(SETTINGS1_AS5047P_nVOL_REG_ADD,5); //0000 0101
    AS5047_WriteData(SETTINGS2_AS5047P_nVOL_REG_ADD,0);
}


void AS5047_SetZeroPosition(void)
{
	uint16_t DIAAGC=AS5047_ReadData(DIAAGC_AS5047P_VOL_REG_ADD);
	//获取当前角度
	uint16_t ANGLEUNC=AS5047_ReadData(ANGLEUNC_AS5047P_VOL_REG_ADD);
	//ANGLEUNC是13:0 14个有效数字，右移6获取高8位。
	AS5047_WriteData(ZPOSM_AS5047P_nVOL_REG_ADD,(ANGLEUNC >>6) & 0x00ff);
	//得到低6位 & 11 1111
	AS5047_WriteData(ZPOSL_AS5047P_nVOL_REG_ADD, ANGLEUNC  & 0x003f);
}


uint16_t AS5047_Get_ZeroPosition(void)
{
	uint16_t ZPOSM=AS5047_ReadData(ZPOSM_AS5047P_nVOL_REG_ADD);
	uint16_t ZPOSL=AS5047_ReadData(ZPOSL_AS5047P_nVOL_REG_ADD);
	//将高八位和低6位拼起来
	return ( ZPOSM<<6 ) & (ZPOSL & 0x003f ) ;
}

uint16_t ReadRegister(uint16_t addr)
{
	uint16_t back;
	
	SPI_CS_ENABLE;
	SPI_ReadWrite_OneByte(addr);
	SPI_CS_DISABLE;
//	delay_s(20);  //1us
	my_delay_us(5);
	SPI_CS_ENABLE;
	back = SPI_ReadWrite_OneByte(0);
	SPI_CS_DISABLE;
//	delay_s(20);  //1us
	my_delay_us(2);
	return back;
}


//float angle_data_as5047p,d_angle;
long angle_data_as5047p,d_angle;

float getAngle_as5047p(void)//AS5047P
{
	cpr = AS5047P_CPR;
	
//	long angle_data_as5047p,d_angle;
	
	angle_data_as5047p = (long)(ReadRegister(0xFFFF)&0x3fff);//原角度
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data_as5047p - angle_data_prev_as5047p;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) >= 0.8*cpr ) 
		full_rotation_offset_as5047p += (d_angle > 0) ? -_2PI : _2PI; 

	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev_as5047p = angle_data_as5047p;
	// return the full angle 
	
	// (number of full rotations)*2PI + current sensor angle 
	return (full_rotation_offset_as5047p + ( (float)angle_data_as5047p / cpr) * _2PI);
}

unsigned long velocity_calc_timestamp_AS5047P;

// Shaft velocity calculation
float getVelocity_AS5047P(void)
{
	float Ts_V;
	unsigned long now_us;
	float vel;
	float angle_now;
	// calculate sample time
//	now_us = SysTick->VAL; //_micros();
//	if(now_us < velocity_calc_timestamp)
//		Ts = (float)(velocity_calc_timestamp - now_us)/6*1e-9f;
//	else
//		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/6*1e-9f;
//	// quick fix for strange cases (micros overflow)
//	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;

	now_us = __HAL_TIM_GetCounter(&htim4);
	if(now_us < velocity_calc_timestamp_AS5047P)
		Ts_V = (float)(velocity_calc_timestamp_AS5047P - now_us)*1e-6f;
	else
		Ts_V = (float)(0x0000FFFF - now_us + velocity_calc_timestamp_AS5047P)*1e-6f;
	if(Ts_V == 0 || Ts_V > 0.5f) Ts_V = 1e-3f;
	
	// current angle
//	angle_now = getAngle();
	angle_now = getAngle_as5047p();
	// velocity calculation
	vel = (angle_now - angle_prev)/Ts_V;

	// save variables for future pass
	angle_prev = angle_now;
	velocity_calc_timestamp_AS5047P = now_us;
	return vel;
}
