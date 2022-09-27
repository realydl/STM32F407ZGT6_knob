/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS5047P.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int16_t adc_buff[ADC_NUM_MAX];    // ADC电压采集缓冲区
//extern int16_t adc_buff2[ADC_NUM_MAX];    // ADC电压采集缓冲区

//extern float adc_buff[ADC_NUM_MAX];    // ADC电压采集缓冲区
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern float adc_u;
extern float adc_v;

extern float adc_u2;
extern float adc_v2;

uint16_t IU,IV;

//extern __IO uint16_t ChannelPulse;


float targetId;
float targetIq;
float velocity_target;	//待设定目标速度
float position_target;	//待设定目标位置

int  pole_pairs	= 14;//电机极对数
int  pole_pairs2 = 14;


long sensor_direction;
long sensor_direction2;



float voltage_power_supply;
float voltage_limit;

float voltage_sensor_align;
float voltage_sensor_align2;

float velocity_limit;
float current_limit;


uint32_t upposition;
uint32_t upvelocity;
uint32_t upcurrentq;
uint32_t upcurrentd;


float now_angle;
float now_velocity;
float now_position;

float now_angle2;
float now_velocity2;
float now_position2;

float Iq_target;

extern float a,b,c;
PhaseCurrent_s current_test;
PhaseCurrent_s current_test2;

float ctt,stt,d_t,q_t;
float i_alpha_t, i_beta_t;

extern float sensor_offset;
extern float Electrical_Angele_Sensor_Offset;

float firstAngle;
float SecondAngle;

float AS5600Angle1;
float AS5600Angle2;

float AS5047_PAngle;
//extern float AS4057_PAnglel;
uint32_t Raw_AS5047p;
float ZeroAngle;

//双电机互控
float abs_angle1;
float abs_angle2;

//模拟旋钮
float knob_angle;
float knob_velocity;
float knob_torque;
float now_Encoder;
float last_Encoder;
uint32_t knob_step;
uint32_t Planning_Number;
float Now_Planing_Number;
float shaftAngle_as5047p;
float getAngle5047p;
DQCurrent_s knob_current;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	voltage_limit = 6;          //V，最大值需小于 供电电压/sqrt(3) 12/1.732=6.9 24/sqrt(3)=13.8568
	voltage_limit = 13.86;          //V，最大值需小于 供电电压/sqrt(3) 12/1.732=6.9 24/sqrt(3)=13.8568
	voltage_power_supply = 24; 	//电源电压
	
	voltage_sensor_align = 3;   //V alignSensor() use it，大功率电机设置的值小一点比如0.5-1，小电机设置的大一点比如2-3
	voltage_sensor_align2 = 3;
	
	current_limit = 50; 				//电流限制，速度模式和位置模式起作用
	
	velocity_limit= 20;         	//位置模式速度限制
	
	velocity_target = 3;        //待设定目标速度 3 rad/s
	position_target = 4.5;				//待设定目标位置 3 rad
	
	Iq_target = 0; 
	targetId = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	//上位机初始化
//	protocol_init();


	
	__HAL_TIM_SetCounter(&htim4, 0);
  __HAL_TIM_ENABLE(&htim4);
	
	IIC_Init();
	IIC2_Init();
	
		
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_buff,ADC_NUM_MAX);//ADC采样点数
	
	
	LPF_init();
	LPF_init2();
	
	PID_init();
	PID_init2();
	
	InlineCurrentSense(0.01,50);    //SimpleMotor//采样电阻阻值，运放倍数
	InlineCurrentSense2(0.01,50);
	
		//AS5047P初始化
#if 1
		HAL_Delay(100);
		AS5047_Init();
		HAL_Delay(100);
		ZeroAngle = AS5047_Get_ZeroPosition() * 360.0 / 0x3FFF;
#endif
	
	Motor_init();
	my_delay_ms(10);
	Motor_initFOC(0,UNKNOWN);
	my_delay_ms(500);
	Current_calibrateOffsets();//电流偏置测量(空载情况下的电流)

	Motor_init2();
	my_delay_ms(10);
	Motor_initFOC2(0,UNKNOWN);
	my_delay_ms(500);
	Current_calibrateOffsets2();//电流偏置测量(空载情况下的电流)
	
//	my_delay_ms(10);
	/*****************************************************************/
	//Uq90度
//	//setPhaseVoltage(voltage_sensor_align, 0, _3PI_2);
//	
//	setPhaseVoltage(voltage_sensor_align, 0, _PI_2);
//	
//	firstAngle = shaftAngle();//理论上应该是90度
//	
////	setPhaseVoltage(0, voltage_sensor_align, 0);
////	my_delay_ms(500);
//	sensor_offset = shaftAngle();// shaft angle
//	sensor_offset = sensor_offset - _PI_2/7;
	
	/*****************************************************************/


		
//	setPhaseVoltage(0, 0, 0);
#if 0
	CALIBRATION_start();

	CALIBRATION_loop();
	my_delay_ms(1000);	
#endif

//上位机启动
#if 0
	set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);                // 同步上位机的启动按钮状态
	set_computer_value(SEND_STOP_CMD, CURVES_CH2, NULL, 0);                // 同步上位机的启动按钮状态
	set_computer_value(SEND_STOP_CMD, CURVES_CH3, NULL, 0);                // 同步上位机的启动按钮状态
	set_computer_value(SEND_STOP_CMD, CURVES_CH4, NULL, 0);                // 同步上位机的启动按钮状态
	
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &upcurrentq, 1);     // 给通道 3发送目标值
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH2, &upcurrentd, 1);     // 给通道 3发送目标值
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH3, &velocity_target, 1);     // 给通道 3发送目标值
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH4, &position_target, 1);     // 给通道 3发送目标值
	
	set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);               // 同步上位机的启动按钮状态
	set_computer_value(SEND_START_CMD, CURVES_CH2, NULL, 0);               // 同步上位机的启动按钮状态
	set_computer_value(SEND_START_CMD, CURVES_CH3, NULL, 0);               // 同步上位机的启动按钮状态
	set_computer_value(SEND_START_CMD, CURVES_CH4, NULL, 0);               // 同步上位机的启动按钮状态
#endif	
//	setPhaseVoltage(voltage_sensor_align,0,_PI_2);
	
//	setPhaseVoltage(0,voltage_sensor_align,0);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
  while (1)
  {

		#if 0
			receiving_process();
		#endif
		
		#if 0
		//AS5600编码器测试
			abs_angle1 = ABS_Angle();
			abs_angle2 = ABS_Angle2();
			AS5600Angle1 = AS5600_ReadRawAngle()/(float)4096*(float)360;
			AS5600Angle2 = AS5600_ReadRawAngle2()/(float)4096*(float)360;
			shaft_angle = shaftAngle();// shaft angle
			electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
			shaft_angle2 = shaftAngle2();
			electrical_angle2 = electricalAngle2();
	//		printf("%f\r\n",AS5600_ReadRawAngle()/4096*360);
		#endif
		
		#if 0
//		shaft_angle = shaftAngle();// shaft angle
		
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		shaft_angle = shaftAngle();// shaft angle
		now_velocity = getVelocity();
		#endif
//		printf("%f\r\n",PIDoperator(&PID_velocity,(10 - 5)));
		
		//电机1校准
		#if 0
		//Ud基准
		setPhaseVoltage(0,voltage_sensor_align,(_2PI * 1.f/ 12.0f));
		
		current_test = getPhaseCurrents();
		
		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
//		i_alpha_t = current_test.a;  
//		i_beta_t = _1_SQRT3 * current_test.a + _2_SQRT3 * current_test.b;//a + b + c = 0
//		
//		ctt = arm_cos_f32(electrical_angle);
//		stt = arm_sin_f32(electrical_angle);
//		
//		//simplefoc的park变换
//		d_t = i_alpha_t * ctt + i_beta_t * stt;
//		q_t = i_beta_t * ctt - i_alpha_t * stt;
		
		#endif
		
		//电机2校准
		#if 0
		//电机2 Ud基准
		setPhaseVoltage2(0,voltage_sensor_align2,(_2PI * 1.f/ 12.0f));
		
		current_test2 = getPhaseCurrents2();
		
		shaft_angle2 = shaftAngle2();// shaft angle
		electrical_angle2 = electricalAngle2();// electrical angle - need shaftAngle to be called first
		
//		i_alpha_t = current_test2.a;  
//		i_beta_t = _1_SQRT3 * current_test2.a + _2_SQRT3 * current_test2.b;//a + b + c = 0
//		
//		ctt = arm_cos_f32(electrical_angle2);
//		stt = arm_sin_f32(electrical_angle2);
//		
//		//simplefoc的park变换
//		d_t = i_alpha_t * ctt + i_beta_t * stt;
//		q_t = i_beta_t * ctt - i_alpha_t * stt;
		
		#endif
//		printf( "angle : %.04f\r\n", ReadAngle() );

//		HAL_Delay(1000);
		
		//电机1对相
		#if 0
//		//a
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0.1*(2100-1));
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
		//b
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0.1*(2100-1));
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
		//c
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0.1*(2100-1));
		
		
		current_test = getPhaseCurrents();
		#endif
		
		//电机2对相
		#if 0
		//a
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0.1*(2100-1));
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		//b
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0.1*(2100-1));
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		//c
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0.1*ChannelPulse);

		current_test = getPhaseCurrents2();

		#endif
//		setPhaseVoltage(0, 1, _2PI * 1.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 3.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 5.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 7.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 9.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 11.0f/ 12.0f);
		
//		shaft_angle = shaftAngle();// shaft angle
//		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
//		current = getFOCCurrents(electrical_angle);
		
		
//		move(Iq_target);
//Id
#if 0
		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
		// read dq currents
		current = getFOCCurrents(electrical_angle);
		// filter values
//		current.q = LPFoperator(&LPF_current_q,current.q);
		current.d = LPFoperator(&LPF_current_d,current.d);
		
		voltage.d = PIDoperator(&PID_current_d, (targetId - current.d));
		setPhaseVoltage(0, voltage.d, electrical_angle);
#endif		

//Iq
#if 0
//		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
		// read dq currents
		current = getFOCCurrents(electrical_angle);
		// filter values
		current.q = LPFoperator(&LPF_current_q,current.q);
//		current.d = LPFoperator(&LPF_current_d,current.d);
		
//		voltage.d = PIDoperator(&PID_current_d, (targetId - current.d));
		voltage.q = PIDoperator(&PID_current_q, (targetIq - current.q));
		setPhaseVoltage(voltage.q , 0, electrical_angle);
#endif
//clark park 变换验证

#if 0
//	DQCurrent_s getFOCCurrents(float angle_el);
	
	float angle_t = _PI/6;
	float ctt,stt,d_t,q_t;
	float i_alpha_t, i_beta_t;
	float Ia_t = 0.5;
	float	Ib_t = -0.3;
	
	i_alpha_t = Ia_t;  
	i_beta_t = _1_SQRT3 * Ia_t + _2_SQRT3 * Ib_t;//a + b + c = 0
	
	ctt = arm_cos_f32(angle_t);
	stt = arm_sin_f32(angle_t);
	
	//simplefoc的park变换
	d_t = i_alpha_t * ctt + i_beta_t * stt;
	q_t = i_beta_t * ctt - i_alpha_t * stt;
	
	printf("d_t:%f\r\n",d_t);
	printf("q_t:%f\r\n",q_t);
#endif

//电流模式
#if 0
		move(0);
		loopFOCtest();
		now_velocity = shaftVelocity();//测速
		now_position = getAngle();
#endif
//速度模式
#if 0
//	//AS5600
//		now_velocity = move_velocity(velocity_target);
//		loopFOCtest();
	//AS5047P
		now_velocity = move_velocity(velocity_target);
	//AS5047P测速
//	shaft_velocity = shaftVelocity_AS5047P();
	//AS5047P测角度
//	shaftAngle_as5047p = shaftAngle_AS5047P();// shaft angle
//	getAngle5047p = getAngle_as5047p();
//	knob_angle = ((float)(ReadRegister(0xFFFF)&0x3fff)*360)/16384;
		Knob_loop();
//向上位机打印数据
//		upposition = getAngle();
		
//		upvelocity = now_velocity * 1000;
//		upcurrentq = current.q * 1000 + 1000;//扩大1000，偏置1000
//		upcurrentd = current.d * 1000 + 1000;//扩大1000,偏置1000
		
//		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &upcurrentq, 1);     // 给通道1发Iq
//		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &upcurrentd, 1);     // 给通道2发Id
//		set_computer_value(SEND_FACT_CMD, CURVES_CH3, &upvelocity, 1);     // 给通道3发送速度值
////		set_computer_value(SEND_FACT_CMD, CURVES_CH4, &upposition, 1);     // 给通道4发送位置值
#endif
//M2速度模式
#if 0
		now_velocity2 = move_velocity2(velocity_target);
		loopFOCtest_second();
#endif

//双电机速度模式
#if 0
		now_velocity = move_velocity(velocity_target);
		loopFOCtest();
		
		now_velocity2 = move_velocity2(velocity_target);
		loopFOCtest_second();
#endif


//位置环模式
#if 1
	//AS5600
//		now_position = move_position(position_target);
//		loopFOCtest();//电流环PID
	//AS5047P
		now_position = move_position(position_target);
		Knob_loop();

//		loopFOCtest();//电流环PID

//		now_velocity = shaftVelocity();
		
//		upvelocity = now_velocity * 1000;//扩大1000
//		upcurrentq = current.q * 1000 + 1000;//扩大1000，偏置1000
//		upcurrentd = current.d * 1000 + 1000;//扩大1000,偏置1000
#endif

//		now_velocity= move_velocity(velocity_target);
//		loopFOCtest();

//	float A,B;
//	float d =_2PI/2048;
	
//双电机力矩互控
#if 0
//		move(0);
//		move2(0);
//		shaft_angle = shaftAngle();// shaft angle
//		shaft_angle2 = shaftAngle2();// shaft angle
		
//		abs_angle1 = ABS_Angle();
//		abs_angle2 = ABS_Angle2();
		
		move_torque2();
		move_torque1();
		
		loopFOCtest_second();
		loopFOCtest();
		
//		test();
		
//		if((adc_u>0.5)&&(adc_v>0.5)&&(adc_u2>0.5)&&(adc_v2>0.5)){
//			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);
//		}else{
//			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_SET);
//		}
		
//	shaft_angle2 = shaftAngle2();// shaft angle
//	electrical_angle2 = electricalAngle2();// electrical angle - need shaftAngle to be called first
	
//	// read dq currents
//	current2 = getFOCCurrents2(electrical_angle2);

////		
//	voltage2.q = PIDoperator(&PID_current_q2,(current_sp - current2.q)); 
//	voltage2.d = PIDoperator(&PID_current_d2, (targetId - current2.d));

//			
//	// set the phase voltage - FOC heart function :)
//  setPhaseVoltage2(voltage2.q, voltage2.d, electrical_angle2);



//		getPhaseCurrents();
//		getPhaseCurrents2();

//		current_test = getPhaseCurrents();
//		current_test2 = getPhaseCurrents2();

		
//		now_velocity = shaftVelocity();//测速
//		now_position = getAngle();
		
//		now_velocity2 = shaftVelocity2();//测速
//		now_position2 = getAngle2();
#endif
	
	//电机旋钮功能
	/*
		1.弹性回正(位置模式)
			电机以一个固定码值为中心，偏离中心码值会产生回复力
			使电机能回复到中心码值位置
			
		2.分段旋钮
			将电机旋转的一周切分成n个点位，在点位上电机不产生主动回复力，
			偏离点位，电机会受到回复力，偏离点位越远，受到的回复力越大
			
		3.阻尼回正
		4.阻尼旋钮
		5.失重转动
		
		6.左右限位
			电机在一定的位置内能自由转动，超出位置限制会产生回复力
			
	*/
	
	//2.分段旋钮
	 //多段旋钮
  /*
    四段式旋钮
    切分点0,90,180,270
    二次分段: 1）0-45,45-90
            2）90-135,135-180
            3）180-225,225-270
            4）270-315,315-360（0）
  */
	//测试AS5047p编码器
	#if 0
	shaftAngle_as5047p = shaftAngle_AS5047P();// shaft angle
	getAngle5047p = getAngle_as5047p();
	knob_angle = ((float)(ReadRegister(0xFFFF)&0x3fff)*360)/16384;
	#endif
	#if 0
	//knob_angle = (float)AS5600_ReadRawAngle()/4096*360.0;//AS5600
	//AS5047P
	knob_angle = ((float)(ReadRegister(0xFFFF)&0x3fff)*360)/16384;
  knob_step = (uint32_t)knob_angle/45 + 1;
  switch(knob_step){
    case 1://0-45
      if((knob_angle > 0.1)&&(knob_angle < 44.9))
        knob_torque = 0.2;//往大走
          break;
    case 2://45-90
      if((knob_angle > 45.1)&&(knob_angle < 89.9))
        knob_torque = -0.2;//往小走
      else
        knob_torque = 0;
          break;
    case 3://90-135
      if((knob_angle > 90.1)&&(knob_angle < 134.9))
        knob_torque = 0.2;//往大走
      else
        knob_torque = 0;
          break;
    case 4://135-180
      if((knob_angle > 135.1)&&(knob_angle < 179.9))
        knob_torque = -0.2;//往小走
      else
        knob_torque = 0;
          break;
    case 5://180-225
      if((knob_angle > 180.1)&&(knob_angle < 224.9))
        knob_torque = 0.2;//往大走
      else
        knob_torque = 0;
          break;    
    case 6://225-270
      if((knob_angle > 225.1)&&(knob_angle < 269.9))
        knob_torque = -0.2;//往小走
      else
        knob_torque = 0;
          break;
    case 7://270-315
      if((knob_angle > 270.1)&&(knob_angle < 314.9))
        knob_torque = 0.2;//往大走
      else
        knob_torque = 0;
          break;
    case 8://315-360（0）
      if((knob_angle > 315.1)&&(knob_angle < 359.9))
        knob_torque = -0.2;//往小走
      else
        knob_torque = 0;
          break;
    default:
        knob_torque = 0;
          break;
   }
	move(knob_torque);
	//AS5600
	//loopFOCtest();
	
	 //AS5047P
	Knob_loop();
	#endif
	
//	#if 0
//		knob_angle = (float)AS5600_ReadRawAngle()/4096*360.0;
//		Multistage_knob_move(knob_angle);
//		loopFOCtest();
//	#endif
		
	//3.阻尼旋钮
	
	//5.失重转动
	#if 0
//		now_Encoder = shaftAngle();//AS5600
		now_Encoder = shaftAngle_AS5047P();
		knob_torque = -0.01 * (now_Encoder - last_Encoder);	
//		if(knob_torque > 0.5)
//			knob_torque = 0.5;
//		else if(knob_torque < -0.5){
//			knob_torque = -0.5;
//		}
		move(knob_torque);
//		loopFOCtest();
		Knob_loop();
		my_delay_ms(10);
		last_Encoder = now_Encoder;
	#endif
	//6.左右限位，力矩模式，超出边界受到反向力矩
	#if 0
//		knob_angle = (float)AS5600_ReadRawAngle()/4096*360.0;
    knob_angle = ((float)(ReadRegister(0xFFFF)&0x3fff)*360)/16384;
		//180度自由旋转范围，超出范围受到反向的力矩
    if((knob_angle > 0)&&(knob_angle < 180)){
      knob_torque = 0;
    }else if((knob_angle >= 180)&&(knob_angle < 270)){
      knob_torque = 0.2;//顺时针，往小走
    }else{//angle0>=270 angle0<=360
      knob_torque = -0.2;//逆时针，往大走
    }
		move(knob_torque);
		Knob_loop();
//		loopFOCtest();
	#endif
		
	//6.左右限位，力矩模式，
	//设计超过边界缓慢递减的Iq,目前尚存在bug
	#if 0
		
//		move(0.01);
//		loopFOCtest();
//		knob_angle = (float)AS5600_ReadRawAngle()/4096*360.0;
		knob_angle = (float)AS5600_ReadRawAngle()/4096*360.0;
		knob_current = getFOCCurrents(electricalAngle());
		knob_torque = knob_current.q;
		if((knob_angle > 0.0)&&(knob_angle < 180.0)){//正常位置
			if(knob_torque > 0.005){
				Planning_Number = (uint32_t)fabs(knob_angle)/0.002;
				while((Planning_Number == 0)&&(knob_torque <= 0.002)){
						Planning_Number--;
						knob_torque = Planning_Number * 0.002;	
						knob_current = getFOCCurrents(electricalAngle());
						knob_torque = knob_current.q;
					}
			}else if(knob_torque < -0.005){
					Planning_Number = (uint32_t)fabs(knob_angle)/0.002;
				while((Planning_Number == 0)&&(knob_torque <= 0.002)){
					Planning_Number--;
					knob_torque = Planning_Number * -0.002;	
					knob_current = getFOCCurrents(electricalAngle());
					knob_torque = knob_current.q;
				}
			}else{
					knob_torque = 0;
			}
//			move(knob_torque);
//			loopFOCtest();
		}else if((knob_angle >= 180.0)&&(knob_angle < 270.0)){//180
				knob_torque = -0.008;
//			move_position(180.0);//位置模式
//			current_sp = (float)-0.2*(knob_angle-180)/90.0;
//			move(-0.01);
//			loopFOCtest();
		}else{
				knob_torque = 0.008;
//			move(0.01);
//			move_position(0.0);//位置模式
//			loopFOCtest();
		}
		
		move(knob_torque);
		loopFOCtest();
	#endif 
	
	#if 0
	//马鞍波测试
		for(uint32_t i = 0; i<5000;i++){
			shaft_angle = shaftAngle();// shaft angle
			electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
			setPhaseVoltage(3*arm_sin_f32(((float)i/5000.0f) * _2PI),3*arm_cos_f32(((float)i/5000.0f) * _2PI),electrical_angle);
	//		my_delay_ms(10);
			set_computer_value(SEND_FACT_CMD, CURVES_CH1, &a, 1);     
			set_computer_value(SEND_FACT_CMD, CURVES_CH2, &b, 1);  
			set_computer_value(SEND_FACT_CMD, CURVES_CH3, &c, 1);     	
		}
	#endif

	//发送通道值
	#if 0
			upposition = now_position * 1000;//扩大1000
			upvelocity = now_velocity * 1000;//扩大1000
			upcurrentq = current.q * 1000 + 1000;//扩大1000，偏置1000
			upcurrentd = current.d * 1000 + 1000;//扩大1000,偏置1000
			set_computer_value(SEND_FACT_CMD, CURVES_CH1, &upcurrentq, 1);     // 给通道1发Iq
			set_computer_value(SEND_FACT_CMD, CURVES_CH2, &upcurrentd, 1);     // 给通道2发Id
			set_computer_value(SEND_FACT_CMD, CURVES_CH3, &upvelocity, 1);     // 给通道3发送速度值
			set_computer_value(SEND_FACT_CMD, CURVES_CH4, &upposition, 1);     // 给通道4发送位置值
	#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//AS5047P
		#if 0
		AS5047_PAngle = ((float)(ReadRegister(0xFFFF)&0x3fff)*360)/16384;
//		printf("%f\r\n",AS5047_PAngle);
		my_delay_ms(20);
		#endif

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
