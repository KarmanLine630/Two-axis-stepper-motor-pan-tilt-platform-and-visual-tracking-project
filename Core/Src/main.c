/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "pid.h"
#include "keyboard.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define base 100000
#define err_angle 2
#define pulse_width 5
#define compsation_constant 0.01        
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
uint8_t rx_massage[50];
int x_point=120;
int y_point=120;
int compensation_value,compensation_Z;
int statue=0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&huart2)
	{
		x_point=0;
		y_point=0;
		if(rx_massage[0]=='P' && rx_massage[1]=='C' && rx_massage[2]=='E' && rx_massage[3]=='N')
		{
			x_point=(rx_massage[5]-'0')*100+(rx_massage[6]-'0')*10+(rx_massage[7]-'0')*1;
			y_point=(rx_massage[9]-'0')*100+(rx_massage[10]-'0')*10+(rx_massage[11]-'0')*1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,rx_massage,100);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_9)
	{
		statue=1;
	}
}

void speed_step_motor1(int speed_in,int init_num)
{
	int speed;
	if(speed_in>0)
	{
		TIM2->CCR1=pulse_width;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
		speed=init_num/speed_in;
		if(speed<500){TIM2->ARR=200;}
		else					{TIM2->ARR=speed;}
	}
	else if(speed_in<0)
	{
		TIM3->CCR1=pulse_width;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
		speed_in=-speed_in;
		speed=init_num/speed_in;
		if(speed<500){TIM2->ARR=200;}
		else 					{TIM2->ARR=speed;}
	}
	else
	{
		TIM2->CCR1=0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
		TIM2->ARR=65535;
	}
}
PID yaw_control;
PID visual_control;
PID visual_control_2;
void speed_step_motor2(int speed_in,int init_num)
{
	int speed;
	if(speed_in>0)
	{
		TIM3->CCR1=pulse_width;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
		speed=init_num/speed_in;
		if(speed<500){TIM3->ARR=200;}
		else					{TIM3->ARR=speed;}
	}
	else if(speed_in<0)
	{
		TIM3->CCR1=pulse_width;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);
		speed_in=-speed_in;
		speed=init_num/speed_in;
		if(speed<500){TIM3->ARR=200;}
		else 					{TIM3->ARR=speed;}
	}
	else
	{
		TIM3->CCR1=0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);
		TIM3->ARR=65535;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
	}
}
void DISP_ANGLE_POINT(float pitch,float yaw,float roll,int x,int y)
{
	char massage[20];
	OLED_ShowNum(1,1,x,3);
	OLED_ShowNum(1,6,y,3);
	sprintf(massage,"pitch: %.1f",pitch);
	OLED_ShowString(2,1,massage);
	sprintf(massage,"yaw: %.1f",yaw);
	OLED_ShowString(3,1,massage);
	sprintf(massage,"roll: %.1f",roll);
	OLED_ShowString(4,1,massage);
}
void DISP_POINT_SPEED(int x,int y,int speed,int speed2)
{
	char massage[20];
	OLED_ShowString(1,1,"MODE2");
	OLED_ShowNum(2,1,x,3);
	OLED_ShowNum(2,6,y,3);
	sprintf(massage,"speed: %d",speed);
	OLED_ShowString(3,1,massage);
	sprintf(massage,"speed2: %d",speed2);
	OLED_ShowString(4,1,massage);
}
void DISP_POINT_GYRO(int x_point_,int y_point_,int speed)
{
	char massage[20];
	OLED_ShowNum(1,1,x_point_,3);
	OLED_ShowNum(1,6,y_point_      ,3);
	sprintf(massage,"speed: %d",speed);
	OLED_ShowString(2,1,massage);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	short x,y,z;
	float pitch,roll,yall;
	int speed;
	int speed_2;
	int MODE;
	int counter=0;
	int button;
	char massage[20];
	int speed_sum;
	int time;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,rx_massage,100);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
	
	OLED_Init();
	MPU_Init();
	
	PID_Init(&yaw_control,DELTA,100,1000,3.0,0.01,0.2);
	OLED_ShowString(1,1,"waiting");
	OLED_ShowString(1,1,"       ");
	mpu_dmp_init();
	HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		mpu_dmp_get_data(&pitch,&roll,&yall);
//		MPU_Get_Gyroscope(&x,&y,&z);
//		DISP_ANGLE_POINT(pitch,yall,roll,x_point,y_point);
//		if(yall<err_angle && yall>-err_angle){speed=0;}
//		else{speed=PID_Calculate(&yaw_control,yall,0);}
//		OLED_ShowString(1,1,massage);
//		speed_step_motor2(speed,base);
		 
		OLED_Clear();
		MPU_Init();
		mpu_dmp_init();
		 HAL_Delay(100);
		while(1)
		{
			statue=0;
			OLED_ShowString(1,1,"MODE CHOOSE");
			OLED_ShowString(2,1,"MODE1");
			OLED_ShowString(3,1,"MODE2");
			OLED_ShowString(4,1,"MODE3");
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_SET){}
				MODE=1;
				break;
			}
			else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET){}
				MODE=2;
				break;
			}
			else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET){}
				MODE=3;
				break;
			}
			
//			button=main_manu();
//			if(button==1){
//				counter++;
//				if(counter>=4){counter=1;}
//			}
//			else if(button==2){
//				counter--;
//				if(counter==0){counter=3;}
//			}
//			else if(button==3){
//				MODE=counter;
//				break;
//			}
		}
		OLED_Clear();
		switch(MODE)
		{
			case 1:{PID_Init(&yaw_control,DELTA,100,1000,24.0,0.01,0.2);//division 256 p=24,divison 16 p=3
							while(statue==0)
							{
							mpu_dmp_get_data(&pitch,&roll,&yall);
							MPU_Get_Gyroscope(&x,&y,&z);
							DISP_ANGLE_POINT(pitch,yall,roll,x_point,y_point);
							if(yall<err_angle && yall>-err_angle){speed=0;}
							else{speed=PID_Calculate(&yaw_control,yall,0);}
							OLED_ShowString(1,1,massage);
							speed_step_motor2(speed,base);}}TIM3->CCR1=0;break;
			case 2:{
							PID_Init(&visual_control_2,DELTA,30,30,4,0,0.0002);//division 256 p=4.0,divison 16 p=0.5
							PID_Init(&visual_control,DELTA,30,100,5,0,0.0002);	 //division 256 p=4.0,divison 16 p=0.5
							while(statue==0)
							{
								if(x_point<=122 && x_point>=118 && y_point<=122 && y_point>=118)
								{
									speed=0;
									speed_2=0;
								}
								else if(x_point>122 && x_point<118 && y_point<=122 && y_point>=118)
								{
									speed=PID_Calculate(&visual_control,x_point,120);
									speed_2=0;
								}
								else if(x_point<=122 && x_point>=118 && y_point>122 && y_point<118)
								{
									speed=0;
									speed_2=PID_Calculate(&visual_control_2,y_point,120);
								}
								else
								{
									speed=PID_Calculate(&visual_control,x_point,120);
									speed_2=PID_Calculate(&visual_control_2,y_point,120);
								}
//							if(x_point<122 && x_point>118){speed=0;}
//							else{speed=PID_Calculate(&visual_control,x_point,120);}
//							
//							if(y_point<122 && y_point>118){speed_2=0;}
//							else{speed_2=PID_Calculate(&visual_control_2,y_point,120);}
							
							DISP_POINT_SPEED(x_point,y_point,speed,speed_2);
							speed_step_motor2(speed,base);
							speed_step_motor1(speed_2,base);

							}}TIM3->CCR1=0;TIM2->CCR1=0;break;
			
			case 3:{PID_Init(&visual_control,DELTA,30,100,4.0,0,0.0001);	//division 256 p=4.0,divison 16 p=0.5
							while(statue==0)
							{MPU_Get_Gyroscope(&x,&y,&z);
							sprintf(massage,"%d",z);
							OLED_ShowString(4,1,massage);        
							compensation_Z=z;
							compensation_value=compensation_Z*compsation_constant;
							 
							if(x_point<122 && x_point>118){speed=0;}
							else{speed=PID_Calculate(&visual_control,x_point,120);}
							
							time=SystemCoreClock;
							speed_sum=compensation_value+speed;
							DISP_POINT_GYRO(x_point,y_point,speed_sum);
							speed_step_motor2(speed_sum,base);}
							}TIM3->CCR1=0;break;
		}
			
		
			
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
