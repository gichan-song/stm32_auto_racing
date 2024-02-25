/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BT
#define RX_BUF_SIZE	100



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_Data[1];
uint8_t state = 0;
//////////////////////////////////////////////////////// 코드 추�?1
uint8_t flag = 0;

volatile uint32_t INC_Value1 = 0;
volatile uint32_t INC_Value2 = 0;
volatile uint32_t echoTime_front = 0;
volatile uint8_t captureFlag1 = 0;
volatile uint32_t distance_front = 0;

volatile uint32_t INC_Value3 = 0;
volatile uint32_t INC_Value4 = 0;
volatile uint32_t echoTime_left = 0;
volatile uint8_t captureFlag2 = 0;
volatile uint32_t distance_left = 0;

volatile uint32_t INC_Value5 = 0;
volatile uint32_t INC_Value6 = 0;
volatile uint32_t echoTime_right = 0;
volatile uint8_t captureFlag3 = 0;
volatile uint32_t distance_right = 0;
//////////////////////////////////////////////////////// 코드 추�?1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	while((__HAL_TIM_GET_COUNTER(&htim1)) < us);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)	// if  interrupt source channel
				{
					if(captureFlag1 == 0)	// first value is not capture
					{
						INC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	// read first value
						captureFlag1 = 1;	// first captured as true

						// change polarity rising edge to falling edge
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
					}
					else if(captureFlag1 == 1)	// if first value already captured
					{
						INC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

						if(INC_Value2 > INC_Value1)
						{
							echoTime_front = INC_Value2 - INC_Value1;
						}
						else if(INC_Value2 < INC_Value1)
						{
							echoTime_front = (0xffff - INC_Value1) + INC_Value2;
						}

						distance_front = echoTime_front / 58;
						captureFlag1 = 0;

						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
						__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
					}
				}
	}

	if(htim->Instance == TIM2)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)	// if  interrupt source channel
				{
					if(captureFlag2 == 0)	// first value is not capture
					{
						INC_Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);	// read first value
						captureFlag2 = 1;	// first captured as true

						// change polarity rising edge to falling edge
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
					}
					else if(captureFlag2 == 1)	// if first value already captured
					{
						INC_Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

						if(INC_Value4  > INC_Value3)
						{
							echoTime_left = INC_Value4 - INC_Value3;
						}
						else if(INC_Value4 < INC_Value3)
						{
							echoTime_left = (0xffff - INC_Value3) + INC_Value4;
						}

						distance_left = echoTime_left / 58;
						captureFlag2 = 0;

						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
						__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
					}
				}
	}

	if(htim->Instance == TIM1)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)	// if  interrupt source channel
					{
					if(captureFlag3 == 0)	// first value is not capture
					{
						INC_Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);	// read first value
						captureFlag3 = 1;	// first captured as true

						// change polarity rising edge to falling edge
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
					}
					else if(captureFlag3 == 1)	// if first value already captured
					{
						INC_Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

						if(INC_Value6 > INC_Value5)
						{
							echoTime_right = INC_Value6 - INC_Value5;
						}
						else if(INC_Value6 < INC_Value5)
						{
							echoTime_right = (0xffff - INC_Value5) + INC_Value6;
						}

						distance_right = echoTime_right / 58;
						captureFlag3 = 0;

						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
						__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
					}
				}
	}
}

void Trig1(void)
{
	HAL_GPIO_WritePin(TrigL_GPIO_Port, TrigL_Pin, 1);	// Trig Pin High
	delay_us(10);								// delay 10us
	HAL_GPIO_WritePin(TrigL_GPIO_Port, TrigL_Pin, 0);	// Trig Pin Low

	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);	// SET Timer Enable
}

void Trig2(void)
{
	HAL_GPIO_WritePin(TrigC_GPIO_Port, TrigC_Pin, 1);	// Trig Pin High
	delay_us(10);								// delay 10us
	HAL_GPIO_WritePin(TrigC_GPIO_Port, TrigC_Pin, 0);	// Trig Pin Low

	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);	// SET Timer Enable
}

void Trig3(void)
{
	HAL_GPIO_WritePin(TrigR_GPIO_Port, TrigR_Pin, 1);	// Trig Pin High
	delay_us(10);								// delay 10us
	HAL_GPIO_WritePin(TrigR_GPIO_Port, TrigR_Pin, 0);	// Trig Pin Low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);	// SET Timer Enable
}

int PWM = 400;
void go()		// ?���???? ?��?��
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);	// Right ?��?��?��
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);	// Left ?��?��?��
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
	htim4.Instance->CCR3 = PWM;		// PWM RIGHT?�� �???? ?��?��
	htim4.Instance->CCR1 = PWM;		// PWM LEFT?�� �???? ?��?��
	//printf("This is Forward Function\r\n");	myDelay(500);
}

void back()		// ?���??? ?��?��
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);	// Right ?��?��?��
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);	// Left ?��?��?��
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
	htim4.Instance->CCR3 = PWM;		// PWM RIGHT?�� �???? ?��?��
	htim4.Instance->CCR1 = PWM;		// PWM LEFT?�� �???? ?��?��
	//printf("This is Backward Function\r\n");	myDelay(500);
} // end of Backward


void stop()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);	// Right ?��?��?��
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);	// Left ?��?��?��
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
} // end of Stop


void right()		// ?��?��?�� ?��?��		// Left ?��?��?��, Right ?��?��?��
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);	// Right ?��?��?��
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);	// Left ?��?��?��
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
	htim4.Instance->CCR3 = PWM;		// PWM RIGHT?�� �???? ?��?��
	htim4.Instance->CCR1 = PWM;		// PWM LEFT?�� �???? ?��?��
	//printf("This is Rotate_Right Function\r\n"); myDelay(500);
} // end of Rotate_R



void left()		// 좌회?�� ?��?��		// Left ?��?��?��, Right ?��?��?��
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);	// Right ?��?��?��
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);	// Left ?��?��?��
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
	htim4.Instance->CCR3 = PWM;		// PWM RIGHT?�� �???? ?��?��
	htim4.Instance->CCR1 = PWM;		// PWM LEFT?�� �???? ?��?��
	//printf("This is Rotate_Left Function\r\n");	myDelay(500);
} // end of Rotate_L

///////////////////////////////////////////////////코드 추�?2
//
//
void left_back()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
}

void right_back()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
}
//
//
//
void at_motor()
{
	if(flag == 1)
	{
		if(distance_front < 10)
		{
			if(distance_right > distance_left)
			{
			  left_back();
			  htim4.Instance->CCR3 = 1000;
			  htim4.Instance->CCR1 = 0;
			}
			else
			{
			  right_back();
			  htim4.Instance->CCR3 = 0;
			  htim4.Instance->CCR1 = 1000;
			}
		}
		else if(distance_front < 60)
		{
			if(distance_left < 20 || distance_right < 20)
			{
				if(distance_left < distance_right)
				{
					right();
					htim4.Instance->CCR3 = 0;
					htim4.Instance->CCR1 = 1000;
				}
				else
				{
					left();
					htim4.Instance->CCR3 = 1000;
					htim4.Instance->CCR1 = 0;
				}
			}
			else
			{
				go();
				htim4.Instance->CCR3 = 1000;
				htim4.Instance->CCR1 = 1000;
			}
		}
		else if(distance_left < 14 || distance_right < 14)
		{
			if(distance_left < distance_right)
			{
				right();
				htim4.Instance->CCR3 = 0;
				htim4.Instance->CCR1 = 1000;
			}
			else
			{
				left();
				htim4.Instance->CCR3 = 1000;
				htim4.Instance->CCR1 = 0;
			}
		}
		else
		{
			go();
			htim4.Instance->CCR3 = 1000;
			htim4.Instance->CCR1 = 1000;
		}
	}
}

/////////////////////////////////////////////////코드 추�?2



void bt_motor()
{

	if(rx_Data[0] == 'H')
	{
		PWM = 600;
	}
	else if(rx_Data[0] == 'M')
	{
		PWM = 300;
	}
	else if(rx_Data[0] == 'L')
	{
		PWM = 100;
	}
	else if(rx_Data[0] == 'f')
	{
	  state = 'f';
	}
	else if(rx_Data[0] == 's')
	{
	  state = 's';
	}
  else if(rx_Data[0] == 'l')
	{
	  state = 'l';

	}
  else if(rx_Data[0] == 'r')
	{
	  state = 'r';

	}
  else if(rx_Data[0] == 'b')
	{
	  state = 'b';
	}

	if(state == 'f'){
		go();
	}
	else if(state == 's')
	{
		stop();
	}
	else if(state == 'l'){
		left();
	}
	else if(state == 'r'){
		right();
	}
	else if(state == 'b')
	{
		back();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		HAL_UART_Receive_IT(&huart1, rx_Data, sizeof(rx_Data));
	}

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) // printf
{
	// console output function

	HAL_UART_Transmit(&huart2, &ch, 1, 10); // 1byte transfer
	// &ch is stack pointer(ch's data address)

	return ch;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim1); // delay_us
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);


//
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  Forward();
	  HAL_UART_Receive_IT(&huart1, rx_Data, sizeof(rx_Data));
	  printf("%c\r\n", rx_Data[0]);
	  if(rx_Data[0] == 'a') flag = !flag;
	  if(!flag) bt_motor();
	  else at_motor();
	  Trig1();
//	  HAL_Delay(10);
	  Trig2();
//	  HAL_Delay(10);
	  Trig3();
//	  printf("%c\r\n", rx_Data[0]);
	  printf("sonic left: %d\r\n", distance_left);
	  printf("sonic center: %d\r\n", distance_front);
	  printf("sonic right: %d\r\n", distance_right);
//	  HAL_Delay(200);
///////////////////////////////////////////////////코드 추�?3
////

//	  rx_Data[0] = 0;


///////////////////////////////////////////////////코드 추�?3



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
