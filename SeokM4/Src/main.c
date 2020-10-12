/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <motor.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rx4_data,
		rx3_data,
		rx2_data;
int8_t int8_tim7flag=1;

char g_char_ReciveG[25];
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
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char* p,int len)
{
	HAL_UART_Transmit(&huart3,p,len,10);
	return len;
}
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
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_UART4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart4, &rx4_data,1);
HAL_UART_Receive_IT(&huart3, &rx3_data,1);
HAL_UART_Receive_IT(&huart2, &rx2_data,1);
HAL_TIM_Base_Start_IT(&htim7);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t ccr=0;
  InitMotor_L(&R_Motor);
  InitMotor_R(&L_Motor);
  g_uint16_pwm_flag=0;
  //R_Motor.f_User_Velocity=-200;
  L_Motor.f_User_Velocity=-200;
  g_servo_pulse=3200;
  while (1)
  {
	  HAL_UART_Receive_IT(&huart3, &rx3_data,1);
	  HAL_UART_Receive_IT(&huart2, &rx2_data,1);
	  HAL_UART_Receive_IT(&huart4, &rx4_data,1);
	 //__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ccr);
	 //TIM4->CCR1=ccr;
	 //ccr+=1000;
	 // if(ccr>TIM4->ARR) ccr=0;
	  HAL_Delay(20);
	  //printf("val0=%4d  val1=%4d  val2=%4d\r\n", adcval[0],adcval[1], adcval[2]);
	 // printf("roll : %3.2f, pitch : %3.2f, yaw : %3.2f\n\r",f_roll,f_pitch,f_yaw);
	 // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
	  //printf("term : %04d\n\r",(uint32_t)R_Motor.f_pidoutresult);
	  //printf("cur : %4.4f   usr : %4.0f\n\r",R_Motor.f_Current_Velocity,R_Motor.f_User_Velocity);
	  //printf("roll : %3.2f, pitch : %3.2f, yaw : %3.2f, accel : %d\n\r",f_roll,f_pitch,f_yaw,(int)L_Motor.i32Accel);
	  //printf("enc : %4d\n\r",R_Motor.U16Qep_Sample);
	 //printf("Rm_usspeed:%4.2f,  RM_speed=%4.2f\n\r",R_Motor.f_User_Velocity ,R_Motor.f_pidoutresult);
	  //printf("offset : %2.2f, angle : %2.2f, kp:%4.2f,  ki:%4.2f, kd:%4.2f ,spd:%4.2f, nextv:%4.2f, pidout:%4.2f, acc:%4.2f\n\r",Angle.f_Next_angle_roll, Angle.f_Current_angle_roll,Angle.f_kp_roll,Angle.f_ki_roll,Angle.f_kd_roll,L_Motor.f_Current_Velocity_av,L_Motor.f_Next_Velocity,Angle.f_pidoutterm_roll,L_Motor.f_Accel);
	  printf("%2.2f, %2.2f, %4.2f\n\r",Angle.f_Next_angle_roll, Angle.f_Current_angle_roll, f_roll);
	  //printf("%d\n\r",g_servo_pulse);
	  //printf("kp:%4.2f,  ki:%4.2f, kd:%4.2f, servo:%d\n\r",Angle.f_kp_roll,Angle.f_ki_roll,Angle.f_kd_roll,g_servo_pulse);
	  //printf("%lf\n\r",sinf(f_pitch*0.017453));

	 /* HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  */

	 /*HAL_UART_Transmit(&huart3,&a, 1, 10);
	  HAL_Delay(1000);
	  if(HAL_UART_Receive(&huart3,&a,1,10)==HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3,&a, 1, 10);
	  }
	 */
\
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
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
  /**Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char receiver = 0;

	if(huart->Instance == USART3)
	{
			switch(rx3_data)
			{
				case 'w':
					 R_Motor.f_User_Velocity+=100;break;
				case 's':
					R_Motor.f_User_Velocity-=100;break;

				case 'a':
					g_servo_pulse-=50;break;
				case 'd':
					g_servo_pulse+=50;break;
				case 'j':
					Angle.f_kp_roll-=1;break;
				case 'u':
					Angle.f_kp_roll+=1;break;
				case 'k':
					Angle.f_kd_roll-=10;break;
				case 'i':
					Angle.f_kd_roll+=10;break;
				case 'l':
					Angle.f_ki_roll-=0.01;break;
				case 'o':
					Angle.f_ki_roll+=0.01;break;
				case '[':
					Angle.f_Next_angle_roll-=0.1;break;
				case ']':
					Angle.f_Next_angle_roll+=0.1;break;
				case 'g':
					g_uint16_pwm_flag=1;break;
				case 'f':
					g_uint16_pwm_flag=0;break;
			}
			//HAL_UART_Transmit(&huart3, &rx3_data, 1, 10);

			if( R_Motor.f_User_Velocity>4000)
				R_Motor.f_User_Velocity=4000;
			if( R_Motor.f_User_Velocity<-4000)
				R_Motor.f_User_Velocity=-4000;
//			if( L_Motor.f_User_Velocity>3000)
//				L_Motor.f_User_Velocity=3000;
//			if( L_Motor.f_User_Velocity<-3000)
//				L_Motor.f_User_Velocity=-3000;
			if(g_servo_pulse<2550)
				g_servo_pulse=2550;//900
			if(g_servo_pulse>3850)
				g_servo_pulse=3850;//5500

			rx3_data=NULL;
			HAL_UART_Receive_IT(&huart3, &rx3_data,1);


	}

	if(huart->Instance == USART2)
		{

		//HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
		receiver = rx2_data;
		//printf("%d\n\r",receiver);
			if( receiver == '\n' )
			{


				float temp_float[3];
				int num =0;
				for(int start=1,l=1;l<25;l++)
				{
					if(*(g_char_ReciveG+l)==','||*(g_char_ReciveG+l)=='\r')
					{
						*(g_char_ReciveG+l)=0;
						temp_float[num++] = atof(g_char_ReciveG+start);
						start = l+1;
						if(*(g_char_ReciveG+l)=='\r')	break;
					}
				}
				f_roll=temp_float[0];
				f_pitch=temp_float[1];
				f_yaw=temp_float[2];

				//printf("roll : %3.2lf, pitch : %3.2lf, yaw : %3.2lf\n\r",temp_float[0],temp_float[1],temp_float[2]);

				memset((void*) g_char_ReciveG,0, sizeof(char)*25 );

			}
			else
				strncat( (char *)g_char_ReciveG, (char *)&receiver, 1 );
			HAL_UART_Receive_IT(&huart2, &rx2_data,1);
		}
	if(huart->Instance==UART4)
	{
		switch(rx4_data)
					{
						case 'x':
							 R_Motor.f_User_Velocity=0;break;
						case 'w':
							 R_Motor.f_User_Velocity=200;break;
						case 's':
							R_Motor.f_User_Velocity=100;break;
						case 'a':
							g_servo_pulse-=50;break;
						case 'd':
							g_servo_pulse+=50;break;
						case 'j':
							Angle.f_kp_roll-=1;break;
						case 'u':
							Angle.f_kp_roll+=1;break;
						case 'k':
							Angle.f_kd_roll-=10;break;
						case 'i':
							Angle.f_kd_roll+=10;break;
						case 'l':
							Angle.f_ki_roll-=0.01;break;
						case 'o':
							Angle.f_ki_roll+=0.01;break;
						case '[':
							Angle.f_Next_angle_roll-=0.1;break;
						case ']':
							Angle.f_Next_angle_roll+=0.1;break;
						case 'g':
							g_uint16_pwm_flag=1;break;
						case 'f':
							g_uint16_pwm_flag=0;break;
						case '1':
							g_servo_pulse=2600;break;
						case '2':
							g_servo_pulse=2900;break;
						case '3':
							g_servo_pulse=3200;break;
						case '4':
							g_servo_pulse=3500;break;
						case '5':
							g_servo_pulse=3800;break;
					}
		if( R_Motor.f_User_Velocity>4000)
			R_Motor.f_User_Velocity=4000;
		if( R_Motor.f_User_Velocity<-4000)
			R_Motor.f_User_Velocity=-4000;
		if(g_servo_pulse<2550)
			g_servo_pulse=2550;//900
		if(g_servo_pulse>3850)
			g_servo_pulse=3850;//5500
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM7)
//	{
////		if(int8_tim7flag==1)
////		{
////			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
////
////			int8_tim7flag=0;
////		}
////		else
////		{
////			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
////			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
////			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
////			int8_tim7flag=1;
////		}
//	}
//}
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
