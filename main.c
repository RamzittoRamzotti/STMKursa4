/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <malloc.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void timer(int i);
int* checkPinState(int*);
void rightPass();
void wrongPass();
void wrongNewPass();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  char pass[]="12345";
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
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim3);
  char buf[20];
  char buf1[6];
  char newpass[6];
  int count=0;
  char newpassconf[6];
  memset(buf,0,strlen(buf));

  int access=0;
  int state[20]={};
  int i=0;
  int j=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		count=__HAL_TIM_GET_COUNTER(&htim4);
		if(access==0){
			 if(count>=10000){
				 memset(buf,0,sizeof(buf));
				 while(i!=0){
					 HAL_UART_Transmit(&huart3, "\b", 1, -1);
					 i--;
				 }

			 }
			 if(i<6){
				HAL_GPIO_WritePin(Eight_GPIO_Port, Eight_Pin, RESET);
				HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, SET);
				HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, SET);
				HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, SET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[1]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[1]++;
						buf[i]='1';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[4]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						buf[i]='4';
						state[4]++;
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[7]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						buf[i]='7';
						state[7]++;
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}HAL_GPIO_WritePin(Eight_GPIO_Port, Eight_Pin, SET);
				HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, RESET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[2]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[2]++;
						buf[i]='2';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[5]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[5]++;
						buf[i]='5';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[8]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[8]++;
						buf[i]='8';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, SET);
				HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, RESET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[3]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[3]++;
						buf[i]='3';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[6]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[6]++;
						buf[i]='6';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[9]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[9]++;
						buf[i]='9';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, SET);
				HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, RESET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[10]==0 ){
					state[10]++;
					if(i>0){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						buf[i-1]=NULL;
						HAL_UART_Transmit(&huart3, "\b", 1, -1);
						i--;
					}
				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[0]==0){
					if(i<5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[0]++;
						buf[i]='0';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[11]==0){
					if(i==5){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[11]++;
						buf[i]='#';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}
				}HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, SET);
				checkPinState(state);
			}else if (i==6){
					strcpy(buf1,buf);
					HAL_UART_Transmit(&huart3, pass, strlen(pass), -1);
					HAL_UART_Transmit(&huart3, "\r\n", 1, -1);
					if(buf[5]=='#'){
						buf1[5]='\0';
						char puk[6];
						int k=strlen(buf1)-strlen(pass);
						itoa(k,puk,10);
						HAL_UART_Transmit(&huart3, buf, strlen(buf), -1);
						HAL_UART_Transmit(&huart3, "\n\r", 2, -1);
						HAL_UART_Transmit(&huart3, pass, strlen(pass), -1);
						HAL_UART_Transmit(&huart3, "\n\r", 2, -1);
						HAL_UART_Transmit(&huart3, puk, strlen(puk), -1);
						if(strcmp(pass, buf1)==0){
							access=1;
							memset(buf,0,sizeof(buf1));
							memset(buf,0,sizeof(buf));
							i=0;
							rightPass();
						}else{
							wrongPass();
							memset(buf,0,sizeof(buf1));
							memset(buf,0,sizeof(buf));
							i=0;
						}
					}
				}

		}else{
			if(i<18){
				if(count=__HAL_TIM_GET_COUNTER(&htim4)>=10000){
						if(strlen(buf)!=0){
							memset(buf,0,sizeof(buf));
							while(i!=0){
								HAL_UART_Transmit(&huart3, "\b", 1, -1);
								i--;
							}
						}else{
							access=0;
						}
						__HAL_TIM_SET_COUNTER(&htim4,0);
				}
				HAL_GPIO_WritePin(Eight_GPIO_Port, Eight_Pin, RESET);
				HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, SET);
				HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, SET);
				HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, SET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[1]==0){
					if(i!=0 && i!=6 && i!=12){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						state[1]++;
						buf[i]='1';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}

				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[4]==0){
					if(i!=0 && i!=6 && i!=12){
						__HAL_TIM_SET_COUNTER(&htim4,0);
						buf[i]='4';
						state[4]++;
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
					}

				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[7]==0){
					if(i!=0 && i!=6 && i!=12){
						buf[i]='7';
						state[7]++;
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}HAL_GPIO_WritePin(Eight_GPIO_Port, Eight_Pin, SET);
				HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, RESET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[2]==0){
					if(i!=0 && i!=6 && i!=12){
						state[2]++;
						buf[i]='2';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[5]==0){
					if(i!=0 && i!=6 && i!=12){
						state[5]++;
						buf[i]='5';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[8]==0){
					if(i!=0 && i!=6 && i!=12){
						state[8]++;
						buf[i]='8';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, SET);
				HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, RESET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[3]==0){
					if(i!=0 && i!=6 && i!=12){
						state[3]++;
						buf[i]='3';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[6]==0){
					if(i!=0 && i!=6 && i!=12){
						state[6]++;
						buf[i]='6';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[9]==0){
					if(i!=0 && i!=6 && i!=12){
						state[9]++;
						buf[i]='9';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, SET);
				HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, RESET);
				if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_RESET && state[10]==0 ){
					state[10]++;
					if(i>0){
						buf[i-1]=NULL;
						HAL_UART_Transmit(&huart3, "\b", 1, -1);
						i--;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}if(HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_RESET && state[0]==0){
					if(i!=0 && i!=6 && i!=12){
						state[0]++;
						buf[i]='0';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}if(HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_RESET && state[11]==0){
					if(i==0 || i==6 || i==12){
						state[11]++;
						buf[i]='#';
						HAL_UART_Transmit(&huart3, &buf[i], 1, -1);
						i++;
						__HAL_TIM_SET_COUNTER(&htim4,0);
					}
				}HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, SET);
				checkPinState(state);
			}else{
				HAL_UART_Transmit(&huart3, "\r\nNachinaem smenu parolya\n\r\0", strlen("\r\nNachinaem smenu parolya\n\r\0"), -1);
				int p=1;
				while(p<6){
					buf1[p-1]=buf[p];
					p++;
				}buf1[p-1]='\0';
				if(strcmp(buf1,pass)==0){
					p=7;
					while(p<12){
						newpass[p-7]=buf[p];
						p++;
					}newpass[p-7]='\0';
					p=13;
					while(p<18){
						newpassconf[p-13]=buf[p];
						p++;
					}newpassconf[p-13]='\0';
					if(strcmp(newpass,newpassconf)==0){
						HAL_UART_Transmit(&huart3, "\r\nNachinaem smenu parolya\n\r\0", strlen("\r\nNachinaem smenu parolya\n\r\0"), -1);
						memset(pass,0,sizeof(pass));
						strcpy(pass,newpass);
						memset(newpass,0,sizeof(newpass));
						memset(newpassconf,0,sizeof(newpassconf));
						access=0;
						i=0;
						HAL_UART_Transmit(&huart3, "\r\nparol smenen\n\r\0", strlen("\r\nparol smenen\n\r\0"), -1);
					}else{
						memset(newpass,0,sizeof(newpass));
						memset(newpassconf,0,sizeof(newpassconf));
						wrongNewPass();
					}
				}else{
					HAL_UART_Transmit(&huart3, "\r\nNe verno vveli parol\n\r\0", strlen("\r\nNe verno vveli parol\n\r\0"), -1);
					wrongPass();

				}
				HAL_UART_Transmit(&huart3, "\r\ndostup zakryvaetsya\n\r\0", strlen("\r\ndostup zakryvaetsya\n\r\0"), -1);
				memset(buf,0,sizeof(buf));
				memset(buf1,0,sizeof(buf1));
				memset(newpass,0,sizeof(newpass));
				memset(newpassconf,0,sizeof(newpassconf));
				i=0;
			}
		}
		if(buf[0]==NULL && access==0){
			HAL_GPIO_WritePin(GRE_LED_GPIO_Port, GRE_LED_Pin, RESET);
			HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
			timer(500);
		}else if (buf[0]!=NULL && access==0){

			HAL_GPIO_WritePin(YEL_LED_GPIO_Port, YEL_LED_Pin, SET);
		}else if (buf[0]!=NULL && access==1){
			HAL_GPIO_WritePin(YEL_LED_GPIO_Port, YEL_LED_Pin, SET);
			HAL_GPIO_WritePin(GRE_LED_GPIO_Port, GRE_LED_Pin, SET);
		}else if (buf[0]==NULL && access==1){
			HAL_GPIO_WritePin(GRE_LED_GPIO_Port, GRE_LED_Pin, SET);
			HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
			timer(500);
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 11000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64000 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 11000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YEL_LED_Pin|RED_LED_Pin|GRE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Eight_Pin|Nine_Pin|Ten_Pin|Eleven_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Four_Pin Five_Pin Six_Pin */
  GPIO_InitStruct.Pin = Four_Pin|Five_Pin|Six_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : YEL_LED_Pin RED_LED_Pin GRE_LED_Pin */
  GPIO_InitStruct.Pin = YEL_LED_Pin|RED_LED_Pin|GRE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Eight_Pin Nine_Pin Ten_Pin Eleven_Pin */
  GPIO_InitStruct.Pin = Eight_Pin|Nine_Pin|Ten_Pin|Eleven_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void timer(int i){
	int count;
	while(count=__HAL_TIM_GET_COUNTER(&htim3)<i){
	}
	__HAL_TIM_SET_COUNTER(&htim3,0);
}
int* checkPinState(int state[20]){
	int j=0;
	HAL_GPIO_WritePin(Eight_GPIO_Port, Eight_Pin, RESET);
	HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, RESET);
	HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, RESET);
	HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, RESET);
	if(HAL_GPIO_ReadPin(Four_GPIO_Port, Four_Pin)==GPIO_PIN_SET && HAL_GPIO_ReadPin(Five_GPIO_Port, Five_Pin)==GPIO_PIN_SET && HAL_GPIO_ReadPin(Six_GPIO_Port, Six_Pin)==GPIO_PIN_SET){
			while(j<=20) {
				state[j]=0;
				j++;
			}
	}HAL_GPIO_WritePin(Eight_GPIO_Port, Eight_Pin, SET);
	HAL_GPIO_WritePin(Nine_GPIO_Port, Nine_Pin, SET);
	HAL_GPIO_WritePin(Ten_GPIO_Port, Ten_Pin, SET);
	HAL_GPIO_WritePin(Eleven_GPIO_Port, Eleven_Pin, SET);
	return state;
}
void rightPass(){
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET);
	timer(500);
	HAL_GPIO_WritePin(YEL_LED_GPIO_Port, YEL_LED_Pin, SET);
	timer(500);
	HAL_GPIO_WritePin(GRE_LED_GPIO_Port, GRE_LED_Pin, SET);
	timer(500);
	HAL_GPIO_WritePin(GRE_LED_GPIO_Port, GRE_LED_Pin, RESET);
	HAL_GPIO_WritePin(YEL_LED_GPIO_Port, YEL_LED_Pin, RESET);
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, RESET);
	HAL_UART_Transmit(&huart3, "\r\nYAHOOOOOO!!!\n\r\0", strlen("\r\nYAHOOOOOO!!!\n\r\0"), -1);
	HAL_UART_Transmit(&huart3, "\n\r", 2, -1);

}
void wrongPass(){
	HAL_GPIO_WritePin(YEL_LED_GPIO_Port, YEL_LED_Pin, RESET);
	timer(300);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(800);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
}
void wrongNewPass(){
	HAL_UART_Transmit(&huart3, "\r\nNoviye paroli ne sovpali\n\r\0", strlen("\r\nNoviye paroli ne sovpali\n\r\0"), -1);
	HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
	HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(800);
	HAL_GPIO_TogglePin(YEL_LED_GPIO_Port, YEL_LED_Pin);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	timer(300);
}
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
