/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
    #include "string.h"
		#include "stdlib.h"
		#include <stdint.h>
		#include "turn_status.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define delay                                                  HAL_Delay  


 #define button_not_pressed    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==GPIO_PIN_SET
 #define button_pressed        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==GPIO_PIN_RESET
 
  #define ledr1    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
  #define ledr0    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
  #define ledg1    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
  #define ledg0    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
  #define ledy1    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
  #define ledy0    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)	
	
	#define pov_lev1    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
  #define pov_lev0    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
	#define pov_prav1    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
  #define pov_prav0    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
	
	#define podsvet_prav1    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
  #define podsvet_prav0    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
	#define podsvet_lev1    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
  #define podsvet_lev0    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
	
	#define gabarit1    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
  #define gabarit0    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
	#define blijny1    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
  #define blijny0    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)
	
	
	 #define in_POV_LEV        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) 
   #define in_POV_PRAV       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)
	 #define in_HAZARD         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
   #define zad_on()    ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) == GPIO_PIN_RESET)
 
  #define mem_angle_1  ee_massiv[0] 		
  #define mem_angle_2  ee_massiv[1] 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//typedef enum {
//TURN_OFF = 0,
//TURN_LEV,
//TURN_PRAV,
//BROKEN_LEV,
//BROKEN_PRAV,
//PASIB,	
//HAZARD
//} TurnStatus;
 TurnStatus status_pov = TURN_OFF;       // sostoyanie ukazatelya povorota
uint8_t status_pov; 
 
char uart_rx_buffer[20]; // Bufer priema UART
volatile uint8_t uart_rx_index = 0; // Indeks bufera (ostorozhno s perepolneniem!)
volatile int16_t uart_data_received = 0; // Prinjatoe znachenie
volatile uint8_t uart_data_ready = 0; // Flag gotovnosti dannykh

    uint8_t prev = 0; 
  //  uint16_t tick_pov=0;	 
	 	uint8_t status_prev; 
	 // uint8_t praviy_prev = 0; 
      uint16_t tick_pov_prav=0, tick_pov_lev=0;	 
	 	uint16_t tick_hazard ; 
	 //   uint8_t status_pov_prav;
			
      uint8_t several_times = 0;
			static uint8_t zad_hazard_active = 0;
			
volatile uint16_t rpm_raw = 0;
volatile uint16_t speed_raw = 0;

volatile uint16_t delta_rpm = 0;
volatile uint16_t delta_speed = 0;

uint16_t rpm;
uint16_t speed_kmh;			
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
// if (status_pov == TURN_OFF) return;
	switch (status_pov) 
		{
			
case PASIB:
{
     several_times++;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); // левый
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // правый
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
}
break;				
case HAZARD:
{
   // several_times++;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); // левый
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // правый
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
}
break;			
case TURN_LEV:
{
	// several_times++;
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
//	 several_times=0;
}
break;
case TURN_PRAV: 
{
	//  several_times++;
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
	//  several_times=0; 
}
break;
case BROKEN_LEV:
 {
	 several_times++;
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
 }
 break;
case BROKEN_PRAV: 
	{
	 several_times++;
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
}
break;
case TURN_OFF:
default:
if (prev == 1)
	{
	 prev=0;	
   several_times = 0;
	  ledg0;
	 ledy0;
	 pov_lev0;
	 pov_prav0; 
  }
break;
}
	
				
	///////////////////////////////////////////////////////////////////	
  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
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
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[0], 1);
	
	 HAL_TIM_Base_Start_IT(&htim14);
	 	 HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);
	// HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
   HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
if(zad_on()) {
    if(status_pov == TURN_OFF) {
        status_pov = HAZARD;
        zad_hazard_active = 1;
    }
} else {
    if(zad_hazard_active && status_pov == HAZARD) {
        status_pov = TURN_OFF;
        zad_hazard_active = 0;

        // 🛑 Гасим всё, что может мигать:
           ledy0;
			     ledg0;

        // ⚡️ Гасим силовые:
        pov_lev0;
        pov_prav0;
    }
}
		
if(status_pov != HAZARD)
{	
 /////////////////////// POVOROT  /////////////////////////////////////////
	
	
	
// if(status_pov == TURN_OFF)
// {

//	 ///////////////////////////////////////////////////
//	 several_times=0;
//	 ledg0;
//	 ledy0;
//	 pov_lev0;
//	 pov_prav0;
//	 prev=0;
// }
  
		 if((status_pov==TURN_LEV) || (status_pov==BROKEN_LEV))  ledg0, pov_prav0;
		 if((status_pov==TURN_PRAV) || (status_pov==BROKEN_PRAV)) ledy0, pov_lev0 ;
	 
   if(status_pov!=TURN_OFF&&prev==0)
	 {
		 __HAL_TIM_SET_COUNTER(&htim14, 0);
 		 prev=1;  
		 if(status_pov==TURN_LEV) ledy1, pov_lev1, ledg0, pov_prav0;
		 if(status_pov==TURN_PRAV) ledg1, pov_prav1,ledy0, pov_lev0 ;
	 }

 if(several_times>=7)		
 {
	 //status_pov = status_prev;
        if ((status_pov == PASIB && status_prev == TURN_LEV) ||  (status_pov == PASIB && status_prev == TURN_PRAV))
        {
            status_pov = status_prev;
					status_prev=0;
					several_times = 0;
        }
        else
        {
            status_pov = TURN_OFF;
        }
 //    status_pov = TURN_OFF  ; status_prev == TURN_LEV
 }	  
		
if(in_POV_LEV==0&&tick_pov_lev<500)   tick_pov_lev++;
if(in_POV_LEV==1 && tick_pov_lev<50)  tick_pov_lev=0;
if(in_POV_LEV==1 && tick_pov_lev>150) tick_pov_lev=0;
if(in_POV_LEV==1 && tick_pov_lev>=50) 
	{ 
 if(status_pov==BROKEN_PRAV || status_pov==TURN_PRAV) {
        ledg0;
        pov_prav0;
        status_pov=TURN_OFF;
    }
    else {
        ledy1;
        pov_lev1;
        status_pov=BROKEN_LEV;
    }
	  tick_pov_lev=0;    
  }

 if(tick_pov_lev>150)
		{ 
		 status_pov=TURN_LEV; 
		}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
 if(in_POV_PRAV==0&&tick_pov_prav<500)   tick_pov_prav++;
if(in_POV_PRAV==1 && tick_pov_prav<50)  tick_pov_prav=0;
if(in_POV_PRAV==1 && tick_pov_prav>150) tick_pov_prav=0;
if(in_POV_PRAV==1 && tick_pov_prav>=50) 
	{ 
		
		 if(status_pov==BROKEN_LEV || status_pov==TURN_LEV) {
        ledy0;
        pov_lev0;
        status_pov=TURN_OFF;
    }
    else {
        ledg1;
        pov_prav1;
        status_pov=BROKEN_PRAV;
    }
    tick_pov_prav=0;
		 
  }

 if(tick_pov_prav>150)
		{ 
		 status_pov=TURN_PRAV; 
		}
		 
 
		
		  /////////////////////// END POVOROT  /////////////////////////////////////////
	}
////////////////////////  HAZARD   /////////////////////////////////////////////////
 
		if(in_HAZARD==0&&tick_hazard<500)   tick_hazard++;
		if(in_HAZARD==1 && tick_hazard<50)  tick_hazard=0;
	//	if(in_HAZARD==1 && tick_hazard>150) tick_hazard=0;
		if(in_HAZARD==1 && tick_hazard>=50) 
     {  
			    if (status_pov == TURN_LEV || status_pov == TURN_PRAV)  status_prev = status_pov;
      
								ledy0;
							  ledg0;
							  pov_prav0;
								pov_lev0; 
			 
             status_pov = PASIB;
						 several_times=0;
	     tick_hazard=0;    
     }

  if(tick_hazard>150)
		{ 
			   
								ledy0;
							  ledg0;
							  pov_prav0;
								pov_lev0; 
              
		 if(status_pov != HAZARD) status_pov = HAZARD;
     else 			status_pov = TURN_OFF ;
			while (in_HAZARD==0 && tick_hazard>150){};
				delay(50);
				tick_hazard=0;
				
		}
		
		//////////////////////// END  HAZARD   /////////////////////////////////////////////////
		
 /////////////////////// USART  /////////////////////////////////////////
//		 if (uart_data_ready)
//			 { 
//            if (uart_data_received < -20) ledy1; 
//					  else   ledy0;   

//            if (uart_data_received > 20) ledg1;  
//				    else ledg0; 
//            uart_data_ready = 0; 
//        }
 ///////////////////////  END USART  /////////////////////////////////////////
 
 		
		
		
		
		
		delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 450;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 479;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 479;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 47;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, gabari_out_Pin|blijniy_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, pow_lew_Pin|pow_praw_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, uderjanie_Pin|led1_Pin|led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : gabari_out_Pin blijniy_Pin */
  GPIO_InitStruct.Pin = gabari_out_Pin|blijniy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : pow_lew_Pin pow_praw_Pin */
  GPIO_InitStruct.Pin = pow_lew_Pin|pow_praw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : zazhiganiye_Pin */
  GPIO_InitStruct.Pin = zazhiganiye_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(zazhiganiye_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : zadnij_Pin awarijka_in_Pin gabarit_in_Pin knopa_Pin
                           IN_PRAW_Pin IN_LEW_Pin */
  GPIO_InitStruct.Pin = zadnij_Pin|awarijka_in_Pin|gabarit_in_Pin|knopa_Pin
                          |IN_PRAW_Pin|IN_LEW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : uderjanie_Pin led1_Pin led2_Pin */
  GPIO_InitStruct.Pin = uderjanie_Pin|led1_Pin|led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
