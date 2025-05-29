/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cg9a01.h"

//#include "odin.h"
#include "dva.h"
#include "tri.h"
#include "chet.h"
#include "pjt.h"
#include "shest.h"
#include "sem.h"
#include "vos.h"
#include "dev.h"
//   #include "nol.h"

#include "d_odin.h"
#include "d_dva.h"
#include "d_tri.h"
#include "d_chet.h"
#include "d_pjt.h"
#include "d_shest.h"
#include "d_sem.h"
#include "d_vos.h"
#include "d_dev.h"
#include "d_nol.h"
#include "s_odin.h"
#include "s_dva.h"
#include "s_tri.h"
#include "s_chet.h"
#include "s_pjt.h"
#include "s_shest.h"
#include "s_sem.h"
#include "s_vos.h"
#include "s_dev.h"
#include "s_nol.h"
#include "toch.h"
#include "bez_toch.h" 
#include "fon.h"
#include "smil.h"
#include "zaved.h"   //  deva
#include "fig.h" 
#include "acc.h"
#include "zajig.h"
#include "nezavod.h"
//    #include "startuem.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

 //# define blk     TIM2->CCR4  
 
       //    GPIOA->BSRR = GPIO_BSRR_BR4;  
 
#define podsvet1    GPIOA->BSRR = GPIO_BSRR_BS2 
#define podsvet0    GPIOA->BSRR = GPIO_BSRR_BR2

#define uderj1    GPIOC->BSRR = GPIO_BSRR_BS13 
#define uderj0    GPIOC->BSRR = GPIO_BSRR_BR13

 #define knop      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) 
  #define cepa      HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_0)
	 #define lock      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
	  #define unlock      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)
		 #define netral      HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_1) 
		 
	//=================  =========================================================	 
	
#define ACC1     GPIOA->BSRR = GPIO_BSRR_BS10 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET) 
#define ACC0     GPIOA->BSRR = GPIO_BSRR_BR10   /// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)		
#define IGN11    GPIOA->BSRR = GPIO_BSRR_BS9  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET) 
#define IGN10    GPIOA->BSRR = GPIO_BSRR_BR9  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)
#define IGN21    GPIOA->BSRR = GPIO_BSRR_BS8  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET) 
#define IGN20    GPIOA->BSRR = GPIO_BSRR_BR8  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define PUSK1    GPIOC->BSRR = GPIO_BSRR_BS9  //  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET) 
#define PUSK0    GPIOC->BSRR = GPIO_BSRR_BR9  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)	 
		 
//================================================================================		 
		 
// # define       OFF	     sostojnie==0
// # define       minut10	 sostojnie==1
// # define       ACC	     sostojnie==2
// # define       IGN1	   sostojnie==3
// # define       IGN2	   sostojnie==4
// # define       START	   sostojnie==5 
		 

 #define delay   HAL_Delay 
 #define knop_najata knop==0
 #define knop_net    knop==1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

//const uint16_t odin[];
const uint16_t dva[];
const uint16_t tri[];
const uint16_t chet[];
const uint16_t pjt[];
const uint16_t shest[];
const uint16_t sem[];
const uint16_t vos[];
const uint16_t dev[];
//  const uint16_t nol[];
const uint16_t d_odin[];
const uint16_t d_dva[];
const uint16_t d_tri[];
const uint16_t d_chet[];
const uint16_t d_pjt[];
const uint16_t d_shest[];
const uint16_t d_sem[];
const uint16_t d_vos[];
const uint16_t d_dev[];
const uint16_t d_nol[];
const uint16_t s_odin[];
const uint16_t s_dva[];
const uint16_t s_tri[];
const uint16_t s_chet[];
const uint16_t s_pjt[];
const uint16_t s_shest[];
const uint16_t s_sem[];
const uint16_t s_vos[];
const uint16_t s_dev[];
const uint16_t s_nol[];
const uint16_t toch[];
const uint16_t bez_toch[]; 
const uint16_t fon[];
const uint16_t smil[];
const uint16_t acc[];
const uint16_t zaved[];
const uint16_t fig[];
const uint16_t zajig[];   
const uint16_t nezavod[];
//  const uint16_t startuem[];

uint8_t sostojnie, toch_tik, to, sekun=5, minut=9, raz, buf_to, najatie, raz_smil;  //  odinn, 

uint16_t tik_knop, pervyj_raz ;

volatile uint16_t  Counter; //   counter1,counter0,, gap

uint8_t sec_start, tic_Counter, raz_zajig;   


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void odinraz_smil (void)
{
	if(raz_smil==0)  DrawImage(0, 0, 240, 240,(uint16_t*) smil), raz_smil=1;  //   if(knop_najata&&raz==0)   
	  
	if(raz_zajig==0) DrawImage(0, 0, 240, 240,(uint16_t*) zajig), raz_zajig=1; 
	
}
void odinraz (void)
{ 
   DrawImage(0, 0, 240, 240,(uint16_t*) fon); 
	 
	 raz=1;
}

void tochki (void)
{	
	//   raz=1; 
		 switch (to)
			 {
						case 0:
						{
						 DrawImage(87, 92, 13, 55,(uint16_t*) toch);
							
						       
							
							if(sekun==0&&toch_tik==20)     DrawImage(114, 82, 46, 76,(uint16_t*) d_nol) ,minut=minut-1, sekun=6 ;
								  if(sekun==0)     DrawImage(114, 82, 46, 76,(uint16_t*) d_nol);
									if(sekun==1)     DrawImage(114, 82, 46, 76,(uint16_t*) d_odin);  
									if(sekun==2)     DrawImage(114, 82, 46, 76,(uint16_t*) d_dva) ; 
									if(sekun==3)     DrawImage(114, 82, 46, 76,(uint16_t*) d_tri) ; 
									if(sekun==4)     DrawImage(114, 82, 46, 76,(uint16_t*) d_chet) ;
									if(sekun==5)     DrawImage(114, 82, 46, 76,(uint16_t*) d_pjt) ;  
							
							
							
				 			if(minut==9)    DrawImage(28, 82, 46, 76,(uint16_t*) dev) ;
				 			if(minut==8)    DrawImage(28, 82, 46, 76,(uint16_t*) vos) ;
				 			if(minut==7)    DrawImage(28, 82, 46, 76,(uint16_t*) sem) ;
				 			if(minut==6)    DrawImage(28, 82, 46, 76,(uint16_t*) shest) ;
				 			if(minut==5)    DrawImage(28, 82, 46, 76,(uint16_t*) pjt) ;
				 			if(minut==4)     DrawImage(28, 82, 46, 76,(uint16_t*) chet) ;
				 			if(minut==3)     DrawImage(28, 82, 46, 76,(uint16_t*) tri) ; 
				 			if(minut==2)     DrawImage(28, 82, 46, 76,(uint16_t*) dva) ;
				 			if(minut==1)     DrawImage(28, 82, 46, 76,(uint16_t*) s_odin);  
							if(minut==0)    DrawImage(28, 82, 46, 76,(uint16_t*) s_nol);
							
							      if(toch_tik==2)      DrawImage(168, 82, 46, 76,(uint16_t*) s_dev) ;    
									 if(toch_tik==4)      DrawImage(168, 82, 46, 76,(uint16_t*) s_vos) ;
									 if(toch_tik==6)      DrawImage(168, 82, 46, 76,(uint16_t*) s_sem) ;    
									 if(toch_tik==8)      DrawImage(168, 82, 46, 76,(uint16_t*) s_shest) ;
									 if(toch_tik==10)      DrawImage(168, 82, 46, 76,(uint16_t*) s_pjt) ;  
									 if(toch_tik==12)     DrawImage(168, 82, 46, 76,(uint16_t*) s_chet) ;
									 if(toch_tik==14)     DrawImage(168, 82, 46, 76,(uint16_t*) s_tri) ; 
									 if(toch_tik==16)     DrawImage(168, 82, 46, 76,(uint16_t*) s_dva) ; 
									 if(toch_tik==18)     DrawImage(168, 82, 46, 76,(uint16_t*) s_odin);  
									 if(toch_tik==20)     DrawImage(168, 82, 46, 76,(uint16_t*) s_nol),  toch_tik=0, sekun=sekun-1; 
									  							  
						}
					 break;
				case 1:
				DrawImage(87, 92, 13, 55,(uint16_t*) bez_toch);  
					break; 
			} 
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	 uderj1;
	 if(knop_najata||unlock==0) sostojnie=20;
	 while (knop_najata||unlock==0) {};
 //	 delay(50);
	 	podsvet1;  
		 ACC1; 
  GC9A01_Initial();
	 
	// HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_3);   //  blk 
	  	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);    // oboroty  
		 HAL_TIM_Base_Start_IT(&htim4);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
	
  while (1)
  {
//		 		 WriteString(70, 100,"ÏÐÎÑÒÎ", Font_16x26, WHITE, BLACK);
// 		     WriteString(70, 140,"POWEPÜ", Font_12x20, WHITE, BLACK);  
//		     ClearScreen2(BLACK);		 
		
		 
		
  if(sekun==0&&minut==0&&toch_tik>18) sostojnie=0; 
  
//========================  KNOPKA  ===========================================================  
 
	if(knop_najata) 
	{
		tik_knop++;
		if(raz_smil==0)  odinraz_smil(); 
	} 
	if(knop_net&&tik_knop<10) tik_knop=0;
	if(knop_net&&tik_knop>=10)   
			{
				//==========  korotko  ================================ 
				ClearScreen2(BLACK); 
				tik_knop=0;
				raz=0; 
				raz_smil=0;
			  najatie=1;  
				sec_start=0;
			  // tic_Counter=0;  
			}

	if(tik_knop>5000)  
			{
			 //	ClearScreen2(BLACK);
			  	DrawImage(0, 0, 240, 240,(uint16_t*) nezavod) ;	
				while(knop_najata) {
				                     ACC0;
														 IGN10;
														 IGN21;
														 PUSK1;
				                    }
				najatie=1;
   			sostojnie=1;				 							
				tik_knop=0; 
				delay(50);
			}	
////=================== end KNOPKA  =========================================	

 
			 /*
          cepa
		      netral
			
			   ACC 10min     = sostojnie==1
			   ACC           = sostojnie==2
			   IGN1 + IGN2   = sostojnie==3
			   IGN1          = sostojnie==4  // otklyuchayetsya na start 
			   IGN2          = sostojnie==5
			   START auto    = sostojnie==6
			   START manual  = sostojnie==7
			   OFF           = sostojnie==0
			 */			
     if(najatie==1)
		   {		 
				 
				 					
					 if((sostojnie==9&&najatie==1)||(sostojnie==10&&najatie==1))    
								{
									najatie=0;
									sostojnie=1;
								   ACC1;
									 IGN10;
									 IGN20;
									 PUSK0;
								 	tic_Counter=0;
									sec_start=0 ; 
									raz=0;  
								}	
				 
				 
				  if(cepa==1)
					{
				        //   ==========  NA PEREDACHE  ===============================================	 //  
								if(sostojnie!=0&&netral==1&&najatie==1) 
								{
									DrawImage(0, 0, 240, 240,(uint16_t*) fig);
									 delay(1000); 
									sostojnie=0;	
									while(knop_najata) { };
									delay(50);
									 DrawImage(0, 0, 240, 240,(uint16_t*) fon);  
								}
								//   ==========   END  NA PEREDACHE  ===============================================	 
	
									if((sostojnie==9&&najatie==1)|| (sostojnie==10&&najatie==1))
									{
									 sostojnie=1;    
									}
								//   ==========   ACC 10 MINUT  ===============================================					
								if(sostojnie==0&&najatie==1)    // ACC 10min
								{ 

									 sostojnie=1; 
									 najatie=0;
									 ACC1;
									 IGN10;
									 IGN20;
									 PUSK0;
								}		
			        	//   ==========   END  ACC 10 MINUT  ===============================================					
							 
								if((sostojnie==1&&najatie==1)||(sostojnie==20&&najatie==1))  // IGN1 + IGN2
								{ 
									sostojnie=3;
										najatie=0;
									 ACC1;
									 IGN11;
									 IGN21;
									 PUSK0;
									 DrawImage(0, 0, 240, 240,(uint16_t*) zajig) ; 
								}				
 
								if(sostojnie==3&&najatie==1)     // ACC
								{
										sostojnie=2;
										najatie=0; 	
										DrawImage(0, 0, 240, 240,(uint16_t*) acc) ;   //  , DrawImage(0, 0, 240, 240,(uint16_t*) startuem)
					      	 ACC1;
									 IGN10;
									 IGN20;
									 PUSK0;
								}						   
				         if(sostojnie==2&&cepa==1&&najatie==1)
								 {
                   najatie=0;
									 sostojnie=0;
									// ClearScreen2(BLACK);
                //   DrawImage(28, 82, 46, 76,(uint16_t*) nol); 
								 }					

//             	if((sostojnie==9&&najatie==1)||(sostojnie==10&&najatie==1))   // IGN1 + IGN2
//								{
//									najatie=0;
//									sostojnie=1;
//									raz=0; 
//					      	 ACC1;
//									 IGN10;
//									 IGN20;
//									 PUSK0;
//								}

								 
			 	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////				
				if(cepa==0)
				{
			        	///////////////////				/// STARTUEM  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
		//   ========================== ZAVELASb  ============================================================							
  
					
										if( sostojnie!=9&&najatie==1)  // ||sostojnie!=8 
									{
										najatie=0;
										raz_zajig=0;
								        	sostojnie=10;  
									
									 DrawImage(0, 0, 240, 240,(uint16_t*) nezavod);

									} 			
 //   ==========================  END  ZAVELASb  ============================================================	

 
					} 
				

	  }		
					//  =====================  END NAJATIE  =============================== 



							 if(sostojnie==9)   
								{ 
									 ACC1;
									 IGN11;
									 IGN21;
									 PUSK0; 
								}
		
						 
									//================   STARTUEM !!!!!!!!!!!!!!!!!!!!!!!!!!  ===============================	  
								
							if(sostojnie==10||sostojnie==9) 
							{								
									 if(Counter<12000) tic_Counter++, delay(6); //  
							}
									 if(sec_start>6||tic_Counter>5) 
											 {
												// tic_Counter=0;
												if(sostojnie!=1)	sostojnie=9; 
												 if(raz_zajig==0) odinraz_smil();
											 }	
								
								
								if(sostojnie!=10)   sec_start=0 ;   
							 if(sostojnie==10)		
								 {    
						       ACC0;
									 IGN10;
									 IGN21;
									 PUSK1; 						     
								 }	
			 
								//  =====================  tushim vsex  ===============================    
									if(sostojnie==0||(sostojnie!=3&&lock==0)) 	uderj0;  
								 
			 	//   ==========   DEVKA NA PESOCHKE  ===============================================	  
						 if(sostojnie==20)
						 {
								if(pervyj_raz==5) DrawImage(0, 0, 240, 240,(uint16_t*)zaved);     
								if(pervyj_raz<11000) pervyj_raz++;
								if(pervyj_raz>=10000) sostojnie=1; 
						 }				 
			 	//   ==========  END DEVKA NA PESOCHKE  ===============================================		
						 
								//================  ACC 10 minut obratnyy otschet  ===============================a
											 
								 if(sostojnie==1&&raz==0) 
								 {
									 sekun=5;
									 minut=9;
									 to=0;
									 toch_tik=0;
									 odinraz(); 
								 } 	
									if(sostojnie==1&&raz==1)   
									{
										if(to!=buf_to)  tochki();
										buf_to=to;  
	                } 	 
								//================   END      ACC 10 minut obratnyy otschet  ===============================

					 

	
	
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 167;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UDERJ_Pin|LCD_RST_Pin|START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PODSVETKA_Pin|LCD_DC_Pin|IGN2_Pin|IGN1_Pin
                          |ACC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UDERJ_Pin START_Pin */
  GPIO_InitStruct.Pin = UDERJ_Pin|START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KNOP_Pin UNLOCK_Pin LOCK_Pin */
  GPIO_InitStruct.Pin = KNOP_Pin|UNLOCK_Pin|LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SCEPA_Pin NEJTRAL_Pin */
  GPIO_InitStruct.Pin = SCEPA_Pin|NEJTRAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PODSVETKA_Pin IGN2_Pin IGN1_Pin ACC_Pin */
  GPIO_InitStruct.Pin = PODSVETKA_Pin|IGN2_Pin|IGN1_Pin|ACC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM2)
//    {
//        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//        {
//            TIM2->CNT = 0;
//        
//            period = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
//         //   pulseWidth = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
//        }
//    }
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
