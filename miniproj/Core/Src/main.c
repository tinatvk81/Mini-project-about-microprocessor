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
#include "string.h"
#include "math.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define NUM_SAMPLES 10
int ms=0;
int sendMS=1000;
int count=0;
uint8_t keyboard_char;

int counter = 0;
unsigned char Rx_data[100];
uint8_t rxBuffer[1] = {0};


uint16_t note_frequency;
TIM_HandleTypeDef *buzzerPwmTimer;
uint32_t buzzerPwmChannel;
uint16_t v2=5;

uint16_t ldr_value = 0;
float lm35_value = 0.0;
int min_ldr=0;
int  max_ldr=4095;
int ldr_show=0;
int lm35_show=0;

int ldr_treshold;
int lm_treshold;

int kind;
int freqMin=900;
int freqMax;
int musicTime=3;
double PI=3.14;
int song=0;
int value;
double amplitude;
double sampling_rate = 100.0;
int temp=3;
double phase = 0.0;
double time_step;
int start=0;
int samplesldr[NUM_SAMPLES];
int sampcountldr=0;
int sampleslm[NUM_SAMPLES];
int sampcountlm=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//buzzerChangeTone(0, 0); for stop playing when a condition meets


int normalize(int value, int* samples) {
    int sum = 0;
    int i;

    for (i = NUM_SAMPLES - 1; i > 0; i--) {
        samples[i] = samples[i - 1];
        sum += samples[i];
    }
    samples[0] = value;
    sum += value;
    int avg = sum / NUM_SAMPLES;
    int variance = 0;
    for (i = 0; i < NUM_SAMPLES; i++) {
        variance += (samples[i] - avg) * (samples[i] - avg);
    }
    int stddev = sqrt(variance / NUM_SAMPLES);
    if (abs(value - avg) > stddev) {
        return avg;
    }
    return value;
}
void buzzerInit() {
    buzzerPwmTimer = &htim3;
    buzzerPwmChannel = TIM_CHANNEL_2;
    HAL_TIM_PWM_Start(buzzerPwmTimer, buzzerPwmChannel);
}

void buzzerChangeTone(uint16_t freq, uint16_t volume) {
    if (freq == 0 || freq > 20000) {
        __HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, 0);
    } else {
        const uint32_t internalClockFreq = HAL_RCC_GetSysClockFreq();
        const uint32_t prescaler = 1 + internalClockFreq / freq / 60000;
        const uint32_t timerClock = internalClockFreq / prescaler;
        const uint32_t periodCycles = timerClock / freq;
        const uint32_t pulseWidth = volume * periodCycles / 1000 / 2;

        buzzerPwmTimer->Instance->PSC = prescaler - 1;
        buzzerPwmTimer->Instance->ARR = periodCycles - 1;
        buzzerPwmTimer->Instance->EGR = TIM_EGR_UG;

        __HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, pulseWidth);
    }
}



void decoder_num(int i){
	if(i>=10)
		i=0;
	int x1=i&1;
	int x2=i&2;
	int x3=i&4;
	int x4=i&8;
	if(x1>0)
			x1=1;
	if(x2>0)
			x2=1;
	if(x3>0)
			x3=1;
	if(x4>0)
			x4=1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, x1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, x2);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, x3);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, x4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	 if(htim->Instance == TIM2){  // each 0.125 ms one interrupt
		 ms++;
		 if((ms%(8*sendMS))==0){
			 HAL_ADC_Start_IT(&hadc1);  //ldr
			 HAL_ADC_Start_IT(&hadc2);  //lm35
		 }

		 switch(ms%4){
		 			case 0:
		 				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,1);//d4
		 		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
		 		  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
		 		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);
		 		  		decoder_num(ldr_show%10);

		 		  		break;
		 			case 1:
		 				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 1);//d3
		 				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
		 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
		 				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);
		 				decoder_num(ldr_show/10);

		 				break;
		 			case 2:
		 				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
		 			 	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);
		 			 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //d2
		 			 	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
		 			 	decoder_num(lm35_show%10);

		 				break;
		 			case 3:
		 				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
		 				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
		 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
		 				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //d1
		 				decoder_num(lm35_show/10);

		 				break;
		 			}
		 if((ms%100)==0){
		 if(song==1){
			 if(count<(musicTime*100)){
				 count++;
			 }else {
				song=0;
				count=0;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 0);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0);
			}

		 }
		}
		 if(ms%60==0){
			 if(song==1){
				 buzzerChangeTone(note_frequency, v2);
			 }else{
				 buzzerChangeTone(0,0);
			 }

		 }

		if(ms%100==0){
			if(kind==1){
			if(start==1){
										 			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);
										 			 note_frequency=freqMin;
										 			 start=0;

										 		}
										 		else{
										 			if(temp%30==0){
										 				if(note_frequency==freqMax)
										 					note_frequency=freqMin;
										 				else {
										 					note_frequency=freqMax;
										 				}

										 			}
										 			temp++;

										 		}
			}else if(kind==3){  //sin
				 if(start==1){
					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
				 		note_frequency=freqMin;
				 		 start=0;
				 		}else {

				 			note_frequency=190 * sin(2*PI/3800*ms)+300;

				 		}


			 	}
		}

		if(ms%2000==0){
			 if(kind==2){  //triangle
								 if(start==1){
									 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 1);
									 note_frequency=freqMin;
									 start=0;
								 }else {
									 note_frequency+=100;
									 if(note_frequency>=freqMax)
											note_frequency=freqMin;

								}
							 	}

		}


	}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance==ADC1){
		ldr_value=HAL_ADC_GetValue(&hadc1);
		ldr_show=(ldr_value - min_ldr)*99 / (max_ldr - min_ldr);
		sampcountldr++;
		samplesldr[sampcountldr]=ldr_show;
		if(sampcountldr>NUM_SAMPLES){
			sampcountldr=0;
		}
		ldr_show=normalize(ldr_show, samplesldr);
		  uint8_t pack_ldr[8];
		  pack_ldr[0]=0xAA;
		  pack_ldr[1]=0x01;
		  pack_ldr[2]=0x02;
		  pack_ldr[3]=0x02;
		  uint8_t ldr_uint=(uint8_t)ldr_show;
		  int length=sizeof(ldr_uint);
		  pack_ldr[4]=0x21;
		  pack_ldr[5]=ldr_uint;
		  pack_ldr[7]=0x00;
		  HAL_UART_Transmit(&huart4, pack_ldr, sizeof(pack_ldr), HAL_MAX_DELAY);

	}
	if(hadc->Instance==ADC2){  //lm35
		lm35_value=HAL_ADC_GetValue(&hadc2);
		float Voltage_mv = lm35_value * 3300 / 1023;
		lm35_show = (Voltage_mv / 10);

		sampcountlm++;
				sampleslm[sampcountlm]=lm35_show;
				if(sampcountlm>NUM_SAMPLES){
					sampcountlm=0;
				}
				lm35_show=normalize(lm35_show, sampleslm);

		uint8_t pack_lm[8];
	    pack_lm[0]=0xAA;
	    pack_lm[1]=0x01;
	    pack_lm[2]=0x02;
	    pack_lm[3]=0x02;
	    uint8_t lm_uint=(uint8_t)lm35_show;
	    int length=sizeof(lm_uint);
	    pack_lm[4]=0x22;
	    pack_lm[5]=lm_uint;
		pack_lm[7]=0x00;
		HAL_UART_Transmit(&huart4, pack_lm, sizeof(pack_lm), HAL_MAX_DELAY);
//
//		char buffer[50];
//		sprintf(buffer, "%d\r\n", lm35_value); // Convert float to string
//		HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);

	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4){
//		if(note_frequency!=0){
//			    	buzzerChangeTone(note_frequency, v2);
//			    }

		Rx_data[counter] = rxBuffer[0];
			if (Rx_data[counter] == 0xAA) {
				Rx_data[counter]='\0';
				counter = 0;
				choosePacket();
				Rx_data[counter] = rxBuffer[0];
			} else {
				counter++;
			}
			HAL_UART_Receive_IT(&huart4, rxBuffer, 1);
	}

}

void choosePacket(){
	switch(Rx_data[3]){
	case 0x41:
		lm_treshold=(int)Rx_data[4];
		//buzer bips if lm>tre
//		if(lm_treshold>0){
//					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
//				}
//		if(lm35_show>0){
//							HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
//						}
		if(lm35_show>=lm_treshold){
			song=1;
			start=1;
			count=0;
		}

		break;
	case 0x42:
		ldr_treshold=(int)Rx_data[4];
//		if(ldr_treshold>0){
//					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
//		}
//		if(ldr_show>0){
//							HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
//				}
		if(ldr_show>=ldr_treshold){
			song=1;
			start=1;
			count=0;
		}
		break;
	case 0x11:
		switch(Rx_data[4]){
			case 0x31:kind=1;// square
				break;
			case 0x32:
				kind=2;// triangle
				break;
			case 0x33:
					kind=3;// sin
					break;

		}
		freqMin=(int)Rx_data[5];
		freqMax=(int)Rx_data[6];
		break;
	}
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  buzzerInit();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart4, rxBuffer, 1); // enable UART interrupt
 // HAL_UART_Receive_IT(&huart4, &keyboard_char,sizeof(keyboard_char)); // enable UART interrupt again

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_7|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD7_Pin|LD9_Pin|LD10_Pin
                          |LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE7 LD4_Pin LD3_Pin
                           LD5_Pin LD7_Pin LD9_Pin LD10_Pin
                           LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_7|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD7_Pin|LD9_Pin|LD10_Pin
                          |LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
