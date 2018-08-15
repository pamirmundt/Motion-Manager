
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "LCD.h"
#include "dwt_stm32_delay.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define encodingType 4.0f //Quadrature Encoding
#define encoderPulsePerRev 100.0f
#define RPMCalculationFreq 100.0f

#define refRpmLowerThreshold 0.0f
#define refRpmUpperThreshold 10000.0f
#define refRpmResolution 100.0f

#define voltageLowerThreshold 0.0f
#define voltageUpperThreshold 24.0f
#define voltageResolution 0.1f

#define pwmResolution 2048

#define debounceMillis 200

LCD * lcd;

/*******************************************************************************
    RPM Resolution per encoder tick
        (Encoder tick/sec) to RPM

                        60sec * (Frequency of RPM calculation timer)
    RPM_res_per_tick = ----------------------------------------------
                        Encoder Resolution * Encoder Reading Mode(x4)
*******************************************************************************/
#define RPM_res_per_tick (60.0f * RPMCalculationFreq) / (encoderPulsePerRev * encodingType)

// RPM Calculation
volatile int16_t prevEncoderCount = 0;
volatile float motorRPM = 0.0f;

//Moving Average for RPM smoothing
volatile float averagedMotorRpm = 0.0f;
volatile float alpha = 0.128f;

// Voltage Control
volatile float voltage = 0.0f;

// PI Control
volatile float Kp = 0.02f;
volatile float Ki = 0.01f;
volatile float integral = 0.0f;
volatile float integralMin = -2000.0f;
volatile float integralMax = 2000.0f;

volatile float refRpm = 0.0f;


//Flow Rate
volatile float flowRate = 0.0f;

//Display Selection
volatile float displayedRefRpm = 0.0f;
volatile float displayedVoltage = 0.0f;

// Rotary Knob
volatile uint8_t prevState = 0;
volatile int16_t cycleCount = 0;

//Debouncing
uint32_t currentMillis = 0;
uint32_t prevMillis = 0;

int pwm = 0;

const int cw[4] = {
			  2, // 10 -> 11
			  0, // 00 -> 10
			  3, // 11 -> 01
			  1  // 01 -> 00
			};

const int ccw[4] = {
			  1, // 01 -> 11
			  3, // 11 -> 10
			  0, // 00 -> 01
			  2  // 10 -> 00
			};

			
// Screen Menu Button Track - Select and Deselect
uint8_t selected = false;

// Underline of "Ref RPM"
uint8_t underlineHighlighted = false;
			
// Menu
enum menu{
	Logo,
	ControlSelection,
	RpmControl,
	VoltageControl,
	About
};

enum menu menuSelection = Logo;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void applyPWM(int16_t pwmValue){
	pwm = pwmValue;
	//Forward
	if(pwmValue > 0){
		if(pwmValue > pwmResolution)
			pwmValue = pwmResolution;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	//Backward
	else{
		if(pwmValue < -pwmResolution)
			pwmValue = -pwmResolution;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	}
	pwmValue = abs(pwmValue);
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwmValue);
}

// Increase Displayed Reference RPM
void incrementDisplayedRefRPM(void){
	if(displayedRefRpm < refRpmUpperThreshold)
		displayedRefRpm += refRpmResolution;
}

// Decrease Displayed Reference RPM
void decreaseDisplayedRefRPM(void){
	if(refRpmLowerThreshold < displayedRefRpm)
		displayedRefRpm -= refRpmResolution;
}

// Increase Displayed Voltage
void incrementDisplayedVoltage(void){
	if(displayedVoltage < voltageUpperThreshold)
		displayedVoltage += voltageResolution;
}

// Decrease Displayed Voltage
void decreaseDisplayedVoltage(void){
	if(voltageLowerThreshold < displayedVoltage)
		displayedVoltage -= voltageResolution;
}

//Count the Digits before .
uint8_t digitsInFloat(unsigned char * f){
	uint8_t i = 0;
	uint8_t digitCount = 0;
	while(f[i++] != '.'){
		digitCount++;
	}
	return digitCount;
}

//Refresh Numbers on the screen
void refreshScreen(void){
	//Clear Second Row - Fill with Spaces
	DisplayStringLeftAlligned(lcd,1,0, (unsigned char *)"                ", strlen("                "));
	//Clear Fourth Rows First Half - Fill with Spaces
	DisplayStringLeftAlligned(lcd,3,0, (unsigned char *)"        ", strlen("        "));

	unsigned char * array = malloc(16);
	
	if(menuSelection == RpmControl){
		if(selected == true)
			sprintf((char *)array, "%.1f", displayedRefRpm);
		else if(selected == false)
			sprintf((char *)array, "%.1f", refRpm);
	}
	else if(menuSelection == VoltageControl){
		if(selected == true){
			sprintf((char *)array, "%.1f", displayedVoltage);
		}
		else if(selected == false){
			sprintf((char *)array, "%.1f", voltage);
		}
	}
	
	uint8_t digitCount = digitsInFloat(array);

	// Display Reference RPM
	DisplayStringRightAlligned(lcd,1,6, array, digitsInFloat(array)+2);
	
	free(array);

	// RPM
	unsigned char * rpmStr = malloc(16); //16 digits max
	sprintf((char *)rpmStr, "%.1f", averagedMotorRpm);
	DisplayStringRightAlligned(lcd,1,15, rpmStr, digitsInFloat(rpmStr)+2);
	free(rpmStr);

	// Flow Rate
	unsigned char * flowRateStr = malloc(16);
	sprintf((char *)flowRateStr, "%.1f", flowRate);
	DisplayStringRightAlligned(lcd,3,7, flowRateStr, digitsInFloat(flowRateStr)+2);
	free(flowRateStr);
}

// Display underline of Ref RPM
void underlineHighlight(void){
	WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 29);
	WriteInstruction(lcd, 0x80);
	WriteRam(lcd, 0x7F);
	WriteRam(lcd, 0xFF);
	WriteRam(lcd, 0xFF);
	WriteRam(lcd, 0xFF);
	WriteRam(lcd, 0xFF);
	WriteRam(lcd, 0xFF);
	WriteRam(lcd, 0xFF);
}

// Hide underline of Ref RPM
void underlineLowlight(void){
	WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 29);
	WriteInstruction(lcd, 0x80);
	WriteRam(lcd, 0x00);
	WriteRam(lcd, 0x00);
	WriteRam(lcd, 0x00);
	WriteRam(lcd, 0x00);
	WriteRam(lcd, 0x00);
	WriteRam(lcd, 0x00);
	WriteRam(lcd, 0x00);
}

// Display Logo

// Display ControlSelection
uint8_t menuIndex = 0;

void menuControlSelection(void){
	menuSelection = ControlSelection;
	
	SetTextMode(lcd);
	ClearScreen(lcd);
	
	SetGraphicsMode(lcd);
	ClearGDRAM(lcd);
	
	for(uint8_t y = 0; y < 15; y++){
		for (uint8_t x = 0; x < 8; x++) {
			WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y);
			WriteInstruction(lcd, 0x80 | x);
			WriteRam(lcd, 0xFF);
			WriteRam(lcd, 0xFF);
		}
	}
		
	menuIndex = 1;
	HighlightMenuItem(lcd,menuIndex,true);
		
	SetTextMode(lcd);
	//ClearScreen(lcd);
	
	DisplayStringLeftAlligned(lcd,0,1, (unsigned char *)"MOTION MANAGER", strlen("MOTION MANAGER"));
	DisplayStringLeftAlligned(lcd,1,1, (unsigned char *)"RPM Control", strlen("RPM Control"));
	DisplayStringLeftAlligned(lcd,2,1, (unsigned char *)"Voltage Control", strlen("Voltage Control"));
	DisplayStringLeftAlligned(lcd,3,1, (unsigned char *)"About", strlen("About"));
}

void menuRpmControl(void){
	SetTextMode(lcd);
	ClearScreen(lcd);
	
	//TIM1 - Encoder
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	
	//TIM2 - Velocity Calculation Interrupt
	HAL_TIM_Base_Start_IT(&htim2);
	
	//TIM16 - PWM
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	
	// Topic Background
	SetGraphicsMode(lcd);
	ClearGDRAM(lcd);
	
	DivideHalfInverseT(lcd);
	HighlightTopLeftText(lcd);
	HighlightTopRightText(lcd);
	HighlightBottomText(lcd);
	
	// Topics
	SetTextMode(lcd);
	
	DisplayStringLeftAlligned(lcd,0,0, (unsigned char *)"Ref RPM", strlen("Ref RPM"));
	DisplayStringLeftAlligned(lcd,0,9, (unsigned char *)"RPM", strlen("RPM"));
	DisplayStringLeftAlligned(lcd,2,0, (unsigned char *)"Flow Rate", strlen("Flow Rate"));
	DisplayStringRightAlligned(lcd,3,14, (unsigned char *)"ml/min", strlen("ml/min"));
	
	//TIM17 - Screen Refresh Rate - 64Mhz / 64000 / 99 = 10Hz
	HAL_TIM_Base_Start_IT(&htim17);
}

void menuVoltageControl(void){
	SetTextMode(lcd);
	ClearScreen(lcd);
	
	//TIM1 - Encoder
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	
	//TIM2 - Velocity Calculation Interrupt
	HAL_TIM_Base_Start_IT(&htim2);
	
	//TIM16 - PWM
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	
	// Topic Background
	SetGraphicsMode(lcd);
	ClearGDRAM(lcd);
	
	DivideHalfInverseT(lcd);
	HighlightTopLeftText(lcd);
	HighlightTopRightText(lcd);
	HighlightBottomText(lcd);
	
	// Topics
	SetTextMode(lcd);
	
	DisplayStringLeftAlligned(lcd,0,0, (unsigned char *)"Voltage", strlen("Voltage"));
	DisplayStringLeftAlligned(lcd,0,9, (unsigned char *)"RPM", strlen("RPM"));
	DisplayStringLeftAlligned(lcd,2,0, (unsigned char *)"Flow Rate", strlen("Flow Rate"));
	DisplayStringRightAlligned(lcd,3,14, (unsigned char *)"ml/min", strlen("ml/min"));
	
	//TIM17 - Screen Refresh Rate - 64Mhz / 64000 / 99 = 10Hz
	HAL_TIM_Base_Start_IT(&htim17);
}

void menuAbout(){
	SetTextMode(lcd);
	ClearScreen(lcd);
	
	SetGraphicsMode(lcd);
	ClearGDRAM(lcd);
	
	SetTextMode(lcd);
	DisplayStringLeftAlligned(lcd,0,0, (unsigned char *)"info@", strlen("info@"));
	DisplayStringLeftAlligned(lcd,1,0, (unsigned char *)"kopernikrobotics", strlen("KopernikRobotics"));
	DisplayStringRightAlligned(lcd,2,15, (unsigned char *)".com", strlen(".com"));
	DisplayStringLeftAlligned(lcd,3,0, (unsigned char *)"v1.0.0", strlen("v1.0.0"));
}

void setVoltage(){
	pwm = (((float)voltage)/((float)voltageUpperThreshold)) * ((float)pwmResolution);
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (int)pwm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	DWT_Delay_Init();
	
	//TIM1 - Encoder
	//HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	
	//TIM16 - PWM
	//HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	

	// LCD	
	// For a minimal system with only one ST7920 and one MPU, only SCLK and SID pins are necessary.
	// CS pin should pull to high.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
	lcd = (struct LCD *)malloc(sizeof(struct LCD));

	InitLcdSPI(lcd, &hspi1);
	
	InitDisplay(lcd);

	menuControlSelection();
	
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		//HAL_Delay(100);
		
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 63999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 799;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2047;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 63999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 199;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_Pin */
  GPIO_InitStruct.Pin = RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	// Motor Encoder
	if(htim->Instance == TIM2){
		//RPM Calculation
		int16_t encoderCount = htim1.Instance->CNT;
		int16_t deltaEncoderCount = encoderCount - prevEncoderCount;
		motorRPM = RPM_res_per_tick * (float)deltaEncoderCount;
		prevEncoderCount = encoderCount;
		
		//A little Moving Average for RPM since resolution is 15RPM
		averagedMotorRpm = (alpha * motorRPM) + (1 - alpha) * averagedMotorRpm;
		
		if(menuSelection == RpmControl){
			//Simple PI Control Implementation
			float rpmError = refRpm - averagedMotorRpm;
			
			//Compute Integral
			integral = integral + Ki * rpmError;
			
			//Compare Integral
			if(integral > integralMax)
				integral = integralMax;
			else if(integral < integralMin)
				integral = integralMin;
			
			//Compute PI
			float out = Kp * rpmError + integral;
			applyPWM((int16_t)out);
		}
	}
	
	if(htim->Instance == TIM15){
		
		SetGraphicsMode(lcd);
		
		underlineHighlight();
		
		if(underlineHighlighted == false){
			underlineHighlight();
			underlineHighlighted = true;
		}
		else{
			underlineLowlight();
			underlineHighlighted = false;
		}
		
		SetTextMode(lcd);
	}

	if(htim->Instance == TIM17){
		refreshScreen();
	}
	
}

/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM17){
		refreshScreen();
	}
}
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	// https://github.com/micheljansen/arduino-rotary-knob
	// Knob Encoder - PB4 PB5
	if((selected == true) || (menuSelection == ControlSelection)){ // If "Set Ref RPM" seleccted
		if((GPIO_Pin == GPIO_PIN_4) || (GPIO_Pin == GPIO_PIN_5)){
			uint8_t a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
			uint8_t b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
			
			
			uint8_t state = (a << 1 | b);
			
			if(state != prevState){
				if(state == cw[prevState]) {
					// going forward
					cycleCount++;

				}
				else if(state == ccw[prevState]) {
					// going backward
					cycleCount--;
				}
				
				if(state == 3) {
					if(cycleCount > 0) {
						//HAL_UART_Transmit(&huart2, (uint8_t *)"+++++\r\n", 3, HAL_MAX_DELAY);
						if(menuSelection == RpmControl)
							incrementDisplayedRefRPM();
						
						else if(menuSelection == VoltageControl)
							incrementDisplayedVoltage();
						
						else if(menuSelection == ControlSelection)
						{
							SetGraphicsMode(lcd);
							menuIndex++;
							menuIndex = (menuIndex%4);
							
							if(menuIndex == 0)
									menuIndex++;
							
							if(menuIndex == 1)
								HighlightMenuItem(lcd, 3, false);
							else
								HighlightMenuItem(lcd, menuIndex - 1, false);
							HighlightMenuItem(lcd, menuIndex, true);
							//SetTextMode(lcd);
						}
						//count++;
						//direction = 1;
					}
					else if(cycleCount < 0) {
						if(menuSelection == RpmControl)
							decreaseDisplayedRefRPM();
						
						else if(menuSelection == VoltageControl)
							decreaseDisplayedVoltage();
						
						else if(menuSelection == ControlSelection)
						{
							SetGraphicsMode(lcd);
							menuIndex--;
							menuIndex = (menuIndex%4);
							
							if(menuIndex == 0)
									menuIndex = 3;
							
							if(menuIndex == 3)
								HighlightMenuItem(lcd, 1, false);
							else
								HighlightMenuItem(lcd, menuIndex + 1, false);
							HighlightMenuItem(lcd, menuIndex, true);
							//SetTextMode(lcd);
						}
						//count--;
						//direction = -1;
					}
					cycleCount = 0;
				}
				
				prevState = state;
			}
		}
	}
		
	// Knob Select Button - PA11
	currentMillis = HAL_GetTick();
	if(currentMillis - prevMillis > debounceMillis){
		if(GPIO_Pin == GPIO_PIN_11){
			if(menuSelection == ControlSelection){
				switch (menuIndex){
					case 1:	//RPM Control
						menuSelection = RpmControl;
						menuRpmControl();
						break;
					case 2: //Voltage Control
						menuSelection = VoltageControl;
						menuVoltageControl();
						break;
					case 3: //About
						menuSelection = About;
						menuAbout();
						break;
				}
			}
			
			else if(menuSelection == RpmControl){
				// "Set Ref RPM" Selected
				// First time selected (to set new rpm)
				if(selected == false){
					selected = true;
					displayedRefRpm = refRpm;
					
					// Start Highlighting
					HAL_TIM_Base_Start_IT(&htim15);
				}
				
				// Second time selected (new rpm set)
				else if (selected == true){
					selected = false;
					refRpm = displayedRefRpm;
					
					// Stop Highlighting
					HAL_TIM_Base_Stop_IT(&htim15);
					SetGraphicsMode(lcd);
					underlineHighlighted = false;
					underlineLowlight();
					SetTextMode(lcd);
				}
			}
			
			else if(menuSelection == VoltageControl){
				// "Set Voltage" Selected
				// First time selected (to set voltge)
				if(selected == false){
					selected = true;
					displayedVoltage = voltage;
					
					//Start Highlighting
					HAL_TIM_Base_Start_IT(&htim15);
				}
				else if(selected == true){
					selected = false;
					voltage = displayedVoltage;
					setVoltage();
					
					//Stop Highlihting
					HAL_TIM_Base_Stop_IT(&htim15);
					SetGraphicsMode(lcd);
					underlineHighlighted = false;
					underlineLowlight();
					SetTextMode(lcd);
				}
			}
			
			else if (menuSelection == About){
				__NOP; //Knob does not do anything
			}
		}
	
		// Stop/Back Button - PF1
		if(GPIO_Pin == GPIO_PIN_1){
			if(menuSelection == ControlSelection){
				__NOP;
			}
			else if((menuSelection == RpmControl) || (menuSelection == VoltageControl)){
				if(selected == true){
					selected = false;
					HAL_TIM_Base_Stop_IT(&htim15);
					
					// Exit selection
					if(underlineHighlighted == true){
						SetGraphicsMode(lcd);
						underlineHighlighted = false;
						underlineLowlight();
						SetTextMode(lcd);
					}
				}
				else if (selected == false){
					//Stop RPM Control Loop
					HAL_TIM_Base_Stop_IT(&htim2);
					//Stop Refreshing Page
					HAL_TIM_Base_Stop_IT(&htim17);
					menuSelection = ControlSelection;
					menuControlSelection();
					
					// Zero Everthing
					voltage = 0;
					displayedVoltage = 0;
					refRpm = 0;
					displayedRefRpm = 0;
					motorRPM = 0;
					averagedMotorRpm = 0;
					pwm = 0;
					integral = 0;
					prevEncoderCount = 0;
					htim1.Instance->CNT = 0;	//Zero Encoder Count
					
					//TIM1 - Encoder
					HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
					
					//TIM2 - Velocity Calculation Interrupt
					HAL_TIM_Base_Stop_IT(&htim2);
					
					// Set PWM to 0
					__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
					
					//TIM16 - PWM
					HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
				}
			}
			else if(menuSelection == About){
				menuSelection = ControlSelection;
				menuControlSelection();
			}
		}
	}
	prevMillis = currentMillis;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
