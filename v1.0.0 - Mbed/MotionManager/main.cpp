#include "mbed.h"
#include "main.h"
#include "LCD.h"
#include "SDFileSystem.h"
#include "logo.h"

// DEBUG
//Serial pc(SERIAL_TX, SERIAL_RX);

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

// mosi, miso, sclk, cs
SDFileSystem sd(PA_7, PA_6, PA_5, PA_3, "sd");

/* Private variables ---------------------------------------------------------*/
#define encodingType 4.0f //Quadrature Encoding
#define RPMCalculationFreq 100.0f

#define pwmResolution 2048

#define debounceMillis 400

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
volatile float Kd = 0.0f;
volatile float integral = 0.0f;
volatile float integralMin = -2048.0f;
volatile float integralMax = 2048.0f;

volatile float refRpm = 0.0f;

// Encoder
float encoderPulsePerRev = 100.0f;

// Voltage
float voltageLowerThreshold = 0.0f;
float voltageUpperThreshold = 24.0f;
float voltageResolution = 0.1f;

// RPM
float refRpmLowerThreshold = 0.0f;
float refRpmUpperThreshold = 10000.0f;
float refRpmResolution = 100.0f;

//Flow Rate
//volatile float flowRate = 0.0f;
// Flow Rate versus Voltage
// Quadratic Function Parameters
// FR = f(V) = A*V^2 + B*V + C
float A = 0.0f, B = 0.0f, C = 0.0f;

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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM15_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

enum menu menuSelection = Logo;

void applyPWM(int16_t pwmValue){
    //Forward
    if(pwmValue > 0){
        if(pwmValue > pwmResolution){
            pwmValue = pwmResolution;
            pwm = pwmValue;
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }
    //Backward
    else{
        if(pwmValue < -pwmResolution){
            pwmValue = -pwmResolution;
            pwm = pwmValue;
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }
    pwmValue = abs(pwmValue);
    pwm = pwmValue;
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

//Calculate Flow Rate from Voltage - Quadratic Function
float calculateFlowRate(float _volt){
    float flowRate = (A * _volt * _volt) + (B * _volt) + C;
    return flowRate;
}

//Refresh Numbers on the screen
void refreshScreen(void){
    //Clear Second Row - Fill with Spaces
    DisplayStringLeftAlligned(lcd,1,0, (unsigned char *)"                ", strlen("                "));
    //Clear Fourth Rows First Half - Fill with Spaces
    DisplayStringLeftAlligned(lcd,3,0, (unsigned char *)"        ", strlen("        "));

    unsigned char * array = (unsigned char *)malloc(16);
    
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
    unsigned char * rpmStr = (unsigned char *)malloc(16); //16 digits max
    sprintf((char *)rpmStr, "%.1f", averagedMotorRpm);
    DisplayStringRightAlligned(lcd,1,15, rpmStr, digitsInFloat(rpmStr)+2);
    free(rpmStr);
    
    //Calculate Flow Rate
    float flowRate =0.0f;
    if(menuSelection == RpmControl)
        flowRate = calculateFlowRate(((float)pwm / ((float)pwmResolution)) * voltageUpperThreshold);
    else if(menuSelection == VoltageControl)
        flowRate = calculateFlowRate(voltage);
        
    // Flow Rate
    unsigned char * flowRateStr = (unsigned char *)malloc(16);
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
    
    //TIM3 - Velocity Calculation Interrupt
    HAL_TIM_Base_Start_IT(&htim3);
    
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
    
    //TIM3 - Velocity Calculation Interrupt
    HAL_TIM_Base_Start_IT(&htim3);
    
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

/* MAIN ***********************************************************************/

int main() {
    HAL_Init();
    
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM16_Init();
    MX_TIM3_Init();
    MX_TIM17_Init();
    MX_TIM15_Init();
    
    
    
    // Init SPI for LCD
    MX_SPI1_Init();
    
    // LCD  
    // For a minimal system with only one ST7920 and one MPU, only SCLK and SID pins are necessary.
    // CS pin should pull to high.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    
    lcd = (struct LCD *)malloc(sizeof(struct LCD));

    InitLcdSPI(lcd, &hspi1);
    
    InitDisplay(lcd);
    
    
    // Display Logo
    SetGraphicsMode(lcd);
    
    FillGDRAM(lcd, kopernik_pixel_logo);
    
    wait(2);
    
    ClearGDRAM(lcd);
    SetTextMode(lcd);
    
    
    // Deselect LCD
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    
    /* SD Card Begin End ******************************************************/

    /* Read Quadratic Function Coefficients  **********************************/
    FILE *fp = fopen("/sd/motionManager/flowRateVersusVoltage.csv", "r");
    
    // No folder
    if(fp == NULL) {
        //error("Could not open file for write\n");
    }
    else{
        // Quadratic function for "Flow Rate vs Voltage"
        if(fscanf(fp, "%f,%f,%f\n", &A, &B, &C) == 3){
            //printf("%f,%f,%f\n", A, B, C);
        }
        else
            A = B = C = 0.0f;
            
        fclose(fp);
    }
    
    /* Read PID Constants *****************************************************/
    fp = fopen("/sd/motionManager/PID.csv", "r");
    
    // No folder
    if(fp == NULL) {
        //error("Could not open file for write\n");
    }
    else{
        // PID Constants
        if(fscanf(fp, "%f,%f,%f,%f,%f,%f\n", &Kp, &Ki, &Kd, &integralMin, &integralMax,&alpha) == 6){
            //printf("%f,%f,%f,%f,%f,%f\n", Kp, Ki, Kd, integralMin, integralMax,alpha);
        }
        fclose(fp);
    }
    
    /* Read Voltage Settings **************************************************/
    fp = fopen("/sd/motionManager/voltage.csv", "r");
    
    // No folder
    if(fp == NULL) {
        //error("Could not open file for write\n");
    }
    else{
        // PID Constants
        if(fscanf(fp, "%f,%f\n", &voltageResolution, &voltageUpperThreshold) == 2){
            //printf("%f,%f\n", voltageResolution, voltageUpperThreshold);
        }
        fclose(fp);
    }
    
    /* Read RPM Settings ******************************************************/
    
    fp = fopen("/sd/motionManager/RPM.csv", "r");
    
    // No folder
    if(fp == NULL) {
        //error("Could not open file for write\n");
    }
    else{
        // PID Constants
        if(fscanf(fp, "%f,%f\n", &refRpmResolution, &refRpmUpperThreshold) == 2){
            //printf("%f,%f\n", refRpmResolution, refRpmUpperThreshold);
        }
        fclose(fp);
    }
    
    /* Read Encoder Settings **************************************************/
    
    fp = fopen("/sd/motionManager/encoder.csv", "r");
    
    // No folder
    if(fp == NULL) {
        //error("Could not open file for write\n");
    }
    else{
        // PID Constants
        if(fscanf(fp, "%f\n", &encoderPulsePerRev) == 1){
            //printf("%f\n", encoderPulsePerRev);
        }
        fclose(fp);
    }    
    
    delete &sd;
    
    /* SD Card Read End *******************************************************/
    
    MX_SPI1_Init();
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    
    InitDisplay(lcd);
    
    SetTextMode(lcd);
    
    menuControlSelection();
    
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    
    while(1) {
        // Empty
    }
}

/******************************************************************************/

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
    if(htim->Instance == TIM3){        
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
                    case 1: //RPM Control
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
                    HAL_TIM_Base_Stop_IT(&htim3);
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
                    htim1.Instance->CNT = 0;    //Zero Encoder Count
                    
                    //TIM1 - Encoder
                    HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
                    
                    //TIM3 - Velocity Calculation Interrupt
                    HAL_TIM_Base_Stop_IT(&htim3);
                    
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
        prevMillis = currentMillis;
    }
    
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
