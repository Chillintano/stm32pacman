/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 19/05/2015 23:04:15
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stm32f429i_discovery.h"
#include "..\Components\ili9341\ili9341.h" 
#include "..\Components\stmpe811\stmpe811.h"
#include "stm32f429i_discovery_ts.h"
#include "stm32f429i_discovery_io.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stm32f429i_discovery_sdram.h"
#include "arm_math.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
typedef struct pos{
	int X;
	int Y;
} pos_t;

static unsigned short map[] = {
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,
	1,0,1,1,0,1,1,1,0,1,0,1,1,1,0,1,1,0,1,
	1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
	1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,0,1,
	1,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,1,
	1,1,1,1,0,1,1,1,0,1,0,1,1,1,0,1,1,1,1,
	0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,
	1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,
	1,1,1,1,0,0,0,1,0,0,0,1,0,0,0,1,1,1,1,
	1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,
	0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,
	1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,
	1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,
	1,0,1,1,0,1,1,1,0,1,0,1,1,1,0,1,1,0,1,
	1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,
	1,1,0,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,1,
	1,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,1,
	1,0,1,1,1,1,1,1,0,1,0,1,1,1,1,1,1,0,1,
	1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
};

static unsigned short food1_base[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,
	0,1,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,1,0,
	0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
	0,1,0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,1,0,
	0,1,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,1,0,
	0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,
	0,0,0,0,1,0,1,1,1,0,1,1,1,0,1,0,0,0,0,
	0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,
	0,0,0,0,1,1,1,0,0,0,0,0,1,1,1,0,0,0,0,
	0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,
	0,0,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,0,
	0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,
	0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,
	0,1,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,1,0,
	0,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,
	0,0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0,0,
	0,1,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,1,0,
	0,1,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,
	0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static unsigned short food_base[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

int temp_map[19*21]={
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static unsigned short food[19*21];


LTDC_HandleTypeDef hltdc;
RNG_HandleTypeDef rng;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
uint8_t status = 0;
WWDG_HandleTypeDef hwwdg;
short nextMove=0;
osThreadId defaultTaskHandle;
int points=0;
int totalFood=0;
unsigned short startXpos=9, startYpos=7;
pos_t playerPosition;
int lives=0;
int initialLives=3;
unsigned short startGhostXpos=9, startGhostYpos=19;
pos_t ghostPosition;
unsigned short SettingLowerTrigger=250, SettingUpperTrigger=70;
		short mode=1;
		short switchedMode=0;
		short controlType=1;
		TS_StateTypeDef  TS_State;
/* USER CODE BEGIN PV */
char text[15];
float Buffer[3];
float Xval=0;
float Yval=0;
float Zval = 0;
float angleX=0;
	float angleY=0;
	float angleZ=0;
	float X_BiasError,Y_BiasError,Z_BiasError;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_WWDG_Init(void);
void StartDefaultTask(void const * argument);
void checkForTouch();
void updateMEMS();
void gameLoop();
void getControl_TS();
void getTSData();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void blinkLed(){
	for(;;){
		BSP_LED_Toggle(LED3);
		osDelay(100);
	}
}

void Gyro_SimpleCalibration(float* GyroData)
{
  uint32_t BiasErrorSplNbr = 1000;
	
  int i = 0;
  X_BiasError = 0;
  Y_BiasError = 0;
  Z_BiasError = 0;
  for (i = 0; i < BiasErrorSplNbr; i++)
  {
		BSP_GYRO_GetXYZ(GyroData);
    X_BiasError += GyroData[0];
    Y_BiasError += GyroData[1];
    Z_BiasError += GyroData[2];
  }
  /* Set bias errors */
  X_BiasError /= BiasErrorSplNbr;
  Y_BiasError /= BiasErrorSplNbr;
  Z_BiasError /= BiasErrorSplNbr;
  
  /* Get offset value on X, Y and Z */
  GyroData[0] = X_BiasError;
  GyroData[1] = Y_BiasError;
  GyroData[2] = Z_BiasError;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LTDC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_WWDG_Init();
	BSP_LED_Init(LED3);
	BSP_LCD_Init();
	__HAL_RCC_RNG_CLK_ENABLE();
	
	if (HAL_RNG_Init(&rng)==HAL_OK) BSP_LED_On(LED3);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);
	/* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);
  
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  
  /* Clear the LCD */ 
  
  status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK); 
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	if (status==TS_OK){
		BSP_LCD_Clear(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(180,310,(uint8_t*)"TS OK",LEFT_MODE);
		BSP_LCD_SetFont(&Font24);
	}
	else{
		BSP_LCD_Clear(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(180,310,(uint8_t*)"TS NOT OK",LEFT_MODE);
		BSP_LCD_SetFont(&Font24);
	}
	
	if (BSP_GYRO_Init() != GYRO_OK)
  {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(100, BSP_LCD_GetYSize()- 80, (uint8_t*)"GYRO NOT OK", LEFT_MODE);
		BSP_LCD_SetFont(&Font24);
  }
	else{
		BSP_GYRO_Reset();
			osThreadDef(gyro,updateMEMS, osPriorityNormal,0,128);
			osThreadId gyroHandle = osThreadCreate(osThread(gyro),NULL);
	
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAt(100, 310, (uint8_t*)"GYRO OK", LEFT_MODE);
			BSP_LCD_SetFont(&Font24);
	}
	
	//Gyro_SimpleCalibration(Buffer);
	
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

//osThreadDef(blink, blinkLed, osPriorityNormal, 0, 128);
  //osThreadId blinkTaskHandle = osThreadCreate(osThread(blink), NULL);
	
	osThreadDef(touch,getControl_TS, osPriorityNormal,0,128);
	osThreadId touchHandle = osThreadCreate(osThread(touch),NULL);
	
	osThreadDef(pollTS,getTSData, osPriorityNormal,0,128);
	osThreadId pollTSHandle = osThreadCreate(osThread(pollTS),NULL);

	osThreadDef(game,gameLoop, osPriorityNormal,0,128);
	osThreadId gameHandle = osThreadCreate(osThread(game),NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 432;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

}

/* LTDC init function */
void MX_LTDC_Init(void)
{

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  HAL_LTDC_Init(&hltdc);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/* WWDG init function */
void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  HAL_WWDG_Init(&hwwdg);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PF0   ------> FMC_A0
     PF1   ------> FMC_A1
     PF2   ------> FMC_A2
     PF3   ------> FMC_A3
     PF4   ------> FMC_A4
     PF5   ------> FMC_A5
     PF7   ------> SPI5_SCK
     PF8   ------> SPI5_MISO
     PF9   ------> SPI5_MOSI
     PC0   ------> FMC_SDNWE
     PF11   ------> FMC_SDNRAS
     PF12   ------> FMC_A6
     PF13   ------> FMC_A7
     PF14   ------> FMC_A8
     PF15   ------> FMC_A9
     PG0   ------> FMC_A10
     PG1   ------> FMC_A11
     PE7   ------> FMC_D4_DA4
     PE8   ------> FMC_D5_DA5
     PE9   ------> FMC_D6_DA6
     PE10   ------> FMC_D7_DA7
     PE11   ------> FMC_D8_DA8
     PE12   ------> FMC_D9_DA9
     PE13   ------> FMC_D10_DA10
     PE14   ------> FMC_D11_DA11
     PE15   ------> FMC_D12_DA12
     PB12   ------> USB_OTG_HS_ID
     PB13   ------> USB_OTG_HS_VBUS
     PB14   ------> USB_OTG_HS_DM
     PB15   ------> USB_OTG_HS_DP
     PD8   ------> FMC_D13_DA13
     PD9   ------> FMC_D14_DA14
     PD10   ------> FMC_D15_DA15
     PD14   ------> FMC_D0_DA0
     PD15   ------> FMC_D1_DA1
     PG4   ------> FMC_A14_BA0
     PG5   ------> FMC_A15_BA1
     PG8   ------> FMC_SDCLK
     PC9   ------> I2C3_SDA
     PA8   ------> I2C3_SCL
     PD0   ------> FMC_D2_DA2
     PD1   ------> FMC_D3_DA3
     PG15   ------> FMC_SDNCAS
     PB5   ------> FMC_SDCKE1
     PB6   ------> FMC_SDNE1
     PE0   ------> FMC_NBL0
     PE1   ------> FMC_NBL1
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct, GPIO_InitStr;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PF0 PF1 PF2 PF3 
                           PF4 PF5 PF11 PF12 
                           PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStr.Pin = GPIO_PIN_0;
  GPIO_InitStr.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStr.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStr);
	HAL_NVIC_SetPriority(EXTI0_IRQn,0,0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG4 PG5 
                           PG8 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10 
                           PE11 PE12 PE13 PE14 
                           PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD14 
                           PD15 PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

static void WaitForPressedState(uint8_t Pressed) 
{
  
  do 
  {
    //BSP_TS_GetState(&TS_State);
    osDelay(10);
    if (TS_State.TouchDetected == Pressed) 
    {
      uint16_t TimeStart = HAL_GetTick();
      do {
        //BSP_TS_GetState(&TS_State);
        osDelay(10);
        if (TS_State.TouchDetected != Pressed) 
        {
          break;
        } else if ((HAL_GetTick() - 50) > TimeStart) 
        {
          return;
        }
      } while (1);
    }
		if (switchedMode) return;
  } while (1);
}

void getTSData(){
	for(;;){
		BSP_TS_GetState(&TS_State);
		osDelay(10);
	}
}

void getControl_TS(){
	for(;;){
		if (controlType==1){
			if (TS_State.TouchDetected==1){
				if (TS_State.Y<SettingUpperTrigger) {nextMove=4;}
				else if (TS_State.Y>SettingLowerTrigger) {nextMove=3;}
				else if (TS_State.X<120) {nextMove=1;}
				else {nextMove=2;}
			}
		}	
		osDelay(10);
	}
}

void updateMEMS(){
	for(;;){
		BSP_GYRO_GetXYZ(Buffer);
		Xval = (Buffer[0])-X_BiasError+1000;
    Yval = (Buffer[1])-Y_BiasError; 
    Zval = (Buffer[2])-Z_BiasError; 
		if (Xval>1000 || Xval<-1000)	angleX+=0.00875*0.01*Xval/8.65;
		if (Yval>1000 || Yval<-1000)	angleY+=0.00875*0.01*Yval/8.65;
		if (Zval>1000 || Zval<-1000)	angleZ+=0.00875*0.01*Zval/8.65;
		if (controlType==0){
			if (angleX<-13){nextMove=4;}
			if (angleX>13){nextMove=3;}
			if (angleY>13){nextMove=2;}
			if (angleY<-13){nextMove=1;}
		}
		osDelay(10);
	}
}

void drawBaseMap(){
	for(int i=0;i<21;i++){
		for(int j=0;j<19;j++){
			if (map[i*19+j]==1)	{BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE); BSP_LCD_FillRect((j*12)+5,i*12+30,12,12);}
			else if (food_base[i*19+j]==1) {BSP_LCD_SetTextColor(LCD_COLOR_RED); BSP_LCD_FillCircle((j*12)+10,i*12+36,2); }
		}
	}
}

void drawMap(){
	for(int i=0;i<21;i++){
		for(int j=0;j<19;j++){
			if (map[i*19+j]==1)	{BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE); BSP_LCD_FillRect((j*12)+5,i*12+30,12,12);}
			else if (food[i*19+j]==1) {BSP_LCD_SetTextColor(LCD_COLOR_RED); BSP_LCD_FillCircle((j*12)+10,i*12+36,2); }
		}
	}
}

void softResetGame(){
	//set initial position
	playerPosition.X = startXpos;
	playerPosition.Y = startYpos;
	ghostPosition.X = startGhostXpos;
	ghostPosition.Y = startGhostYpos;
	//draw map and player/ghost circles
	drawMap();
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36),5);
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36),5);
	//display lives and points
	sprintf(text,"Lives: %d",lives);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(235,295,(uint8_t*)text,RIGHT_MODE);
	sprintf(text,"Points: %d",points);
	BSP_LCD_DisplayStringAt(10,10,(uint8_t*)text,LEFT_MODE);
	
}

void ResetGame(){
	for(int i=0;i<19*21;i++){
		food[i]=food_base[i];
	}	
	//count the amount of food
	for(int i=0;i<19*21;i++){
		if (food[i]==1) totalFood++;
	}
	softResetGame();
}

void fullReset(){
	lives=initialLives;
	//verify that no food is on walls
	for(int i=0;i<19*21;i++){
		if (map[i]==1) food_base[i]=0;
	}
	//reset points
	points=0;
	ResetGame();
}


void mark(int* pathmap){
	int value=1;
	pathmap[ghostPosition.Y*19+ghostPosition.X]=1;
	int changes=1;
	while (changes>0){
		changes=0;
		for(int j=0;j<19;j++){
			for(int i=0;i<21;i++){
				if (pathmap[i*19+j]==value){
					if (map[i*19+j-1]==0 && pathmap[i*19+j-1]==0) {pathmap[i*19+j-1]=value+1; changes++;}
					if (map[i*19+j+1]==0 && pathmap[i*19+j+1]==0) {pathmap[i*19+j+1]=value+1; changes++;}
					if (map[i*19+j-19]==0 && pathmap[i*19+j-19]==0) {pathmap[i*19+j-19]=value+1; changes++;}
					if (map[i*19+j+19]==0 && pathmap[i*19+j+19]==0) {pathmap[i*19+j+19]=value+1; changes++;}
				}
			}
		}
		value++;
	}
}

int proceedMovements(){
	unsigned short nextGhostMove=0;
	unsigned short nextPlayerMove=0;
	
	//pathfinding
	//prepare data storage for wave algorithm
	for(int j=0;j<19;j++){
		for(int i=0;i<21;i++){
			temp_map[i*19+j]=0;
		}
	}
	//run wave algorithm
	mark(temp_map);
	//walk from destination to source to get next step
	short nextX=playerPosition.X,nextY=playerPosition.Y;
	for(int i=temp_map[playerPosition.Y*19+playerPosition.X];i>2;i--){
		if (temp_map[nextY*19+nextX-1]==i-1) {nextX--;}
		if (temp_map[nextY*19+nextX+1]==i-1) {nextX++;}
		if (temp_map[nextY*19+nextX-19]==i-1) {nextY--;}
		if (temp_map[nextY*19+nextX+19]==i-1) {nextY++;}
	}
	//get the "code" of next step
	if (nextX<ghostPosition.X) {nextGhostMove=1;}
	if (nextX>ghostPosition.X) {nextGhostMove=2;}
	if (nextY<ghostPosition.Y) {nextGhostMove=3;}
	if (nextY>ghostPosition.Y) {nextGhostMove=4;}
	if ((temp_map[nextY*19+nextX-1]+temp_map[nextY*19+nextX+1]+temp_map[nextY*19+nextX-19]+temp_map[nextY*19+nextX+19])==0) {nextGhostMove=0;}
	//determine, if player's next move is possible
	if (nextMove==1 && map[playerPosition.Y*19+playerPosition.X-1]==0 && playerPosition.X-1>=0){
		nextPlayerMove=1;
	}
	else if (nextMove==2 && map[playerPosition.Y*19+playerPosition.X+1]==0 && playerPosition.X+1<=19){
		nextPlayerMove=2;
	}
	else if (nextMove==3 && map[(playerPosition.Y+1)*19+playerPosition.X]==0 && playerPosition.Y+1<=21){
		nextPlayerMove=3;
	}
	else if (nextMove==4 && map[(playerPosition.Y-1)*19+playerPosition.X]==0 && playerPosition.Y-1>=0){
		nextPlayerMove=4;
	}
	
	//determine, if ghost and player will collide
	short drawFrames;
	if ((ghostPosition.X==playerPosition.X-1 && ghostPosition.Y==playerPosition.Y && (nextPlayerMove==1 || nextPlayerMove==0) && nextGhostMove==2)||(ghostPosition.X==playerPosition.X+1 && ghostPosition.Y==playerPosition.Y && (nextPlayerMove==2 || nextPlayerMove==0) && nextGhostMove==1)||
		(ghostPosition.Y==playerPosition.Y+1 &&  ghostPosition.X==playerPosition.X && (nextPlayerMove==3 || nextPlayerMove==0) && nextGhostMove==3)||(ghostPosition.Y==playerPosition.Y-1 && ghostPosition.X==playerPosition.X && (nextPlayerMove==4 || nextPlayerMove==0) && nextGhostMove==4)|| (ghostPosition.Y==playerPosition.Y && ghostPosition.X==playerPosition.X)){
		drawFrames=6;
	}
	else drawFrames=12;
	
	//determine, if there is a food piece in player's destination and remove it, if player eats it - remove from grid and award points
	if (nextPlayerMove==1 && food[playerPosition.Y*19+playerPosition.X-1]==1 && drawFrames==12){
		food[playerPosition.Y*19+playerPosition.X-1]=0;
		points+=10;
		totalFood--;
	}
	else if (nextPlayerMove==2 && food[playerPosition.Y*19+playerPosition.X+1]==1 && drawFrames==12){
		food[playerPosition.Y*19+playerPosition.X+1]=0;
		points+=10;
		totalFood--;
	}
	else if (nextPlayerMove==3 && food[(playerPosition.Y+1)*19+playerPosition.X]==1 && drawFrames==12){
		food[(playerPosition.Y+1)*19+playerPosition.X]=0;
		points+=10;
		totalFood--;
	}
	else if (nextPlayerMove==4 && food[(playerPosition.Y-1)*19+playerPosition.X]==1 && drawFrames==12){
		food[(playerPosition.Y-1)*19+playerPosition.X]=0;
		points+=10;
		totalFood--;
	}
		
	//draw
	for(int i=1;i<=drawFrames;i++){
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		switch (nextPlayerMove){
			case 1:BSP_LCD_FillCircle((playerPosition.X*12+10)-(i-1),playerPosition.Y*12+36,5); break;
			case 2:BSP_LCD_FillCircle((playerPosition.X*12+10)+(i-1),playerPosition.Y*12+36,5); break;
			case 3:BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36)+(i-1),5); break;
			case 4:BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36)-(i-1),5); break;
			default:break;
		}
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		switch (nextPlayerMove){
			case 1:BSP_LCD_FillCircle((playerPosition.X*12+10)-(i),playerPosition.Y*12+36,5); break;
			case 2:BSP_LCD_FillCircle((playerPosition.X*12+10)+(i),playerPosition.Y*12+36,5); break;
			case 3:BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36)+(i),5); break;
			case 4:BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36)-(i),5); break;
			default:break;
		}
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		switch (nextGhostMove){
			case 1:BSP_LCD_FillCircle((ghostPosition.X*12+10)-(i-1),ghostPosition.Y*12+36,5); break;
			case 2:BSP_LCD_FillCircle((ghostPosition.X*12+10)+(i-1),ghostPosition.Y*12+36,5); break;
			case 3:BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36)-(i-1),5); break;
			case 4:BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36)+(i-1),5); break;
			default:break;
		}
		BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
		switch (nextGhostMove){
			case 1:BSP_LCD_FillCircle((ghostPosition.X*12+10)-(i),ghostPosition.Y*12+36,5); break;
			case 2:BSP_LCD_FillCircle((ghostPosition.X*12+10)+(i),ghostPosition.Y*12+36,5); break;
			case 3:BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36)-(i),5); break;
			case 4:BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36)+(i),5); break;
			default:break;
		}
		osDelay(20);
	}
	for(int i=12;i>=drawFrames;i--){
		osDelay(20);
	}
	
	//if ghost catches player, we need to remove their models from screen
	if (drawFrames==6) {
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillCircle((playerPosition.X*12+10),playerPosition.Y*12+36,5); 
		switch (nextPlayerMove){
			case 1:BSP_LCD_FillCircle((playerPosition.X*12+10)-6,playerPosition.Y*12+36,5); break;
			case 2:BSP_LCD_FillCircle((playerPosition.X*12+10)+6,playerPosition.Y*12+36,5); break;
			case 3:BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36)+6,5); break;
			case 4:BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36)-6,5); break;
			default:break;
		}
		switch (nextGhostMove){
			case 1:BSP_LCD_FillCircle((ghostPosition.X*12+10)-6,ghostPosition.Y*12+36,5); break;
			case 2:BSP_LCD_FillCircle((ghostPosition.X*12+10)+6,ghostPosition.Y*12+36,5); break;
			case 3:BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36)-6,5); break;
			case 4:BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36)+6,5); break;
			default:break;
		}
	}
	//update position
	switch (nextPlayerMove){
			case 1:playerPosition.X--; break;
			case 2:playerPosition.X++; break;
			case 3:playerPosition.Y++; break;
			case 4:playerPosition.Y--; break;
			default:break;
		}
	switch (nextGhostMove){
			case 1:ghostPosition.X--; break;
			case 2:ghostPosition.X++; break;
			case 3:ghostPosition.Y--; break;
			case 4:ghostPosition.Y++; break;
			default:break;
		}
	//return 1 if everything is ok, 0 if collision occured
	if (drawFrames==6) return 0;
	else return 1;
}

void mapEditor(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DrawRect(214,0,25,25);
	BSP_LCD_DrawLine(220,5,235,20);
	BSP_LCD_DrawLine(235,5,220,20);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillRect(10,300,12,12);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
	BSP_LCD_DrawRect(28,300,12,12);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_FillCircle(51,306,2);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_FillCircle(68,306,5);
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	BSP_LCD_FillCircle(85,306,5);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DrawRect(4,29,229,253);
	BSP_LCD_FillCircle(startXpos*12+11, startYpos*12+36,5);
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	BSP_LCD_FillCircle(startGhostXpos*12+11, startGhostYpos*12+36,5);
	int currentInstrument=0;
	int changed=0;
	int touchX=0;
	int touchY=0;
	for(;;){
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_FillCircle(startXpos*12+11, startYpos*12+36,5);
		BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
		BSP_LCD_FillCircle(startGhostXpos*12+11, startGhostYpos*12+36,5);
		drawBaseMap();
		//close button
		if (TS_State.TouchDetected==1 && TS_State.X>214 && TS_State.X<240 && TS_State.Y>0 && TS_State.Y<25) {
			fullReset();
			BSP_LCD_Clear(LCD_COLOR_BLACK);
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			BSP_LCD_FillRect(40,130,160,90);
			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
			BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
			BSP_LCD_SetFont(&Font20);
			BSP_LCD_DisplayStringAt(75,135,(uint8_t*)"Paused",LEFT_MODE);
			BSP_LCD_DrawRect(70,160,100,20);
			BSP_LCD_SetFont(&Font16);
			BSP_LCD_DisplayStringAt(75,162,(uint8_t*)"Settings",LEFT_MODE);
			BSP_LCD_DrawRect(70,185,100,20);
			BSP_LCD_SetFont(&Font16);
			BSP_LCD_DisplayStringAt(85,187,(uint8_t*)"Editor",LEFT_MODE);
			return;
		}
		if (TS_State.TouchDetected==1 && TS_State.X>10 && TS_State.X<22 && TS_State.Y>300 && TS_State.Y<312) {
			changed=currentInstrument; currentInstrument=1; 
		}
		if (TS_State.TouchDetected==1 && TS_State.X>28 && TS_State.X<40 && TS_State.Y>300 && TS_State.Y<312) {
			changed=currentInstrument; currentInstrument=2;
		}
		if (TS_State.TouchDetected==1 && TS_State.X>45 && TS_State.X<57 && TS_State.Y>300 && TS_State.Y<312) {
			changed=currentInstrument; currentInstrument=3;
		}
		if (TS_State.TouchDetected==1 && TS_State.X>62 && TS_State.X<74 && TS_State.Y>300 && TS_State.Y<312) {
			changed=currentInstrument; currentInstrument=4;
		}
		if (TS_State.TouchDetected==1 && TS_State.X>79 && TS_State.X<91 && TS_State.Y>300 && TS_State.Y<312) {
			changed=currentInstrument; currentInstrument=5;
		}
		if (changed){
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			switch (changed){
				case 1:BSP_LCD_DrawRect(9,299,14,14); break;
				case 2:BSP_LCD_DrawRect(27,299,14,14); break;
				case 3:BSP_LCD_DrawRect(44,299,14,14); break;
				case 4:BSP_LCD_DrawRect(61,299,14,14); break;
				case 5:BSP_LCD_DrawRect(78,299,14,14); break;
			}
			changed=0;
			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
			switch (currentInstrument){
				case 1:BSP_LCD_DrawRect(9,299,14,14); break;
				case 2:BSP_LCD_DrawRect(27,299,14,14); break;
				case 3:BSP_LCD_DrawRect(44,299,14,14); break;
				case 4:BSP_LCD_DrawRect(61,299,14,14); break;
				case 5:BSP_LCD_DrawRect(78,299,14,14); break;
			}
		}
		
		if (TS_State.TouchDetected==1 && TS_State.X>5 && TS_State.X<234 && TS_State.Y>30 && TS_State.Y<282) {
			touchX=(TS_State.X-5)/12;
			touchY=(TS_State.Y-30)/12;
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK); 
			BSP_LCD_FillRect((touchX*12)+5,touchY*12+30,12,12);
			if (currentInstrument==1) {
				map[touchY*19+touchX]=1;
				food_base[touchY*19+touchX]=0;
			}
			if (currentInstrument==2) {
				map[touchY*19+touchX]=0;
				food_base[touchY*19+touchX]=0;				
			}
			if (currentInstrument==3) {
				food_base[touchY*19+touchX]=1;
				map[touchY*19+touchX]=0;
			}
			if (currentInstrument==4) {
				food_base[touchY*19+touchX]=0;
				map[touchY*19+touchX]=0;
				BSP_LCD_FillRect((startXpos*12)+5,startYpos*12+30,12,12);
				startXpos=touchX;
				startYpos=touchY;
			}
			if (currentInstrument==5) {
				food_base[touchY*19+touchX]=0;
				map[touchY*19+touchX]=0;
				BSP_LCD_FillRect((startGhostXpos*12)+5,startGhostYpos*12+30,12,12);
				startGhostXpos=touchX;
				startGhostYpos=touchY;
			}
		}
		osDelay(40);
	}
}

void manageSettings(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	//header
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillRect(0,0,240,25);
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(70,2,(uint8_t*)"Settings",LEFT_MODE);
	//cross
	BSP_LCD_DrawRect(214,0,25,25);
	BSP_LCD_DrawLine(220,5,235,20);
	BSP_LCD_DrawLine(235,5,220,20);
	//+/- buttons
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayChar(150,80,(uint8_t)'+');
	BSP_LCD_DisplayChar(180,80,(uint8_t)'-');
	BSP_LCD_DrawRect(148,80,20,20);
	BSP_LCD_DrawRect(178,80,20,20);
	BSP_LCD_DisplayChar(150,110,(uint8_t)'+');
	BSP_LCD_DisplayChar(180,110,(uint8_t)'-');
	BSP_LCD_DrawRect(148,110,20,20);
	BSP_LCD_DrawRect(178,110,20,20);
	//frames of switches
	BSP_LCD_DrawRect(30,150,80,25);
	BSP_LCD_DrawRect(130,150,80,25);
	
	//draw control type buttons depending on control type chosen
	if (controlType==0){
		
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillRect(31,151,79,24);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(131,151,79,24);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_SetFont(&Font20);
		BSP_LCD_DisplayStringAt(153,153,(uint8_t*)"TS",LEFT_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(43,153,(uint8_t*)"Gyro",LEFT_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	}
	else{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillRect(131,151,79,24);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(31,151,79,24);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_SetFont(&Font20);
		BSP_LCD_DisplayStringAt(43,153,(uint8_t*)"Gyro",LEFT_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(153,153,(uint8_t*)"TS",LEFT_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	}
	
	//zone marks
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawLine(220,SettingUpperTrigger,239,SettingUpperTrigger);
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	BSP_LCD_DrawLine(220,SettingLowerTrigger,239,SettingLowerTrigger);
	BSP_LCD_SetFont(&Font24);
	char buf[6];
	sprintf(buf,"   %d",SettingUpperTrigger);
	BSP_LCD_DisplayStringAt(140,80,(uint8_t*)buf,RIGHT_MODE);
	sprintf(buf,"   %d",SettingLowerTrigger);
	BSP_LCD_DisplayStringAt(140,110,(uint8_t*)buf,RIGHT_MODE);
	for(;;){
		//close button
		if (TS_State.TouchDetected==1 && TS_State.X>214 && TS_State.X<240 && TS_State.Y>0 && TS_State.Y<25) {
			BSP_LCD_Clear(LCD_COLOR_BLACK);
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			BSP_LCD_FillRect(40,130,160,90);
			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
			BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
			BSP_LCD_SetFont(&Font20);
			BSP_LCD_DisplayStringAt(75,135,(uint8_t*)"Paused",LEFT_MODE);
			BSP_LCD_DrawRect(70,160,100,20);
			BSP_LCD_SetFont(&Font16);
			BSP_LCD_DisplayStringAt(75,162,(uint8_t*)"Settings",LEFT_MODE);
			BSP_LCD_DrawRect(70,185,100,20);
			BSP_LCD_SetFont(&Font16);
			BSP_LCD_DisplayStringAt(85,187,(uint8_t*)"Editor",LEFT_MODE);
			return;
		}
		
		//upper trigger +
		if (TS_State.TouchDetected==1 && TS_State.X>148 && TS_State.X<168 && TS_State.Y>80 && TS_State.Y<100) {
			if (SettingUpperTrigger<SettingLowerTrigger-20){
				SettingUpperTrigger++;
				sprintf(buf,"   %d",SettingUpperTrigger);
				BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
				BSP_LCD_DisplayStringAt(140,80,(uint8_t*)buf,RIGHT_MODE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_DrawLine(220,SettingUpperTrigger-1,238,SettingUpperTrigger-1);
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_DrawLine(220,SettingUpperTrigger,238,SettingUpperTrigger);
			}
		}
		
		//upper trigger -
		if (TS_State.TouchDetected==1 && TS_State.X>178 && TS_State.X<198 && TS_State.Y>80 && TS_State.Y<100) {
			if (SettingUpperTrigger>30){
				SettingUpperTrigger--;
				sprintf(buf,"   %d",SettingUpperTrigger);
				BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
				BSP_LCD_DisplayStringAt(140,80,(uint8_t*)buf,RIGHT_MODE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_DrawLine(220,SettingUpperTrigger+1,238,SettingUpperTrigger+1);
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_DrawLine(220,SettingUpperTrigger,238,SettingUpperTrigger);
			}
		}
		
		//lower trigger +
		if (TS_State.TouchDetected==1 && TS_State.X>148 && TS_State.X<168 && TS_State.Y>110 && TS_State.Y<130) {
			if (SettingLowerTrigger<290){
				SettingLowerTrigger++;
				sprintf(buf,"   %d",SettingLowerTrigger);
				BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
				BSP_LCD_DisplayStringAt(140,110,(uint8_t*)buf,RIGHT_MODE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_DrawLine(220,SettingLowerTrigger-1,238,SettingLowerTrigger-1);
				BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
				BSP_LCD_DrawLine(220,SettingLowerTrigger,238,SettingLowerTrigger);
			}
		}
		
		//lower trigger -
		if (TS_State.TouchDetected==1 && TS_State.X>178 && TS_State.X<198 && TS_State.Y>110 && TS_State.Y<130) {
			if (SettingLowerTrigger>SettingUpperTrigger+20){
				SettingLowerTrigger--;
				sprintf(buf,"   %d",SettingLowerTrigger);
				BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
				BSP_LCD_DisplayStringAt(140,110,(uint8_t*)buf,RIGHT_MODE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_DrawLine(220,SettingLowerTrigger+1,238,SettingLowerTrigger+1);
				BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
				BSP_LCD_DrawLine(220,SettingLowerTrigger,238,SettingLowerTrigger);
			}
		}
		
		//gyro button
		if (TS_State.TouchDetected==1 && TS_State.X>30 && TS_State.X<110 && TS_State.Y>150 && TS_State.Y<175) {
			if (controlType==1){
				controlType=0;
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_FillRect(31,151,79,24);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_FillRect(131,151,79,24);
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_SetFont(&Font20);
				BSP_LCD_DisplayStringAt(153,153,(uint8_t*)"TS",LEFT_MODE);
				BSP_LCD_SetBackColor(LCD_COLOR_RED);
				BSP_LCD_DisplayStringAt(43,153,(uint8_t*)"Gyro",LEFT_MODE);
				BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
				angleX=0;
				angleY=0;
				angleZ=0;
			}
		}
		
		//touchscreen button
		if (TS_State.TouchDetected==1 && TS_State.X>130 && TS_State.X<210 && TS_State.Y>150 && TS_State.Y<175) {
			if (controlType==0){
				controlType=1;
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_FillRect(131,151,79,24);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_FillRect(31,151,79,24);
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_SetFont(&Font20);
				BSP_LCD_DisplayStringAt(43,153,(uint8_t*)"Gyro",LEFT_MODE);
				BSP_LCD_SetBackColor(LCD_COLOR_RED);
				BSP_LCD_DisplayStringAt(153,153,(uint8_t*)"TS",LEFT_MODE);
				BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			}
		}
		osDelay(10);
	}
}

void gameLoop(){
	fullReset();
	//wait for touch
	WaitForPressedState(1);
	for(;;){
		if (mode==1){
			//to be executed after mode switch - redraw everything
			if (switchedMode){
				switchedMode=0; 
				BSP_LCD_Clear(LCD_COLOR_BLACK);
				drawMap();
				BSP_LCD_SetFont(&Font12);
				BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				sprintf(text,"Points: %d",points);
				BSP_LCD_DisplayStringAt(10,10,(uint8_t*)text,LEFT_MODE);
				sprintf(text,"Lives: %d",lives);
				BSP_LCD_DisplayStringAt(235,295,(uint8_t*)text,RIGHT_MODE);
				BSP_LCD_FillCircle((playerPosition.X*12+10),(playerPosition.Y*12+36),5);
				BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
				BSP_LCD_FillCircle((ghostPosition.X*12+10),(ghostPosition.Y*12+36),5);
				//wait for touch
				WaitForPressedState(1);
				if (switchedMode) continue;
			}
			if (totalFood==0){
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_DrawRect(39,129,161,61);
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
				BSP_LCD_FillRect(40,130,160,60);
				BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_SetFont(&Font16);
				BSP_LCD_DisplayStringAt(43,135,(uint8_t*)"Level complete",LEFT_MODE);
				BSP_LCD_SetFont(&Font12);
				BSP_LCD_DisplayStringAt(59,168,(uint8_t*)"Touch to continue",LEFT_MODE);
				WaitForPressedState(1);
				ResetGame();
				switchedMode=1;
			}
			//gameover message
			else if (lives==0){
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_DrawRect(39,129,161,61);
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
				BSP_LCD_FillRect(40,130,160,60);
				BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_SetFont(&Font20);
				BSP_LCD_DisplayStringAt(60,135,(uint8_t*)"GAME OVER",LEFT_MODE);
				osDelay(50);
			}
			else {
				//if not gameover, continue
				drawMap();
				if (!proceedMovements()){
					//executes if collision was detected
					//minus 1 life
					lives--;
					
					//wait for touch
					if (lives){
						softResetGame();
						WaitForPressedState(1);
					}
					else{
						BSP_LCD_SetFont(&Font12);
						BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
						BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
						sprintf(text,"Lives: %d",lives);
						BSP_LCD_DisplayStringAt(235,295,(uint8_t*)text,RIGHT_MODE);
					}
				}
				//update points
				BSP_LCD_SetFont(&Font12);
				BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				sprintf(text,"Points: %d",points);
				BSP_LCD_DisplayStringAt(10,10,(uint8_t*)text,LEFT_MODE);
			}
		}
		else{
			//display pause screen
			//it has no moving or changing objects, so actually need to draw it once
			if (switchedMode){
				//check if pause message needs to be drawn
				switchedMode=0; 
				BSP_LCD_Clear(LCD_COLOR_BLACK);
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
				BSP_LCD_FillRect(40,130,160,90);
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
				BSP_LCD_SetFont(&Font20);
				BSP_LCD_DisplayStringAt(75,135,(uint8_t*)"Paused",LEFT_MODE);
				BSP_LCD_DrawRect(70,160,100,20);
				BSP_LCD_SetFont(&Font16);
				BSP_LCD_DisplayStringAt(75,162,(uint8_t*)"Settings",LEFT_MODE);
				BSP_LCD_DrawRect(70,185,100,20);
				BSP_LCD_SetFont(&Font16);
				BSP_LCD_DisplayStringAt(85,187,(uint8_t*)"Editor",LEFT_MODE);
			}
			
			//if settings button pressed for more than 50 ms
			if (TS_State.TouchDetected==1 && TS_State.X>70 && TS_State.X<170 && TS_State.Y>160 && TS_State.Y<180){
				uint16_t TimeStart = HAL_GetTick();
				int check=0;
				while(1){
					osDelay(10);
					if (TS_State.TouchDetected==0){
						if (HAL_GetTick()-TimeStart>50) {check=1; break;}
					}
				}
				if (check) manageSettings();
			}
			
			//if settings button pressed for more than 50 ms
			if (TS_State.TouchDetected==1 && TS_State.X>70 && TS_State.X<170 && TS_State.Y>185 && TS_State.Y<205){
				uint16_t TimeStart = HAL_GetTick();
				int check=0;
				while(1){
					osDelay(10);
					if (TS_State.TouchDetected==0){
						if (HAL_GetTick()-TimeStart>50) {check=1; break;}
					}
				}
				if (check) mapEditor();
			}
			osDelay(10);
		}
	}
}

//user button press handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0){
		mode=1-mode;
		switchedMode=1;
	}
}

void EXTI0_IRQHandler(void){
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
