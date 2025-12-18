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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "usbd_cdc_if.h"
#include "function.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_CHANNELS 5
uint16_t ADCValue[ADC_CHANNELS];
uint16_t ADCFilterResult[ADC_CHANNELS];

enum LedState Led1State = Normal;

volatile uint8_t OLEDUpdateFlag = 0;
unsigned int fps;
void OLEDDisplay(void);
void Update_FPS(void);

#define FilterNum 5
volatile uint8_t ADCNum=0;
volatile uint8_t ADCConvertDone = 0;
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
uint16_t AverageFilter(uint16_t *ValueBuff, uint8_t len)
{
	 uint32_t sum = 0;
   uint8_t i;
   for(i = 0; i < len; ++i)
   {
        sum += ValueBuff[i];	
   }
   return sum/len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t ValueBuff[ADC_CHANNELS][FilterNum];
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	
	//HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);//ADCУ׼����
	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCValue, ADC_CHANNELS);
	
	//LED����
	HAL_TIM_Base_Start_IT(&htim10);
	
	//OLEDˢ�¿���
	HAL_TIM_Base_Start_IT(&htim11);
	
	OLED_Init();
	OLED_Clear();
	OLED_ShowString(0, 0, "Hello", 16, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        // 改进：使用一阶滞后滤波替代均值滤波
        // 优点：无需等待采集满N个点，每次DMA完成即可更新结果，实时性更强，且无需大数组节省RAM
        if(ADCConvertDone == 1)
        {
            ADCConvertDone = 0;
            for(uint8_t i = 0; i < ADC_CHANNELS; i++)
            {
                // 简单的初始化判断，防止上电时从0缓慢爬升
                if(ADCFilterResult[i] == 0)
                {
                    ADCFilterResult[i] = ADCValue[i];
                }
                else
                {
                    // 一阶滤波公式: Y(n) = (X(n) + (K-1)*Y(n-1)) / K
                    // FilterNum 在此处作为滤波系数 K 使用
                    // 系数越大，滤波效果越强（越平滑），但对变化的响应越慢
                    ADCFilterResult[i] = (ADCValue[i] + (FilterNum - 1) * ADCFilterResult[i]) / FilterNum;
                }
            }
        }

        if(OLEDUpdateFlag == 1)
        {
            OLEDDisplay();
            OLEDUpdateFlag = 0;
            //HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	ADCConvertDone = 1;
//	ADCValue[0] = KalmanFilter(ADCValue[0]);
//	ADCValue[1] = KalmanFilter(ADCValue[1]);
//	ADCValue[2] = KalmanFilter(ADCValue[2]);
//	ADCValue[3] = KalmanFilter(ADCValue[3]);
#ifdef UserDebug
	//���޸ģ����Ч��
	char Tmp[100];
	float TempValue = CalculateTemperature(ADCValue[3]);
	uint16_t len = sprintf(Tmp, "Battery V:%d---VADJ:%d---Current V:%d---Temperature:%f\r\n", ADCValue[0],  ADCValue[1],  ADCValue[2],  TempValue);
	CDC_Transmit_FS((uint8_t*)Tmp, len);
#endif
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10)
	{
		static uint8_t Tim7Count = 0;
		Tim7Count++;
		if(Led1State == Normal)
		{
			if(Tim7Count >= 6)
			{
				Tim7Count = 0;
				LED1ON;
			}
			if(Tim7Count == 1)
				LED1OFF;
			
		}
		else if(Led1State == Fast)
		{
			if(Tim7Count >= 2)
			{
				Tim7Count = 0;
				LED1ON;
			}
			if(Tim7Count == 1)
				LED1OFF;
		}
		else if(Led1State == Slow)
		{
			if(Tim7Count >= 10)
			{
				Tim7Count = 0;
				LED1ON;
			}
			if(Tim7Count == 1)
				LED1OFF;
		}
	}
	if(htim->Instance == TIM11)
	{
		//if(OLEDUpdateFlag == 0)
		//{
			OLEDUpdateFlag = 1;
		//}
		//CDC_Transmit_FS((uint8_t*)OLEDUpdateFlag, 2);
	}
}

void OLEDDisplay()
{
	float TempValue = CalculateTemperature(ADCFilterResult[0]);
	float ADCResult[4];
	CalculateResult(ADCFilterResult, 4, ADCResult);
	//OLED_Clear();\
	
	static unsigned int frame_count = 0;
    static uint32_t last_time = 0;
    frame_count++;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_time >= 1000) {
        fps = frame_count;
        frame_count = 0;
        last_time = current_time;
    }
	
	OLED_ShowString(0, 0, "FPS:", 12, 0);
	OLED_ShowFloat(4*6, 0, fps, 2 ,12, 1);
	
	OLED_ShowString(0, 1, "Temperature:", 12, 0);
	OLED_ShowFloat(12*6, 1, TempValue, 2 ,12, 0);
	
	OLED_ShowString(0, 2, "Rectifier OUT A:", 12, 0);
	OLED_ShowFloat(sizeof("Rectifier OUT A:")*6-6, 2, ADCResult[0], 2, 12, 0);
	
	OLED_ShowString(0, 3, "Rectifier OUT V:", 12, 0);
	OLED_ShowFloat(sizeof("Rectifier OUT V:")*6-6, 3, ADCResult[1], 2, 12, 0);
	
	OLED_ShowString(0, 4, "DC-DC OUT V:", 12, 0);
	OLED_ShowFloat(sizeof("DC-DC OUT V:")*6-6, 4, ADCResult[2], 2, 12, 0);
	
	OLED_ShowString(0, 5, "DC-DC OUT A:", 12, 0);
	OLED_ShowFloat(sizeof("DC-DC OUT A:")*6-6, 5, ADCResult[3], 2, 12, 0);
	//OLED_Refresh();
	//Update_FPS();
}

void Update_FPS()
{
  uint32_t current_time = HAL_GetTick(); // ��ȡ��ǰϵͳʱ�䣨��λ��ms��
	static unsigned int frame_count = 0;
	static uint32_t last_time = 0;
	

  frame_count++;  // �ۼ�֡��

  if (current_time - last_time >= 1000)  // ÿ�����һ�� FPS
  {
     fps = frame_count;  // ��¼ 1 ���ڵ�֡��
     frame_count = 0;    // ���¼���
     last_time = current_time; // ��¼ʱ��
  }
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
