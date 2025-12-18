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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include "los_task.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void send_to_lcd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t cmdBuf[8]={0x01, 0x06, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00};
uint8_t dispCmdBuf[32];
uint16_t index_cmd;
uint8_t  voiceFlag;
/*
  CRC16的C语言算法：
*/
#define PRESET_VALUE 0xFFFF
#define POLYNOMIAL  0xa001   // RFID读写器为  0x8408

static uint16_t uiCrc16Cal(uint8_t *pucY, uint8_t ucX)
{
	uint8_t ucI,ucJ;
	uint16_t  uiCrcValue = PRESET_VALUE;

   	for(ucI = 0; ucI < ucX; ucI++)
	   {
		   uiCrcValue = uiCrcValue ^ (pucY[ucI] & 0xff);
	  	   for(ucJ = 0; ucJ < 8; ucJ++)
	   	  {
		 	if(uiCrcValue & 0x0001)
		   	{
		    	uiCrcValue = (uiCrcValue >> 1) ^ POLYNOMIAL;
		   	}
		 	else
		   	{
		    	uiCrcValue = (uiCrcValue >> 1);
		   	}
		}
 	}
    return uiCrcValue;
}


/*
	通过串口发送到显示屏
*/
static void send_to_lcd(void)
{
	dispCmdBuf[index_cmd] = 0xff;
	dispCmdBuf[index_cmd+1] = 0xff;
	dispCmdBuf[index_cmd+2] = 0xff;
	HAL_UART_Transmit_DMA(&huart2, dispCmdBuf, index_cmd+3);
}

static void VoiceTask(void)
{
	uint16_t crc16;
	
	cmdBuf[0] = 0x01;
	cmdBuf[1] = 0x06;
	cmdBuf[2] = 0x00;
	cmdBuf[3] = 0x0d;
	cmdBuf[4] = 0x00;
	cmdBuf[5] = 0x00;
	
  while(1) {

    LOS_TaskDelay(100);
		
		if(Uart3Rx.RxFlag){
        Uart3Rx.RxFlag = 0;
				if (strstr((char *)&Uart3Rx.RxBuff[16],"openled") != NULL)
				{
					cmdBuf[4] = 0xff;
					crc16 = uiCrc16Cal(cmdBuf, 6);
					cmdBuf[6] = crc16 & 0xff;
					cmdBuf[7] = (crc16 >> 8) & 0xff;
					//HAL_UART_Transmit_DMA(&huart4, cmdBuf, 8);
					voiceFlag = 1;

				}
				else
				{
					if (strstr((char *)&Uart3Rx.RxBuff[16],"closeled") != NULL)
					{
							cmdBuf[4] = 0x00;
							crc16 = uiCrc16Cal(cmdBuf, 6);
							cmdBuf[6] = crc16 & 0xff;
							cmdBuf[7] = (crc16 >> 8) & 0xff;
							//HAL_UART_Transmit_DMA(&huart4, cmdBuf, 8);
							voiceFlag = 1;
					}
				}
		}
  }
}

#define CIRCUIT_BREAKER_ADDR 1
#define TEMP_HUMI_ADDR 			 8

typedef struct{
	int16_t temp;
	uint16_t humi;
	uint16_t volty;
	uint16_t current;
} SENSOR_DATA;

SENSOR_DATA sensor;

static uint8_t func03[]={0x01,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
static uint8_t addrSet[]={CIRCUIT_BREAKER_ADDR,TEMP_HUMI_ADDR};

static void SensorTask(void)
{
	uint8_t devAddr;
	uint8_t addrIndex;
	uint16_t crc16;
	
	addrIndex = 0;
  while(1) {
			if (voiceFlag == 1)
			{
				voiceFlag = 0;
				HAL_UART_Transmit_DMA(&huart4, cmdBuf, 8);
				LOS_TaskDelay(100);
			}
				
			devAddr = addrSet[addrIndex];
			func03[0] = devAddr;
			switch(devAddr)
			{
				case CIRCUIT_BREAKER_ADDR:
					func03[1] = 0x04;
					func03[3] = 0x08;
					func03[5] = 0x02;
					break;
				case TEMP_HUMI_ADDR:
					func03[1] = 0x03;
					func03[3] = 0x00;
					func03[5] = 0x02;
					break;
				default:
					break;
			}
			crc16 = uiCrc16Cal(func03, 6);
			func03[6] = crc16 & 0xff;
			func03[7] = (crc16 >> 8) & 0xff;
			HAL_UART_Transmit_DMA(&huart4, func03, 8);
			addrIndex += 1;
			addrIndex %= sizeof(addrSet);
			
			LOS_TaskDelay(300);
			
			if(Uart4Rx.RxFlag){
        Uart4Rx.RxFlag = 0;
				switch(Uart4Rx.RxBuff[0])
				{
					case CIRCUIT_BREAKER_ADDR:
//						HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
						sensor.volty = (Uart4Rx.RxBuff[3] << 8) + Uart4Rx.RxBuff[4];
						sensor.current = (Uart4Rx.RxBuff[5] << 8) + Uart4Rx.RxBuff[6];
						break;
					case TEMP_HUMI_ADDR:
						sensor.humi = (Uart4Rx.RxBuff[3] << 8) + Uart4Rx.RxBuff[4];
						sensor.temp = (Uart4Rx.RxBuff[5] << 8) + Uart4Rx.RxBuff[6];
						break;
					default:
						break;
				}
			}
			
			LOS_TaskDelay(500);
  }
}

static void DisplayTask(void)
{
	uint8_t pageNo, idNo, status, timeCnt;
	uint16_t crc16;
	
	pageNo = 0;
	status = 0;
	timeCnt = 0;
	
	strcpy((char*)dispCmdBuf, "page 0");
	index_cmd = 6;
	send_to_lcd();
	LOS_TaskDelay(50);


	while(1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		if(Uart2Rx.RxFlag){
        Uart2Rx.RxFlag = 0;
				switch(Uart2Rx.RxBuff[0])
				{
					case 0x65:
						pageNo = Uart2Rx.RxBuff[1];
						idNo = Uart2Rx.RxBuff[2];
						if (pageNo == 0)
							pageNo = 1;
						else
						{
							if ((pageNo == 2) && (idNo == 9))
							{
									status ^= 1;
									if (status == 1)
									{
										cmdBuf[4] = 0xff;
									}
									else
									{
										cmdBuf[4] = 0x00;
									}
									crc16 = uiCrc16Cal(cmdBuf, 6);
									cmdBuf[6] = crc16 & 0xff;
									cmdBuf[7] = (crc16 >> 8) & 0xff;
									//HAL_UART_Transmit_DMA(&huart4, cmdBuf, 8);
									voiceFlag = 1;
							}
							else
							{
								pageNo = idNo - 3;
							}
						}
						break;
					case 0x66:
						pageNo = Uart2Rx.RxBuff[1];
						break;
					case 0x1a:
						break;
					default:
						break;
			}
		}
		
		timeCnt += 1;
		
		if (timeCnt == 5)
		{
				timeCnt = 0;
				if (pageNo == 1) 
				{
						index_cmd = sprintf((char*)dispCmdBuf, "x0.val=%d", sensor.temp);
						send_to_lcd();
						LOS_TaskDelay(50);

						index_cmd = sprintf((char*)dispCmdBuf, "x1.val=%d", sensor.humi);
						send_to_lcd();
						LOS_TaskDelay(50);
					
						index_cmd = sprintf((char*)dispCmdBuf, "x5.val=%d", sensor.volty);
						send_to_lcd();
						LOS_TaskDelay(50);
					
						index_cmd = sprintf((char*)dispCmdBuf, "x6.val=%d", sensor.current);
						send_to_lcd();
						LOS_TaskDelay(50);
						timeCnt = 1;
				}
	}
		
#if 0
		else
		{
			strcpy((char*)dispCmdBuf, "sendme");
			index_cmd = 6;
			send_to_lcd();
		}
#endif		
    LOS_TaskDelay(200);
		
  }
}

UINT32 Voice_Task_Handle;
UINT32 Sensor_Task_Handle;
UINT32 Display_Task_Handle;

static UINT32 AppTaskCreate(void)
{
  UINT32 uwRet = LOS_OK;
	
  TSK_INIT_PARAM_S task_init_param;	

	task_init_param.usTaskPrio = 5;
	task_init_param.pcName = "VoiceTask";
	task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)VoiceTask;
	task_init_param.uwStackSize = 1024;	
	uwRet = LOS_TaskCreate(&Voice_Task_Handle, &task_init_param);
  if (uwRet != LOS_OK)
  {
    printf("VoiceTask create failed, %X\n", uwRet);
    return uwRet;
  }
    
  task_init_param.usTaskPrio = 4;	
	task_init_param.pcName = "SensorTask";
	task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)SensorTask;
	task_init_param.uwStackSize = 1024;
	uwRet = LOS_TaskCreate(&Sensor_Task_Handle, &task_init_param);
  if (uwRet != LOS_OK)
  {
    printf("SensorTask create failed, %X\n", uwRet);
    return uwRet;
  } 

    
  task_init_param.usTaskPrio = 6;	
	task_init_param.pcName = "DisplayTask";
	task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)DisplayTask;
	task_init_param.uwStackSize = 1024;
	uwRet = LOS_TaskCreate(&Display_Task_Handle, &task_init_param);
  if (uwRet != LOS_OK)
  {
    printf("DisplayTask create failed, %X\n", uwRet);
    return uwRet;
  } 
	
	return LOS_OK;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	
	voiceFlag = 0;

#if 1
  LOS_KernelInit(); // ��ʼ�� LiteOS-m �ں�
	UINT32 uwRet = AppTaskCreate(); // ��������
  if(uwRet != LOS_OK) {
      printf("LOS Creat task failed\r\n");
  }
  LOS_Start();      // ���� LiteOS-m �ں�
#endif	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      printf("This is output by printf!\r\n");
      if(Uart1Rx.RxFlag){
        Uart1Rx.RxFlag = 0;
        UART_SendData(USART1,Uart1Rx.RxBuff,Uart1Rx.RxLen);
      }
		
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
