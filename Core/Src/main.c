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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "LIB_MPU6500_SPI.h"
#include "LIB_DEBUG.h"
#include "LIB_FUNCIONES.h"
#include "LIB_MENU.h"

#define	Delay_BTN	200		// Tiempo para verificar los ADC por DMA
#define Delay_LED	20		// Tiempo para apagar LEDs
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


odometria_init_t odometria={0.0,0.0,0.0,0.0,0,0};
MPU6500_Init_Values_t 	MPU6500_Datos; //Iniciamos donde se guardaran todos los datos a leer
//MPU6500_float_t	MPU6500_Values_float;
MPU6500_status_e	MPU6500_Status;

/* Variables para los ADC */
uint32_t ADC_DMA[5];	//datos DMA
volatile uint16_t ADC_buffer[4]; //datos ya obtenidos y convertidos a 16bits
uint16_t ADC_Sensor=0;
uint8_t ValorBTN=0;

/* Variables para los encoders*/
const int8_t estadoTabla[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //valor encoders de tabla de verdad
uint8_t estadoAnterior_L=0;
uint8_t estadoAnterior_R=0;

/* Variables para definir tiempo de espera */
uint32_t tiempoActual=0;
uint32_t tiempoAnterior=0;
uint32_t tiempoAnterior_LED=0;

bool	timeValid=false;

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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC1)
	{
		ADC_buffer[0]=(uint16_t)ADC_DMA[0];
		ADC_buffer[1]=(uint16_t)ADC_DMA[1];
		ADC_buffer[2]=(uint16_t)ADC_DMA[2];
		ADC_buffer[3]=(uint16_t)ADC_DMA[3];
		/*filtramos el boton pulsado en un rango*/
		if(ADC_buffer[3]>3900 && ADC_buffer[3]<4100)
		{
			ValorBTN=BTN_DERECHA;
		}
		else if(ADC_buffer[3]>2100 && ADC_buffer[3]<2200)
				{
					ValorBTN=BTN_IZQUIERDA;
				}
		else if(ADC_buffer[3]>2600 && ADC_buffer[3]<2800)
				{
					ValorBTN=BTN_ACEPTAR;
				}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin==ENCA_L_Pin||GPIO_Pin==ENCB_L_Pin)
		{
		uint8_t bitStatusL=((HAL_GPIO_ReadPin(ENCA_L_GPIO_Port, ENCA_L_Pin))?2:0) | ((HAL_GPIO_ReadPin(ENCB_L_GPIO_Port, ENCB_L_Pin))?1:0);
		odometria.ticks_L+=estadoTabla[((estadoAnterior_L<<2)|bitStatusL)];
		estadoAnterior_L=bitStatusL;
		}
	if(GPIO_Pin==ENCA_R_Pin||GPIO_Pin==ENCB_R_Pin)
		{
		uint8_t bitStatusR=((HAL_GPIO_ReadPin(ENCA_R_GPIO_Port, ENCA_R_Pin))?2:0) | ((HAL_GPIO_ReadPin(ENCB_R_GPIO_Port, ENCB_R_Pin))?1:0);
		odometria.ticks_R+=(-estadoTabla[((estadoAnterior_R<<2)|bitStatusR)]);
		estadoAnterior_R=bitStatusR;
		}
}
void leer_ADC(void)
{
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	ADC_Sensor=HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, &ADC_DMA[0], 4);
  //HAL_ADC_Start_IT(&hadc1);/// activa la interrupcion del dma para leer los datos
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);

  MPU6500_Status=MPU6500_Init(&MPU6500_Datos,10,DPS1000,G4);
  if (MPU6500_Status==MPU6500_fail) {
  	for (;;) {
  		DEBUG_Imprimir("Fallo al iniciar MPU\r\n");
  		Menu_LED(Aviso_fallo);
  		}
  }
  DEBUG_Imprimir("Exito al iniciar MPU\r\n");
  Menu_LED(Aviso_ok);
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Esta parte simplemente cada X tiempo verificara el DMA para ver el estado del BTN*/
	  tiempoActual=HAL_GetTick();
	  if ((tiempoActual-tiempoAnterior)>Delay_BTN) {
		tiempoAnterior=tiempoActual;
		DEBUG_ADC(ADC_Sensor, ADC_buffer[0], ADC_buffer[1], ADC_buffer[2], ADC_buffer[3]);
		HAL_ADC_Start_IT(&hadc1);// iniciamos conversion ADC
	  }

	  /*
	   * Obtenido el valor del BTN y siempre distinto a 0
	   * si entra a la funcion se genera un delay de tiempo para apagar los leds
	   * */
	  if(ValorBTN!=0)
	  {
		  Menu_LED(Apagar_LED);
		  timeValid=Menu_Navegacion(ValorBTN);
		  ValorBTN=0;	// Se reetablece valor para evitar que vuelva a entrar
		  tiempoAnterior_LED=HAL_GetTick();
	  }
	  if(timeValid==true)
	  {
		  tiempoActual=HAL_GetTick();
		  if ((tiempoActual-tiempoAnterior_LED)>Delay_LED) {
		  		timeValid=false;
		  		Menu_LED(estadoLED);
		  		}
	  }
	  /* Aca se ejecutara el codigo si se dio aceptar y dependiendo el menu donde este*/
	  if(Menu_Ejecucion()==true)
	  {
		  switch (Menu_Global) {
				case Opcion_Iniciar_CodigoA:
					break;
				case Opcion_Calibracion_Sensores:
					break;
				case Opcion_Calibracion_X:
					break;
				case Opcion_Configuracion_PID_1:
					break;
				case Opcion_Configuracion_PID_2:
					break;
				case Opcion_Guardar:
					break;
				default:
					break;
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
