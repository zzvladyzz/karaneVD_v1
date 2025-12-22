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
#include "LIB_MPU6500_SPI.h"
#include "LIB_FiltroMadgWick.h"
// Constantes físicas del robot (debes medirlas y ajustarlas)
#define TICKS_POR_REVOLUCION  200.0 // Cuantos ticks da el encoder por vuelta completa
#define DIAMETRO_RUEDA_MM      	29.1571 // Diámetro de la rueda en mm y grosor 12,7
#define DISTANCIA_ENTRE_RUEDAS_MM 105.8 // Distancia entre los centros de las ruedas
#define LONGITUD_MUESTRA_MM (M_PI * DIAMETRO_RUEDA_MM / TICKS_POR_REVOLUCION)


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t ADC_DMA[5];	//datos DMA
volatile uint16_t ADC_buffer[4]; //datos ya obtenidos y convertidos a 16bits
char bufferTxt[30];//buffer para enviar datos maximo 30
uint16_t duty_pwm=0;
uint8_t en_pwm=0;
uint16_t ADC_poll=0;
uint32_t tiempoActual=0;
uint32_t tiempoAnterior1=0;
uint32_t tiempoAnterior2=0;
uint32_t tiempo_conv=0;

const int8_t estadoTabla[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //valor encoders de tabla de verdad
uint8_t estadoAnterior_L=0;
uint8_t estadoAnterior_R=0;
// Variables de estado global del robot
volatile int32_t ticks_L = 0; // Contadores de los encoders (desde el callback EXTI)
volatile int32_t ticks_R = 0;

float robot_X = 0.0;     // Posición X actual en mm
float robot_Y = 0.0;     // Posición Y actual en mm
float robot_Angulo = 0.0; // Orientación/Rumor actual en radianes
float robot_Angulo_180=0.0;
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

void UART_Send(UART_HandleTypeDef *huart, char buffer [])
{
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
void imprimir(void)
{
	/*sprintf(bufferTxt," ADC0=%u ",ADC_buffer[0]);
	UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," ADC1=%u ",ADC_buffer[1]);
	UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," ADC2=%u ",ADC_buffer[2]);
	UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," ADC3=%u ",ADC_buffer[3]);
	UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," ADC4=%u \r \n",ADC_poll);
	UART_Send(&huart3, bufferTxt);

	sprintf(bufferTxt," EncL=%ld ",ticks_L);
		UART_Send(&huart3, bufferTxt);
		sprintf(bufferTxt," EncR=%ld " ,ticks_R);
		UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," LEncL=%0.2f ",ticks_L*LONGITUD_MUESTRA_MM);
	UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," LEncR=%0.2f \r \n" ,ticks_R*LONGITUD_MUESTRA_MM);
	UART_Send(&huart3, bufferTxt);

*/



	sprintf(bufferTxt," angulo=%0.2f ",robot_Angulo_180);
	UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," x=%0.2f ",robot_X);
	UART_Send(&huart3, bufferTxt);
	sprintf(bufferTxt," y=%0.2f \r \n",robot_Y);
	UART_Send(&huart3, bufferTxt);


}
void odometria(void)
{
/*
 * Codigo para odometria implementarlo en un timer para la ejecucion
 */
// 1. Obtener los ticks desde la última vez y resetear contadores para evitar overflow de datos
// Usamos variables intermedias para ser thread-safe (si las interrupciones EXTI pueden ocurrir ahora)
int32_t delta_ticks_L = ticks_L;
int32_t delta_ticks_R = ticks_R;
ticks_L = 0; // Resetear el contador
ticks_R = 0; // Resetear el contador

// 2. Convertir ticks a distancia lineal recorrida (en mm)
float distancia_L = delta_ticks_L * LONGITUD_MUESTRA_MM;
float distancia_R = delta_ticks_R * LONGITUD_MUESTRA_MM;

// Distancia promedio recorrida por el centro del robot
float delta_distancia = (distancia_L + distancia_R) / 2.0;

// 3. Calcular el cambio en la orientación angular (en radianes)
float delta_angulo = (distancia_R - distancia_L) / DISTANCIA_ENTRE_RUEDAS_MM;

// 4. Actualizar la posición global del robot (Integración)

// Usamos el ángulo actual + la mitad del cambio angular para mayor precisión (Runge-Kutta de 2do orden)
float angulo_promedio = robot_Angulo + delta_angulo / 2.0;

// Calcular el desplazamiento en X e Y y sumarlo a la posición actual
robot_X += delta_distancia * cos(angulo_promedio);
robot_Y += delta_distancia * sin(angulo_promedio);

// 5. Actualizar el ángulo global del robot
robot_Angulo += delta_angulo;

// Opcional: Mantener el ángulo entre -PI y PI (para evitar overflow float)
if (robot_Angulo > M_PI) robot_Angulo -= 2 * M_PI;
if (robot_Angulo < -M_PI) robot_Angulo += 2 * M_PI;
robot_Angulo_180=(robot_Angulo*180.0)/M_PI;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC1)
	{
		ADC_buffer[0]=(uint16_t)ADC_DMA[0];
		ADC_buffer[1]=(uint16_t)ADC_DMA[1];
		ADC_buffer[2]=(uint16_t)ADC_DMA[2];
		ADC_buffer[3]=(uint16_t)ADC_DMA[3];
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	if(GPIO_Pin==ENCA_L_Pin||GPIO_Pin==ENCB_L_Pin)
		{
		uint8_t bitStatusL=((HAL_GPIO_ReadPin(ENCA_L_GPIO_Port, ENCA_L_Pin))?2:0) | ((HAL_GPIO_ReadPin(ENCB_L_GPIO_Port, ENCB_L_Pin))?1:0);
		ticks_L+=estadoTabla[((estadoAnterior_L<<2)|bitStatusL)];
		estadoAnterior_L=bitStatusL;
		}
	if(GPIO_Pin==ENCA_R_Pin||GPIO_Pin==ENCB_R_Pin)
		{
		uint8_t bitStatusR=((HAL_GPIO_ReadPin(ENCA_R_GPIO_Port, ENCA_R_Pin))?2:0) | ((HAL_GPIO_ReadPin(ENCB_R_GPIO_Port, ENCB_R_Pin))?1:0);
		ticks_R+=(-estadoTabla[((estadoAnterior_R<<2)|bitStatusR)]);
		estadoAnterior_R=bitStatusR;
		}
	/*codigo no optimizado
	 *
	if (GPIO_Pin==ENCA_R_Pin || GPIO_Pin==ENCB_R_Pin)
		{
			uint8_t bitStatusR=((HAL_GPIO_ReadPin(ENCA_R_GPIO_Port, ENCA_R_Pin))?2:0) | ((HAL_GPIO_ReadPin(ENCB_R_GPIO_Port, ENCB_R_Pin))?1:0);
			if(GPIO_Pin==ENCA_R_Pin)
				{
					if((bitStatusR&2)==2)
						{
						encR+=((bitStatusR&1)==1)?1:-1;
						}
					else{
						encR+=((bitStatusR&1)==1)?-1:1;
						}
				}
			else if(GPIO_Pin==ENCB_R_Pin)
						{
						if((bitStatusR&1)==1)
							{
							encR+=((bitStatusR&2)==2)?-1:1;
							}
							else{
								encR+=((bitStatusR&2)==2)?1:-1;
								}
					}
		}
	 */
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
  HAL_ADC_Start_IT(&hadc1);/// lo volvemos a activar por que no activamos la conversion continua en el IOC
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);


//inicializamos librerias
 // MPU6500_Read_t 	MPU_6500_Read;
  MPU6500_Read_t 	MPU_6500_Offset;
  //MPU6500_float_t	MPU_6500_float;
  MPU6500_status_e	MPU_6500_Status;
  MadgWick_t 		MadgWick;
  madgwickInit(&MadgWick);
  MPU_6500_Status=MPU6500_Init(&MPU_6500_Offset,10,DPS1000,G4);
  if (MPU_6500_Status==MPU6500_fail) {
  	for (;;) {
  	UART_Send(&huart3, "Fallo al iniciar MPU\r\n");
  	HAL_Delay(500);
  	}
  }
  UART_Send(&huart3, "Exito al iniciar MPU\r\n");
  	HAL_Delay(500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_ADC_Start_IT(&hadc1);
	  imprimir();
	  HAL_Delay(2000);
	  for(;;){
		  tiempoActual=HAL_GetTick();
		  if(tiempoActual-tiempoAnterior1>=1003)
		  {
			  tiempoAnterior1=tiempoActual;
			  /*tiempo_conv=HAL_GetTick();
			  for(int x=0;x<13;x++)
			  {
			  HAL_ADC_Start(&hadc2);
			  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
			  ADC_poll=HAL_ADC_GetValue(&hadc2);
			  HAL_ADC_Stop(&hadc2);
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			  				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			  				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
			  				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
			  				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
			  				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
			  }
			  HAL_Delay(1);
			  tiempo_conv=HAL_GetTick()-tiempo_conv;*/
			  imprimir();

		  }

		  if(tiempoActual-tiempoAnterior2>=10)
		  {
			  tiempoAnterior2=tiempoActual;
			 // HAL_ADC_Start_IT(&hadc1);
			  odometria();
		  }


	  }
	  /*
 HAL_GPIO_WritePin(EN_MOT_GPIO_Port, EN_MOT_Pin, 1);
 	  MPU6500_Read(&MPU_6500_Read);
	  MPU_6500_float=MPU6500_Scale(&MPU_6500_Read, DPS1000_CONV, G4_CONV);
	  //madgwickUpdateIMU(&MadgWick, &MPU_6500_float);

	  sprintf(bufferTxt," pitch= %0.2f ",(MPU6500_Pitch(&MPU_6500_float)));
	  UART_Send(&huart3, bufferTxt);

	  sprintf(bufferTxt," roll= %0.2f \r\n",(MPU6500_Roll(&MPU_6500_float)));
	  UART_Send(&huart3, bufferTxt);
	  HAL_Delay(10);*/

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
