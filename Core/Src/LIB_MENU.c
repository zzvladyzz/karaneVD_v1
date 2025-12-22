/*
 * LIB_LED_INDA.c
 *
 *  Created on: Dec 13, 2025
 *      Author: Vlady-Chuwi
 */
#include "LIB_MENU.h"
#include "gpio.h"
//static Avisos_Alarmas_e 	Aviso;
static Menu_Parametros_e 	Menu=Pagina_Inicial;
static bool BTN_SEL=false;
static Tiempo_bloqueante_s Tiempo_bloqueante_A={0,false,0};



void	Tiempo_bloqueante()
{

	uint32_t tiempo_actual=HAL_GetTick();
	switch (Tiempo_bloqueante_A.estado_interno) {
		case 0:
			Tiempo_bloqueante_A.tiempo_espera=tiempo_actual;
			Tiempo_bloqueante_A.en_espera=true;
			Tiempo_bloqueante_A.estado_interno=1;
			break;
		case 1:
			if(tiempo_actual-Tiempo_bloqueante_A.tiempo_espera>20)
			{
				Tiempo_bloqueante_A.estado_interno=2;
				Tiempo_bloqueante_A.en_espera=false;
			}
			break;
		case 2:
			Tiempo_bloqueante_A.estado_interno=0;
			break;
		default:
			break;
	}
}
void 	Menu_Navegacion(uint8_t BTN)
{
	if(BTN_SEL==false){
		switch (BTN) {
			case BTN_ARRIBA:
				Menu=(Menu==Pagina_Inicial)?Pagina_Final:(Menu-1);
				break;
			case BTN_ABAJO:
				Menu=(Menu==Pagina_Final)?Pagina_Inicial:(Menu+1);
				break;
			case BTN_ACEPTAR:
				Menu=Menu;
				BTN_SEL=true;
				break;
			default:
				break;
		}
	}
	else{
		switch (BTN) {
			case BTN_ARRIBA:
				BTN_SEL=false;
				break;
			default:
				break;
		}
	}


}
void	Menu_Ejecucion(void)
{
	if(BTN_SEL==true){
		switch (Menu) {
			case Pagina_Inicial:

				break;
			case Opcion_Iniciar_CodigoA:

				break;
			case Opcion_Calibracion_IMU:

				break;
			case Opcion_Calibracion_Sensores:

				break;
			case Opcion_Calibracion_X:

				break;
			case Opcion_Configuracion_PID_1:

				break;
			case Opcion_Configuracion_PID_2:

				break;
			case Opcion_Monitoreo:

				break;
			case Opcion_Guardar:
				break;

			default:
				break;
			}
	}
}

/*
Estado_ok,
Estado_fallo,
Estado_advertencia,
Aviso_conectado,
Aviso_procesando,
Aviso_ok,
Aviso_listo,
Aviso_desconectado,
Aviso_error_sensor,
Aviso_bateria_baja,
Aviso_fallo_comunicacion,*/
void Menu_Avisos(Avisos_Alarmas_e Aviso_)
{
	switch(Aviso_){
			case 0://FALLO PARA CUALQUIER PROBLEMA
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
				break;
			case 1:// LED PARA VERIFICAR QUE SE INICIALIZO ALGUN MODULO
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
				break;
			case 2:// LED PARA INDICAR QUE SE PULSO EL BOTON PARA ARRIBA
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
				break;
			case 3:// LED PARA INDICAR QUE SE PULSO EL BOTON DE ABAJO
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
				break;
			case 4:// LED PARA INDICAR QUE SE PULSO EL BOTON DE ACEPTAR
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);
				break;
			case 5:// LED PARA INDICAR QUE SE ESTAN INGRESANDO PARAMETROS
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
				break;
			case 6:// LED PARA INDICAR QUE SE INICIO EL PROGRAMA
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 0);
				break;
			default:
				break;


	}
}
