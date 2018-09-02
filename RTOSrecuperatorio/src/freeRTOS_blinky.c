/*
 * Author: Gaston Vallasciani
 *
 * Materia: Sistemas operativos de tiempo real 1
 *
 * Examen recuperatorio
 *
 * Fecha: 31/08/2018
 *
 * Revision: 2/09/2018
 */

/*==================[inclusiones]============================================*/
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"   		// Se agrega la libreria de manejo de semaforos
#include "queue.h"  		// Se agrega la libreria de manejo de colas
#include "isrPinLevel.h"
#include "global.h"

// sAPI header
#include "sapi.h"

/*==================[definiciones y macros]==================================*/
/*==================[definiciones de datos internos]=========================*/
QueueHandle_t queueMedicionTiempo;
QueueHandle_t buttonsInfo;
QueueHandle_t manejoDePWMLED1;
QueueHandle_t manejoDePWMLED2;
uint8_t LED = LEDR;
/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
static void checkButtonsDebounce(buttonData_t * buttonData);

/* Prototipo de funciones de tareas */
void debounce( void* taskParmPtr );
void timeMeasure( void* taskParmPtr );
void taskPwmLed1( void* taskParmPtr );
void taskPwmLed2( void* taskParmPtr );

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );
   debugPrintlnString( "Blinky con freeRTOS y sAPI." );

   // Creacion de colas
   buttonsInfo = xQueueCreate( LARGO_COLA_BUTTONS_INFO, sizeof(buttonData_t));
   queueMedicionTiempo = xQueueCreate( LARGO_COLA_MEDICION_TIEMPO, sizeof(buttonData_t));
   manejoDePWMLED1 = xQueueCreate( LARGO_COLA_CICLO_DE_TRABAJO, sizeof(pwmManagement_t));
   manejoDePWMLED2 = xQueueCreate( LARGO_COLA_CICLO_DE_TRABAJO, sizeof(pwmManagement_t));

   // Creacion tarea debounce
   xTaskCreate(
   	  debounce,                     // Funcion de la tarea a ejecutar
      (const char *)"debounce",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
      0,                            // Parametros de tarea
      tskIDLE_PRIORITY+1,           // Prioridad de la tarea
      0                             // Puntero a la tarea creada en el sistema
   );

   // Creacion tarea timeMeasure
   xTaskCreate(
      timeMeasure,                  // Funcion de la tarea a ejecutar
      (const char *)"timeMeasure",  // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
      0,                            // Parametros de tarea
      tskIDLE_PRIORITY+2,           // Prioridad de la tarea
      0                             // Puntero a la tarea creada en el sistema
   );

   // Creacion tarea taskPwmLed1
   xTaskCreate(
      taskPwmLed1,                  // Funcion de la tarea a ejecutar
      (const char *)"taskPwmLed1",  // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
      0,                            // Parametros de tarea
      tskIDLE_PRIORITY+3,           // Prioridad de la tarea
      0                             // Puntero a la tarea creada en el sistema
    );

   // Creacion tarea taskPwmLed2
      xTaskCreate(
         taskPwmLed2,                  // Funcion de la tarea a ejecutar
         (const char *)"taskPwmLed2",  // Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
         0,                            // Parametros de tarea
         tskIDLE_PRIORITY+3,           // Prioridad de la tarea
         0                             // Puntero a la tarea creada en el sistema
       );

   // Configuracion interrupcion gpio
   isrPinLevelConfig();
   // Iniciar scheduler
   vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/


void taskPwmLed2( void* taskParmPtr ){

	TickType_t tiempo_inicio_ciclo;
	static pwmManagement_t pwmManagementStructTemp;
	static uint8_t counter, ledState;

	gpioWrite(LED1,ON);
	gpioWrite(LED2,ON);

	counter = 50;

	ledState = LED1ON;

	while(TRUE){

		if(xQueueReceive( manejoDePWMLED2, &pwmManagementStructTemp, 0 ) == pdTRUE){

			if(pwmManagementStructTemp.pwmDutyFlagSuma == TRUEs){
				// saturo el contador en 78 porque el led esta totalmente prendido
				if(counter<100){
					counter++;
				}
			}
			else if(pwmManagementStructTemp.pwmDutyFlagResta == TRUEs){
				if(counter!=0){
				// si es cero dejo de restar para que no se desborde
					counter--;
				}
			}

			if(pwmManagementStructTemp.led1Toggle == TRUEs){
				if (ledState == LED1ON){
					ledState = LED1OFF;
					gpioWrite(LED2,OFF);
				}
				else{
					ledState = LED1ON;
				}
			}
		}

		tiempo_inicio_ciclo = xTaskGetTickCount();
		if (ledState == LED1ON){
			gpioWrite(LED2,ON);
		}
		vTaskDelay( counter );
		gpioWrite(LED2,OFF);
		vTaskDelayUntil( &tiempo_inicio_ciclo, 100 );
	}
}

void taskPwmLed1( void* taskParmPtr ){

	TickType_t tiempo_inicio_ciclo;
	static pwmManagement_t pwmManagementStructTemp;
	static uint8_t  counter, ledState;

	gpioWrite(LED1,ON);
	gpioWrite(LED2,ON);

	counter = 50;
	LED = LED1;
	ledState = LED1ON;

	while(TRUE){

		if(xQueueReceive( manejoDePWMLED1, &pwmManagementStructTemp, 0 ) == pdTRUE){

			if(pwmManagementStructTemp.pwmDutyFlagSuma == TRUEs){
				// saturo el contador en 100 porque el led esta totalmente prendido
				if(counter<100){
					counter++;
				}
			}
			else if(pwmManagementStructTemp.pwmDutyFlagResta == TRUEs){
				// si es cero dejo de restar para que no se desborde
				if(counter!=0){
					counter--;
				}
			}

			if(pwmManagementStructTemp.led1Toggle == TRUEs){
				if (ledState == LED1ON){
					ledState = LED1OFF;
					gpioWrite(LED1,OFF);
				}
				else{
					ledState = LED1ON;
				}
			}
		}

		tiempo_inicio_ciclo = xTaskGetTickCount();
		if (ledState == LED1ON){
			gpioWrite(LED1,ON);
		}
		vTaskDelay( counter );
		gpioWrite(LED1,OFF);
		vTaskDelayUntil( &tiempo_inicio_ciclo, 100 );

	}
}

void timeMeasure( void* taskParmPtr ){

	buttonData_t buttonDataTemp;

	static pwmManagement_t pwmManagementStruct;

	uint8_t dutyCycleCounter = 50;
	uint8_t ledAnt = LED2;
	uint8_t ocurrioEvento = 0;

	TickType_t tiempoMedicionInicial;
	TickType_t tiempoMedicionFinal;
	TickType_t tiempoEntreEventos;

	//pwmManagementStruct.pwmDutyCounter = EMPTYs;
	pwmManagementStruct.pwmDutyFlagSuma = EMPTYs;
	pwmManagementStruct.pwmDutyFlagResta = EMPTYs;
	pwmManagementStruct.ledSelect = LED1A;
	pwmManagementStruct.led1Toggle = LED1ON;

	 gpioWrite( LED3, OFF );

	while(TRUE){
		// Recibo datos por la cola de medicion de tiempo
		xQueueReceive( queueMedicionTiempo, &buttonDataTemp, portMAX_DELAY );

			if((buttonDataTemp.buttonIndex == TECLA1)&&(buttonDataTemp.buttonLastState == FALLING)){

				pwmManagementStruct.pwmDutyFlagSuma = TRUEs;
				pwmManagementStruct.pwmDutyFlagResta = FALSEs;
				pwmManagementStruct.ledSelect = EMPTYs;
				pwmManagementStruct.led1Toggle = EMPTYs;
				ocurrioEvento = 1;

			}
			else if((buttonDataTemp.buttonIndex == TECLA2)&&(buttonDataTemp.buttonLastState == FALLING)){

				pwmManagementStruct.pwmDutyFlagSuma = FALSEs;
				pwmManagementStruct.pwmDutyFlagResta = TRUEs;
				pwmManagementStruct.ledSelect = EMPTYs;
				pwmManagementStruct.led1Toggle = EMPTYs;
				ocurrioEvento = 1;

			}else if(buttonDataTemp.buttonIndex == TECLA3){

					if(buttonDataTemp.buttonState == FALLING){
						tiempoMedicionInicial = buttonDataTemp.buttonTime;
					}else if(buttonDataTemp.buttonState == RAISING){

						tiempoMedicionFinal = buttonDataTemp.buttonTime;
						tiempoEntreEventos = tiempoMedicionFinal - tiempoMedicionInicial;

						if(tiempoEntreEventos < 500*portTICK_RATE_MS){

							pwmManagementStruct.ledSelect = TRUEs;
							pwmManagementStruct.led1Toggle = FALSEs;
							pwmManagementStruct.pwmDutyFlagSuma = EMPTYs;
							pwmManagementStruct.pwmDutyFlagResta = EMPTYs;

						}else if(tiempoEntreEventos > 500*portTICK_RATE_MS){
							pwmManagementStruct.ledSelect = FALSEs;
							pwmManagementStruct.led1Toggle = TRUEs;
							pwmManagementStruct.pwmDutyFlagSuma = EMPTYs;
							pwmManagementStruct.pwmDutyFlagResta = EMPTYs;

						}
						ocurrioEvento = 1;
					}
			buttonDataTemp.buttonIndex = _NONE;
		}
		// Si se presiono alguna tecla envio datos a las tareas de pwm
		if(ocurrioEvento == 1){
			ocurrioEvento = 0;
			// switcheo de tarea a la que se envia el contador
			if((pwmManagementStruct.ledSelect == TRUEs)){
				if (ledAnt == LED2){
					//xQueueSend(manejoDePWMLED1, &pwmManagementStruct, portMAX_DELAY);
					ledAnt = LED1;
				}
				else if(ledAnt == LED1){
					//xQueueSend(manejoDePWMLED2, &pwmManagementStruct, portMAX_DELAY);
					ledAnt = LED2;
				}
			}
			// sino ocurrio un evento de switcheo envio a la que estaba y no modifico la tarea a la que se envia
			else if((pwmManagementStruct.ledSelect == EMPTYs)){
				// Se envia si hay que sumar o restar 1 al ciclo de trabajo del pwm
				if (ledAnt == LED2){
					xQueueSend(manejoDePWMLED1, &pwmManagementStruct, portMAX_DELAY);
				}
				else if(ledAnt == LED1){
					xQueueSend(manejoDePWMLED2, &pwmManagementStruct, portMAX_DELAY);
				}
			}
			else if((pwmManagementStruct.ledSelect == FALSEs)){
				//como se deben apagar o prender ambos led se envia a las dos tareas de manejo de pwm
				xQueueSend(manejoDePWMLED1, &pwmManagementStruct, portMAX_DELAY);
				xQueueSend(manejoDePWMLED2, &pwmManagementStruct, portMAX_DELAY);
			}
		}
	}
}

void debounce( void* taskParmPtr )
{
	static buttonData_t buttonDataTemp;   // Data temporal
	static buttonData_t buttonsData[4];   // Data de cada tecla
	static uint8_t i;
	static buttonNum_t lastButtonInt;

// Inicializo las estructuras que manejan los pulsadores
	for (i=0;i<4;i++){
		buttonsData[i].buttonLastState = NONE;
		buttonsData[i].buttonState = IDLE;
		buttonsData[i].buttonFirstEvent = NONE;
		buttonsData[i].buttonIndex = i;
	}
   while(TRUE){
	   if(xQueueReceive( buttonsInfo, &buttonDataTemp, DEBOUNCE_TIME ) == pdTRUE){
		   buttonsData[buttonDataTemp.buttonIndex].buttonState = buttonDataTemp.buttonState; // Actualizo el flanco ocurrido
		   buttonsData[buttonDataTemp.buttonIndex].buttonTime = buttonDataTemp.buttonTime; // Actualizo el tiempo de ocurrencia del flanco
		   lastButtonInt = buttonDataTemp.buttonIndex; // Actualizo la tecla sobre la que ocurrio el evento
	   }
	   // Chequeo el rebote con el antirrebote, si no hay rebote envio los datos por la cola
	   checkButtonsDebounce(&buttonsData[lastButtonInt]);
   }
}

static void checkButtonsDebounce(buttonData_t * buttonData) {
	/*
	 * El antirrebote se diseña considerando que si se presiona la tecla, el 1er
	 * evento es de FALLING , y si luego de 20ms la tecla sigue presionada,
	 * entonces la medicion del flanco descendente es correcta y se valida
	 * el evento de FALLING. Si se lee un evento de RAISING entonces hubo un
	 * falso contacto, y por l otanto es considerado un rebote.
	 * El razonamiento analogo es para el caso de liberarse la tecla, siendo el
	 * 1er evento de RAISING... el ultimo evento dentro de la ventana de 20ms
	 * tambien debe ser de RAISING, sino existio un rebote.
	 */
	buttonData_t buttonDataTemp = buttonData[0];

	switch(buttonData->buttonState) {

	case FALLING:
		// Verifico si es el primer evento de falling de la secuencia antirrebote
		if(buttonData->buttonFirstEvent == IDLE) {
			// Actualizo el Flag FirstEvent.
			buttonData->buttonFirstEvent = FIRST_FALLING;
			// Guardo valor de cuenta actual. Este valor es actualizado en la interrupcion del flanco
			buttonData->buttonTimeFirstEvent = buttonData->buttonTime;
		}

		// Si hubo un evento de FIRST_FALLING me fijo si se mantiene presionado
		if(buttonData->buttonFirstEvent == FIRST_FALLING) {
			// Me fijo si el tiempo entre eventos de FALLING es menor a 20ms
			if((xTaskGetTickCount() - buttonData->buttonTimeFirstEvent) < (DEBOUNCE_TIME * portTICK_RATE_MS)) {
				// Guardo ultimo estado de tecla
				buttonData->buttonLastState = buttonData->buttonState;

			} else {
				// Si no lo es entonces es mayor a 20 ms
				if(buttonData->buttonLastState == FALLING) {
					// Si el ultimo estado habia sido de falling entonces la tecla fue presionada y fue detectado un rebote
					// Envio data por cola
					xQueueSend(queueMedicionTiempo, &buttonDataTemp, portMAX_DELAY); // se envia el tiempo medido al presionar
//					xQueueSend(queueLaunchLeds, &buttonDataTemp, portMAX_DELAY);	// envio la cola a la tarea que maneja el encendido de leds
				} else {
					// si el estado anterior era de RAISING entonces fue detectado un rebote
				}
				// limpio los flags de estado
				buttonData->buttonState = NONE;
				buttonData->buttonFirstEvent = IDLE;
			}
		}
		break;

	case RAISING:
		// Verifico si es el primer evento de raising de la secuencia antirrebote
		if(buttonData->buttonFirstEvent == IDLE) {
			// Actualizo el Flag FirstEvent.
			buttonData->buttonFirstEvent = FIRST_RAISING;
			// Guardo valor de cuenta actual. Este valor es actualizado en la interrupcion del flanco
			buttonData->buttonTimeFirstEvent = buttonData->buttonTime;
		}

		// Si hubo un evento de FIRST_RAISING me fijo si se mantiene presionado
		if(buttonData->buttonFirstEvent == FIRST_RAISING) {
			// Me fijo si el tiempo entre eventos de RAISING es menor a 20ms
			if((xTaskGetTickCount() - buttonData->buttonTimeFirstEvent) < (DEBOUNCE_TIME * portTICK_RATE_MS)) {
				// Guardo ultimo estado de tecla
				buttonData->buttonLastState = buttonData->buttonState;

			} else {
				// Si no lo es entonces es mayor a 20 ms
				if(buttonData->buttonLastState == RAISING) {
					// Si el ultimo estado habia sido de falling entonces la tecla fue liberada y fue detectado un rebote
					// Envio data por cola
				xQueueSend(queueMedicionTiempo, &buttonDataTemp, portMAX_DELAY); // se envia el tiempo medido al presionar
//				xQueueSend(queueLaunchLeds, &buttonDataTemp, portMAX_DELAY);	// envio la cola a la tarea que maneja el encendido de leds
				} else {
					// si el estado anterior era de RAISING entonces fue detectado un rebote
				}
				// limpio los flags de estado
				buttonData->buttonState = NONE;
				buttonData->buttonFirstEvent = IDLE;
			}
		}
		break;

	case NONE:
	default:
		//Caso de error, no hace nada
		break;
	}
}





/*==================[fin del archivo]========================================*/
