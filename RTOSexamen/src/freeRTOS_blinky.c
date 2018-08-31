/* Author: Gaston Vallasciani
 * Materia: Sistemas operativos de tiempo real 1
 * Fecha: 12/08/2018
 *
 * Ejercicio 7: Colas e ISR
 *
 */

/*==================[inclusiones]============================================*/
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"   // Hay que agregarlo siempre para poder utilizar un semaforo
#include "isrPinLevel.h"
#include "global.h"

// sAPI header
#include "sapi.h"

/*==================[definiciones y macros]==================================*/
/*==================[definiciones de datos internos]=========================*/
SemaphoreHandle_t mutexUart;
SemaphoreHandle_t semaforo;
QueueHandle_t queueLaunchLeds;
QueueHandle_t queueMedicionTiempo;
QueueHandle_t buttonsInfo;
QueueHandle_t queueLogs;
uint8_t LED = LEDR;
/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
static void checkButtonsDebounce(buttonData_t * buttonData);

/* Prototipo de funciones de tareas */
void showLEDs( void* taskParmPtr );
void debounce( void* taskParmPtr );
void timeMeasure( void* taskParmPtr );
void showLog( void* taskParmPtr );

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

   mutexUart = xSemaphoreCreateMutex(); // creacion de Mutex, arrancan liberados
   semaforo = xSemaphoreCreateBinary();

   buttonsInfo = xQueueCreate( LARGO_COLA_BUTTONS_INFO, sizeof(buttonData_t));
   queueLaunchLeds = xQueueCreate( LARGO_COLA_LAUNCH_LEDS, sizeof(buttonData_t));
   queueMedicionTiempo = xQueueCreate( LARGO_COLA_MEDICION_TIEMPO, sizeof(buttonData_t));
   queueLogs = xQueueCreate( LARGO_COLA_SYSTEM_LOG, sizeof(systemLogData_t));

   semaforo = xSemaphoreCreateBinary();
   xSemaphoreGive(semaforo); // Arranco el semaforo liberado

   // Creacion de la tarea showLeds
   xTaskCreate(
	  showLEDs,                     // Funcion de la tarea a ejecutar
      (const char *)"showLED1",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
      0,                            // Parametros de tarea
      tskIDLE_PRIORITY+1,           // Prioridad de la tarea
      0                             // Puntero a la tarea creada en el sistema
   );

   // Creacion tarea debounce
   xTaskCreate(
   	  debounce,                     // Funcion de la tarea a ejecutar
      (const char *)"debounce",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
      0,                            // Parametros de tarea
      tskIDLE_PRIORITY+2,           // Prioridad de la tarea
      0                             // Puntero a la tarea creada en el sistema
   );

   // Creacion tarea timeMeasure
   xTaskCreate(
      timeMeasure,                  // Funcion de la tarea a ejecutar
      (const char *)"timeMeasure",  // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
      0,                            // Parametros de tarea
      tskIDLE_PRIORITY+3,           // Prioridad de la tarea
      0                             // Puntero a la tarea creada en el sistema
   );

   // Creacion tarea showLog
   xTaskCreate(
      showLog,                      // Funcion de la tarea a ejecutar
      (const char *)"showLog",      // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
      0,                            // Parametros de tarea
      tskIDLE_PRIORITY+1,           // Prioridad de la tarea
      0                             // Puntero a la tarea creada en el sistema
   );

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

void showLog( void* taskParmPtr ){

	systemLogData_t systemLogData;

	while(TRUE){

		xQueueReceive( queueMedicionTiempo, &systemLogData, portMAX_DELAY );

		xSemaphoreTake(mutexUart, portMAX_DELAY); // Tomo el Mutex
		printf("{B%i:B%i:%ims}\r\n", systemLogData.x, systemLogData.y, systemLogData.tiempoEntreEventos);
		xSemaphoreGive(mutexUart); // Libero el Mutex

		gpioWrite( LED3, ON );
		vTaskDelay(DELAY_LED_BLINK/portTICK_RATE_MS);
		gpioWrite( LED3, OFF );
	}
}

void timeMeasure( void* taskParmPtr ){

	buttonData_t buttonDataTemp;
	systemLogData_t systemLogData;
	buttonNum_t lastButtonPressed = _NONE;

	TickType_t tiempoMedicionInicial;
	TickType_t tiempoMedicionFinal;
	TickType_t tiempoEntreEventos;

	 gpioWrite( LED3, OFF );

	while(TRUE){
		// Recibo datos por la cola de medicion de tiempo
		xQueueReceive( queueMedicionTiempo, &buttonDataTemp, portMAX_DELAY );
		// Hay que verificar que ocurran eventos solapados
		// Si el boton presionado fue igual al anterior reinicio
		if(buttonDataTemp.buttonIndex == lastButtonPressed) {
			lastButtonPressed = _NONE;
			xSemaphoreGive(semaforo); // restablezco el semaforo
		}
		else{
			lastButtonPressed = buttonDataTemp.buttonIndex; // si son distintos tomo el semaforo
			if(xSemaphoreTake( semaforo , 0) == pdTRUE){
				// si devolvio pdTRUE entonces es el primer evento, es decir, la primer medicion
				tiempoMedicionInicial = buttonDataTemp.buttonTime;
				systemLogData.x = buttonDataTemp.buttonIndex;
			}
			else{
				// Si la respuesta es pdFALSE entonces es el segundo evento que ocurrio
				tiempoMedicionFinal = buttonDataTemp.buttonTime;
				tiempoEntreEventos = tiempoMedicionFinal - tiempoMedicionInicial;
				systemLogData.tiempoEntreEventos = tiempoEntreEventos;
				systemLogData.y = buttonDataTemp.buttonIndex;

				// Reseteo todas las mediciones
				//tiempoMedicionInicial = 0;
				//tiempoMedicionFinal = 0;
				//tiempoEntreEventos = 0;
				lastButtonPressed = _NONE;

				xSemaphoreGive(semaforo); // restablezco el semaforo

				xQueueSend(queueLogs, &systemLogData, portMAX_DELAY); // se envia la cola de logs

			}

		}
	}
}

// Implementacion de funcion de la tarea
void showLEDs( void* taskParmPtr ){

	buttonData_t buttonDataTemp;
	// ---------- CONFIGURACIONES ------------------------------
	gpioWrite( LED1, OFF );
	gpioWrite( LED2, OFF );

   while(TRUE) {
	   xQueueReceive( queueLaunchLeds, &buttonDataTemp, portMAX_DELAY );

	   switch(buttonDataTemp.buttonIndex){
	   case TECLA1:
		   gpioWrite( LED1, ON );
		   break;
	   case TECLA2:
		   gpioWrite( LED2, ON );
		   break;
	   default:
		   break;
	   }
	   vTaskDelay(DELAY_LED_BLINK/portTICK_RATE_MS);
	   gpioWrite( LED1, OFF );
	   gpioWrite( LED2, OFF );
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
					xQueueSend(queueLaunchLeds, &buttonDataTemp, portMAX_DELAY);	// envio la cola a la tarea que maneja el encendido de leds
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
				xQueueSend(queueLaunchLeds, &buttonDataTemp, portMAX_DELAY);	// envio la cola a la tarea que maneja el encendido de leds
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
