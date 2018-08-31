/* Author: Gaston Vallasciani
 * Materia: Sistemas operativos de tiempo real 1
 * Fecha: 12/08/2018
 *
 * Ejercicio 1: HearthBeat
 *
 * Para comenzar implementaremos un Heartbeat en el led azul de la EDU-CIAA,
 * utilizando FreeRTOS, donde el período sea de 1s y el led cambie de estado
 * cada 500ms
 *
 */

/*==================[inclusiones]============================================*/
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

// sAPI header
#include "sapi.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void hearthBeat( void* taskParmPtr );

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

   // Led para dar se�al de vida
   gpioWrite( LED3, ON );

   // Crear tarea en freeRTOS
   xTaskCreate(
	  hearthBeat,                     // Funcion de la tarea a ejecutar
      (const char *)"hearthBeat",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, 	  // Cantidad de stack de la tarea
      0,                              // Parametros de tarea
      tskIDLE_PRIORITY+1,             // Prioridad de la tarea
      0                               // Puntero a la tarea creada en el sistema
   );

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

// Implementacion de funcion de la tarea
void hearthBeat( void* taskParmPtr )
{
	TickType_t tiempo_inicio_ciclo;
	//tiempo_inicio_ciclo = xTaskGetTickCount();
	gpioWrite( LEDB, OFF );
	uint16_t contDelay = 1;

   // ---------- CONFIGURACIONES ------------------------------
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
      // Intercambia el estado del LEDB
      gpioToggle( LEDB );
	  vTaskDelay( 500 / portTICK_RATE_MS );
   }
}

/*==================[fin del archivo]========================================*/
