
/*==================[inclusiones]============================================*/

#include "sapi.h"        // <= Biblioteca sAPI
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "isrPinLevel.h"

#define FREE 0x00
#define DESCENDENTE 0x02
#define ASCENDENTE 0x03

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/
extern QueueHandle_t cola;
/*==================[declaraciones de funciones internas]====================*/
/*==================[declaraciones de funciones externas]====================*/
void isrPinLevelConfig(void){

	//CONFIGURACION

	    /* CONFIGURO ISR (1 HANDLER PARA EL MISMO PIN) */

	    /*Seteo la interrupción para el flanco descendente
	     *                channel, GPIOx, [y]    <- no es la config del pin, sino el nombre interno de la señal
	     *                      |  |      |
	     *                      |  |      |    */
	    Chip_SCU_GPIOIntPinSel( 0, 0,     4 );


	    //Borra el pending de la IRQ
	    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( 0 )); // INT0 (canal 0 -> hanlder GPIO0)

	    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( 0 )); // INT0

	    Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH( 0 )); // INT0
	   	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH( 0 ));	// INT0

	    //Borra el clear pending de la IRQ y lo activa
	    NVIC_ClearPendingIRQ( PIN_INT0_IRQn );
	    NVIC_EnableIRQ( PIN_INT0_IRQn );


}

void GPIO0_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken;

	colaStruct1.tecIndex = 0x01;
	colaStruct1.countTick = xTaskGetTickCountFromISR();
	//colaStruct1.flanco = DESCENDENTE;
	 if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(0) ) {
		 colaStruct1.flanco = ASCENDENTE;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(0));
	 }
	 else{
		 colaStruct1.flanco = DESCENDENTE;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(0));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(cola, &colaStruct1, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH( 0 ));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}





/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
