
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
	    Chip_SCU_GPIOIntPinSel( 0, 0,     4 ); //TEC1


	    //Borra el pending de la IRQ
	    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( 0 )); // INT0 (canal 0 -> hanlder GPIO0)

	    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( 0 )); // INT0

	    Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH( 0 )); // INT0
	   	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH( 0 ));	// INT0

	   	Chip_SCU_GPIOIntPinSel( 1, 0,     8 ); //TEC2

	    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( 1 )); // INT1

	    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( 1 )); // INT1

	    Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH( 1 )); // INT1
	    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH( 1 ));	// INT1

	    Chip_SCU_GPIOIntPinSel( 2, 0,     9 ); //TEC3

	    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( 2 )); // INT2

	    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( 2 )); // INT2

	    Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH( 2 )); // INT2
	    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH( 2 ));	// INT2

	    Chip_SCU_GPIOIntPinSel( 3, 1,     9 ); //TEC4

	    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( 3 )); // INT3

	    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( 3 )); // INT3

	    Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH( 3 )); // INT3
	   Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH( 3 ));	// INT3


	    //Borra el clear pending de la IRQ y lo activa
	    NVIC_ClearPendingIRQ( PIN_INT0_IRQn );
	    NVIC_EnableIRQ( PIN_INT0_IRQn );

	    //Borra el clear pending de la IRQ y lo activa
	    NVIC_ClearPendingIRQ( PIN_INT1_IRQn );
	    NVIC_EnableIRQ( PIN_INT1_IRQn );

	    //Borra el clear pending de la IRQ y lo activa
	    NVIC_ClearPendingIRQ( PIN_INT2_IRQn );
	    NVIC_EnableIRQ( PIN_INT2_IRQn );

	    //Borra el clear pending de la IRQ y lo activa
	    NVIC_ClearPendingIRQ( PIN_INT3_IRQn );
	    NVIC_EnableIRQ( PIN_INT3_IRQn );

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

void GPIO1_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken;

	colaStruct1.tecIndex = 0x02;
	colaStruct1.countTick = xTaskGetTickCountFromISR();
	//colaStruct1.flanco = DESCENDENTE;
	 if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(1) ) {
		 colaStruct1.flanco = ASCENDENTE;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(1));
	 }
	 else{
		 colaStruct1.flanco = DESCENDENTE;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(1));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(cola, &colaStruct1, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH( 1 ));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}

void GPIO2_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken;

	colaStruct1.tecIndex = 0x03;
	colaStruct1.countTick = xTaskGetTickCountFromISR();
	//colaStruct1.flanco = DESCENDENTE;
	 if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(2) ) {
		 colaStruct1.flanco = ASCENDENTE;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(2));
	 }
	 else{
		 colaStruct1.flanco = DESCENDENTE;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(1));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(cola, &colaStruct1, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH( 2 ));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}

void GPIO3_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken;

	colaStruct1.tecIndex = 0x04;
	colaStruct1.countTick = xTaskGetTickCountFromISR();
	//colaStruct1.flanco = DESCENDENTE;
	 if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(3) ) {
		 colaStruct1.flanco = ASCENDENTE;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(3));
	 }
	 else{
		 colaStruct1.flanco = DESCENDENTE;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(3));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(cola, &colaStruct1, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(3));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}


/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
