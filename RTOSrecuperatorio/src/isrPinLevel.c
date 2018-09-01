
/*==================[inclusiones]============================================*/

#include "sapi.h"        // <= Biblioteca sAPI
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "isrPinLevel.h"
#include "global.h"

#define FREE 0x00
#define DESCENDENTE 0x02
#define ASCENDENTE 0x03

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/
extern QueueHandle_t buttonsInfo;
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

	buttonData_t buttonData;

	buttonData.buttonIndex = TECLA1;
	buttonData.buttonTime = xTaskGetTickCountFromISR();

	if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(0) ) {
		 buttonData.buttonState = RAISING;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(0));
	 }
	 else{
		 buttonData.buttonState = FALLING;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(0));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(buttonsInfo, &buttonData, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH( 0 ));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}

void GPIO1_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken;

	buttonData_t buttonData;

	buttonData.buttonIndex = TECLA2;
	buttonData.buttonTime = xTaskGetTickCountFromISR();

	 if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(1) ) {
		 buttonData.buttonState = RAISING;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(1));
	 }
	 else{
		 buttonData.buttonState = FALLING;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(1));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(buttonsInfo, &buttonData, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH( 1 ));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}

void GPIO2_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken;

	buttonData_t buttonData;

	buttonData.buttonIndex = TECLA3;
	buttonData.buttonTime = xTaskGetTickCountFromISR();

	 if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(2) ) {
		 buttonData.buttonState = RAISING;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(2));
	 }
	 else{
		 buttonData.buttonState = FALLING;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(1));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(buttonsInfo, &buttonData, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH( 2 ));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}

void GPIO3_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken;

	buttonData_t buttonData;

	buttonData.buttonIndex = TECLA4;
	buttonData.buttonTime = xTaskGetTickCountFromISR();

	 if ( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(3) ) {
		 buttonData.buttonState = RAISING;
		   Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT,PININTCH(3));
	 }
	 else{
		 buttonData.buttonState = FALLING;
		 Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT,PININTCH(3));
	 }

	pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(buttonsInfo, &buttonData, &pxHigherPriorityTaskWoken );

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(3));

	portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}


/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
