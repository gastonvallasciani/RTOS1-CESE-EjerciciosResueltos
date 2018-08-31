/*
 * ENUNCIADO:
 *
 * Supongamos que estamos en el desarrollo de una aeronave que tendrá control
 * automático de altitud (cabeceo) que no permite otras maniobras durante su
 * operación, un control automático de guiñada y un sistema que nos permite
 * controlar el alabeo con las teclas 2 y 3 de la EDU-CIAA.
 /*==================[inclusiones]============================================*/

// sAPI header
#include "sapi.h"
#include "sapi_imu_mpu9250.h"

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/*==================[definiciones y macros]==================================*/

#define LARGO_COLA_BUTTONS 				10
#define LARGO_COLA_SYSTEM_LOG				10
#define LARGO_COLA_LAUNCH_LED_ALABEO	10
#define DELAY_UNTIL_MED_ALABEO 			200
#define DELAY_UNTIL_MED_TEMP 				1000
#define DELAY_UNTIL_BLINKY_LED_ALABEO	100
#define TIEMPO_MED_ALABEO 					3000
#define TIEMPO_BLINKY_LED_ALABEO 		1000
#define TIEMPO_ANTIREBOTE 					20
#define TEMP_MAX_CABINA 					30
#define INCLINACION_MAX						-7		// Valor aprox a cos (45°) * 9.8

/*==================[definiciones de datos internos]=========================*/

// If MPU9250 AD0 pin is connected to GND
MPU9250_address_t addr = MPU9250_ADDRESS_0;

SemaphoreHandle_t mutexUart;
SemaphoreHandle_t launchMedAlabeo;

QueueHandle_t queueButtons;
QueueHandle_t queueSystemLog;
QueueHandle_t queueLaunchLedAlabeo;

typedef enum {
	NONE, FALLING, RAISING
} buttonState_t;

typedef enum {
	IDLE, FIRST_FALLING, FIRST_RAISING
} buttonFirstEvent_t;

typedef enum {
	TECLA1, TECLA2, TECLA3, TECLA4
} buttonNum_t;

typedef struct {
	buttonNum_t buttonIndex;	// Funciona para indice tecla en FSM
	TickType_t buttonTime;		// Valor TickCount al momento del evento
	buttonState_t buttonState;	// Flanco de ocurrencia del evento
	buttonFirstEvent_t buttonFirstEvent;	// Indica si 1er evento fue F. o R.
	TickType_t buttonTimeFirstEvent;			// Almacena el tiempo del 1er evento
	buttonState_t buttonLastState;			// Indica ultimo estado de tecla
} buttonData_t;

typedef struct {
	uint8_t modulo[9];			// Modulo de SW que envia msj
	//uint8_t valor[10];			// Valor a imprimir
	float valor;
} systemLogData_t;

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Tasks
static void taskTeclas(void * taskParmPtr);
static void taskMedAlabeo(void * taskParmPtr);
static void taskMedTemp(void * taskParmPtr);
static void taskSystemLog(void * taskParmPtr);
static void taskLaunchLedAlabeo(void* taskParmPtr);

// Handlers ISR
void GPIO0_IRQHandler(void);
void GPIO1_IRQHandler(void);
void GPIO2_IRQHandler(void);
void GPIO3_IRQHandler(void);

// Otras funciones
static void isrConfig(void);
static void enableGPIOIrq(uint8_t irqChannel, uint8_t port, uint8_t pin, uint8_t edge);
static void checkButtonsDebounce(buttonData_t * buttonData);
static void buttonPressed(buttonNum_t buttonIndex);
static void buttonReleased(buttonNum_t buttonIndex);

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void) {

	int8_t imuStatus;

	// Inicializar y configurar la plataforma
	boardConfig();

	/* UART for debug messages */
	debugPrintConfigUart( UART_USB, 115200 );
	debugPrintlnString("Ejercicio Monitor de Navegacion.");

	/* Inicializo MPU */
	imuStatus = mpu9250Init(addr);

	if(imuStatus < 0) {
		printf("IMU MPU9250 no inicializado, chequee las conexiones:\r\n\r\n");
		printf("MPU9250 ---- EDU-CIAA-NXP\r\n\r\n");
		printf("    VCC ---- 3.3V\r\n");
		printf("    GND ---- GND\r\n");
		printf("    SCL ---- SCL\r\n");
		printf("    SDA ---- SDA\r\n");
		printf("    AD0 ---- GND\r\n\r\n");
		printf("Se detiene el programa.\r\n");
		while(1)
			;
	}
	printf("IMU MPU9250 inicializado correctamente.\r\n\r\n");

	/* Config ISRs */
	isrConfig();

	/* Creacion de semaforos (arrancan "tomado") */
	launchMedAlabeo = xSemaphoreCreateBinary();
	/* Creacion de mutexes (arrancan "liberados") */
	mutexUart = xSemaphoreCreateMutex();
	/* Creacion de colas */
	queueButtons = xQueueCreate(LARGO_COLA_BUTTONS, sizeof(buttonData_t));
	queueSystemLog = xQueueCreate(LARGO_COLA_SYSTEM_LOG, sizeof(systemLogData_t));
	queueLaunchLedAlabeo = xQueueCreate(LARGO_COLA_LAUNCH_LED_ALABEO, sizeof(bool_t));

	/* Creacion de tareas */
	xTaskCreate(taskTeclas, (const char *) "taskTeclas",
	configMINIMAL_STACK_SIZE * 2, 0,
	tskIDLE_PRIORITY + 2,			// Prioridad = + 2
	0);

	xTaskCreate(taskMedAlabeo, (const char *) "taskMedAlabeo",
	configMINIMAL_STACK_SIZE * 2, 0,
	tskIDLE_PRIORITY + 3,			// Prioridad = + 3
	0);

	xTaskCreate(taskMedTemp, (const char *) "taskMedTemp",
	configMINIMAL_STACK_SIZE * 4, 0,
	tskIDLE_PRIORITY + 1,			// Prioridad = + 1
	0);

	xTaskCreate(taskSystemLog, (const char *) "taskSystemLog",
	configMINIMAL_STACK_SIZE * 2, 0,
	tskIDLE_PRIORITY + 1,			// Prioridad = + 1
	0);

	xTaskCreate(taskLaunchLedAlabeo, (const char *) "taskLaunchLedAlabeo",
	configMINIMAL_STACK_SIZE * 2, 0,
	tskIDLE_PRIORITY + 1,			// Prioridad = + 2
	0);

	// Iniciar scheduler
	vTaskStartScheduler();

	// ---------- REPETIR POR SIEMPRE --------------------------
	while( TRUE) {
		// Si cae en este while 1 significa que no pudo iniciar el scheduler
	}
	return 0;
}

/*==================[definiciones de funciones externas]=====================*/

static void checkButtonsDebounce(buttonData_t * buttonData) {

	/*
	 * El antirebote se diseña considerando que si se presiona la tecla, el 1er
	 * evento es de FALLING, y en caso de que luego de 20ms la tecla efectivamen-
	 * te se presiono, entonces el ultimo evento dentro de esa ventana de 20ms
	 * debe ser tambien de FALLING (si es de RAISING, entonces existio un falso
	 * contacto).
	 * El razonamiento analogo es para el caso de liberarse la tecla, siendo el
	 * 1er evento de RAISING... el ultimo evento dentro de la ventana de 20ms
	 * tambien debe ser de RAISING, sino existio un falso contacto.
	 */
	switch(buttonData->buttonState) {

	case FALLING:
		/* Verifico si es el 1er evento de falling de la secuencia antirebote */
		if(buttonData->buttonFirstEvent == IDLE) {
			/* Flag */
			buttonData->buttonFirstEvent = FIRST_FALLING;
			/* Guardo valor de cuenta actual */
			buttonData->buttonTimeFirstEvent = buttonData->buttonTime;
		}

		/* Detecto boton presionado */
		if(buttonData->buttonFirstEvent == FIRST_FALLING) {

			if((xTaskGetTickCount() - buttonData->buttonTimeFirstEvent) < (TIEMPO_ANTIREBOTE / portTICK_RATE_MS)) {
				/* Guardo ultimo estado de tecla */
				buttonData->buttonLastState = buttonData->buttonState;

			} else {
				/* > 20 ms */
				if(buttonData->buttonLastState == FALLING) {
					/* Tecla presionada efectivamente */
					buttonPressed(buttonData->buttonIndex);
				} else {
					/* Existio falso contacto / rebote */
				}
				/* Reestablezco estado */
				buttonData->buttonState = NONE;
				buttonData->buttonFirstEvent = IDLE;
			}
		}
		break;

	case RAISING:
		/* Verifico si es el 1er evento de raising de la secuencia antirebote */
		if(buttonData->buttonFirstEvent == IDLE) {
			/* Flag */
			buttonData->buttonFirstEvent = FIRST_RAISING;
			/* Guardo valor de cuenta actual */
			buttonData->buttonTimeFirstEvent = buttonData->buttonTime;
		}

		/* Detecto boton liberado */
		if(buttonData->buttonFirstEvent == FIRST_RAISING) {

			if((xTaskGetTickCount() - buttonData->buttonTimeFirstEvent) < (TIEMPO_ANTIREBOTE / portTICK_RATE_MS)) {
				/* Guardo ultimo estado de tecla */
				buttonData->buttonLastState = buttonData->buttonState;

			} else {
				/* > 20 ms */
				if(buttonData->buttonLastState == RAISING) {
					/* Tecla liberada efectivamente */
					buttonReleased(buttonData->buttonIndex);
				} else {
					/* Existio falso contacto / rebote */
				}
				/* Reestablezco estado */
				buttonData->buttonState = NONE;
				buttonData->buttonFirstEvent = IDLE;
			}
		}
		break;

	case NONE:
	default:
		/*
		 * Condicion de error, reinicio la MEF
		 * Case NONE: do nothing
		 */
		break;
	}
}

static void buttonPressed(buttonNum_t buttonIndex) {

	/* Tomo mutex UART */
	xSemaphoreTake(mutexUart, portMAX_DELAY);

	switch(buttonIndex) {

	case TECLA1:
		debugPrintlnString("DEBUG: TECLA1 presionada");
		break;

	case TECLA2:
		debugPrintlnString("DEBUG: TECLA2 presionada");

		/* Libero semaforo */
		xSemaphoreGive(launchMedAlabeo);
		break;

	case TECLA3:
		debugPrintlnString("DEBUG: TECLA3 presionada");

		/* Libero semaforo */
		xSemaphoreGive(launchMedAlabeo);
		break;

	case TECLA4:
		debugPrintlnString("DEBUG: TECLA4 presionada");
		break;

	default:
		break;
	}
	/* Libero mutex */
	xSemaphoreGive(mutexUart);
}

static void buttonReleased(buttonNum_t buttonIndex) {

	/* Tomo mutex UART */
	xSemaphoreTake(mutexUart, portMAX_DELAY);

	switch(buttonIndex) {

	case TECLA1:
		debugPrintlnString("DEBUG: TECLA1 liberada");
		break;

	case TECLA2:
		debugPrintlnString("DEBUG: TECLA2 liberada");
		break;

	case TECLA3:
		debugPrintlnString("DEBUG: TECLA3 liberada");
		break;

	case TECLA4:
		debugPrintlnString("DEBUG: TECLA4 liberada");
		break;

	default:
		break;
	}
	/* Libero mutex */
	xSemaphoreGive(mutexUart);
}

static void taskTeclas(void * taskParmPtr) {
	/*
	 * Revisa estado de teclas (alabeo) y dispara tarea de medicion que corres-
	 * ponde.
	 */

	static buttonData_t buttonDataTemp;   // Data temporal
	static buttonData_t buttonsData[4];   // Data de cada tecla
	static uint8_t i;
	static buttonNum_t lastButtonInt;

	/* Init estados teclas */
	for(i = 0; i < 4; i++) {
		buttonsData[i].buttonState = NONE;
		buttonsData[i].buttonFirstEvent = IDLE;
		buttonsData[i].buttonLastState = NONE;
		buttonsData[i].buttonIndex = i;   // TECLA1, 2, 3 y 4 respectivamente
	}

	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE) {

		/* Recibo elemento de la cola */
		if(xQueueReceive(queueButtons, &buttonDataTemp, TIEMPO_ANTIREBOTE/*portMAX_DELAY*/) == pdTRUE) {

			/* Guardo data de la tecla que corresponde */
			buttonsData[buttonDataTemp.buttonIndex].buttonState = buttonDataTemp.buttonState;
			buttonsData[buttonDataTemp.buttonIndex].buttonTime = buttonDataTemp.buttonTime;
			lastButtonInt = buttonDataTemp.buttonIndex;
		}


		/* Chequeo estado de teclas (+ debounce) y ejecuto buttonPressed */
		checkButtonsDebounce(&buttonsData[lastButtonInt/*buttonDataTemp.buttonIndex*/]);
	}
}

static void taskMedAlabeo(void* taskParmPtr) {
	/*
	 * Al hacer cualquier movimiento de alabeo, deberá medir la inclinación de
	 * la aeronave durante 3 segundos. Si la inclinación no supera los 45° en
	 * ningún momento, parpadeará el led G, si es igual o mayor a 45° parpadeará
	 * el led R. En ambos casos, el parpadeo durará 1 segundo.
	 */

	systemLogData_t tempSystemLogData;
	TickType_t tiempoInicioCiclo;
	TickType_t tiempoInicioMedicion;
	bool_t flagInclinacionMaxSuperada = FALSE;

	strcpy(tempSystemLogData.modulo, "MED_INCL");

	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE) {

		/* Tomo semaforo */
		xSemaphoreTake(launchMedAlabeo, portMAX_DELAY);

		/* Escribo por UART */
		xSemaphoreTake(mutexUart, portMAX_DELAY);debugPrintlnString("Inicio conteo 3 segundos");
		xSemaphoreGive(mutexUart);

		tiempoInicioCiclo = xTaskGetTickCount();
		tiempoInicioMedicion = xTaskGetTickCount();

		/* Mientras sea menor a 3 segundos */
		while((xTaskGetTickCount() - tiempoInicioMedicion) < TIEMPO_MED_ALABEO) {

			vTaskDelayUntil(&tiempoInicioCiclo, DELAY_UNTIL_MED_ALABEO);

			/*
			 * Mido el angulo de alabeo: 0,707 veces 9,8 m/s2 = 7 (aprox.), valor
			 * a medir sobre eje Z
			 */

			/* Leer el sensor y guardar en estructura de control */
			mpu9250Read();
			tempSystemLogData.valor = mpu9250GetAccelZ_mss();

			/* Activo flag si en alguna medicion se supero el valor max de incl.*/
			if(tempSystemLogData.valor > INCLINACION_MAX)
				flagInclinacionMaxSuperada = TRUE;

			/* Envio info a cola */
			xQueueSend(queueSystemLog, &tempSystemLogData, portMAX_DELAY);
		}

		/* Envio por cola data del flag */
		xQueueSend(queueLaunchLedAlabeo, &flagInclinacionMaxSuperada, portMAX_DELAY);
		/* Bajo flag si es que se activo */
		flagInclinacionMaxSuperada = FALSE;

	}
}

static void taskMedTemp(void * taskParmPtr) {
	/*
	 * Monitoreará constantemente la temperatura de cabina: si es mayor a 30°
	 * informará al piloto, dejando encendido el led 3 hasta que baje la misma.
	 */

	TickType_t tiempoInicioCiclo;
	static float tempMedida;
	systemLogData_t tempSystemLogData;

	tiempoInicioCiclo = xTaskGetTickCount();

	strcpy(tempSystemLogData.modulo, "MED_TEMP");

	while(TRUE) {

		vTaskDelayUntil(&tiempoInicioCiclo, DELAY_UNTIL_MED_TEMP);

		/* Leer el sensor y guardar en estructura de control */
		mpu9250Read();
		tempSystemLogData.valor = mpu9250GetTemperature_C();

		/* Envio info a cola */
		xQueueSend(queueSystemLog, &tempSystemLogData, portMAX_DELAY);

		/* Control del led de alerta TEMP_MAX */
		if(tempSystemLogData.valor > TEMP_MAX_CABINA) {
			gpioWrite(LED3, ON);
		} else {
			gpioWrite(LED3, OFF);
		}
	}
}

static void taskSystemLog(void * taskParmPtr) {
	/*
	 * Tendrá un sistema de Log que permitirá recibir mensajes de distintos
	 * módulos de SW y los imprimirá por UART con el formato [modulo],[valor]
	 */

	systemLogData_t tempSystemLogData;

	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE) {

		xQueueReceive(queueSystemLog, &tempSystemLogData, portMAX_DELAY);

		xSemaphoreTake(mutexUart, portMAX_DELAY);
		printf("%s,%.2f \r\n", tempSystemLogData.modulo, tempSystemLogData.valor);
		xSemaphoreGive(mutexUart);
	}
}

static void taskLaunchLedAlabeo(void* taskParmPtr) {
	/*
	 * Al hacer cualquier movimiento de alabeo, deberá medir la inclinación de
	 * la aeronave durante 3 segundos. Si la inclinación no supera los 45° en
	 * ningún momento, parpadeará el led G, si es igual o mayor a 45° parpadeará
	 * el led R. En ambos casos, el parpadeo durará 1 segundo.
	 */

	systemLogData_t tempSystemLogData;
	bool_t flagInclinacionMaxSuperada;
	TickType_t tiempoInicioCiclo;
	TickType_t tiempoInicioMedicion;

	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE) {

		/* Tomo semaforo */
		xQueueReceive(queueLaunchLedAlabeo, &flagInclinacionMaxSuperada, portMAX_DELAY);

		tiempoInicioCiclo = xTaskGetTickCount();
		tiempoInicioMedicion = xTaskGetTickCount();

		/* Mientras sea menor a 1 segundo */
		while((xTaskGetTickCount() - tiempoInicioMedicion) < TIEMPO_BLINKY_LED_ALABEO) {

			vTaskDelayUntil(&tiempoInicioCiclo, DELAY_UNTIL_BLINKY_LED_ALABEO);

			if(flagInclinacionMaxSuperada == TRUE) {
				gpioToggle(LEDR);
			} else {
				gpioToggle(LEDG);
			}
		}

		/* Garantizo apagar leds */
		gpioWrite(LEDR, OFF);
		gpioWrite(LEDG, OFF);
	}
}

void GPIO0_IRQHandler(void) {
	/*
	 * ISR TEC1
	 */

	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	buttonData_t tempData;

	/* Me fijo si la INT salto por flanco asc. o desc. */
	if(Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & 0x01) {
		tempData.buttonState = RAISING;
	} else {
		tempData.buttonState = FALLING;
	}

	/* Asigno info */
	tempData.buttonIndex = TECLA1;
	tempData.buttonTime = xTaskGetTickCountFromISR();

	/* Envio info a cola */
	xQueueSendFromISR(queueButtons, &tempData, &pxHigherPriorityTaskWoken);

	/* Clear flag ISR */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));

	/* If xHigherPriorityTaskWoken was set to true you we should yield */
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void GPIO1_IRQHandler(void) {
	/*
	 * ISR TEC2
	 * Alabeo
	 */

	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	buttonData_t tempData;

	/* Me fijo si la INT salto por flanco asc. o desc. */
	if(Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & 0x02) {
		tempData.buttonState = RAISING;
	} else {
		tempData.buttonState = FALLING;
	}

	/* Asigno info */
	tempData.buttonIndex = TECLA2;
	tempData.buttonTime = xTaskGetTickCountFromISR();

	/* Envio info a cola */
	xQueueSendFromISR(queueButtons, &tempData, &pxHigherPriorityTaskWoken);

	/* Clear flag ISR */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));

	/* If xHigherPriorityTaskWoken was set to true you we should yield */
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void GPIO2_IRQHandler(void) {
	/*
	 * ISR TEC3
	 * Alabeo
	 */

	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	buttonData_t tempData;

	/* Me fijo si la INT salto por flanco asc. o desc. */
	if(Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & 0x04) {
		tempData.buttonState = RAISING;
	} else {
		tempData.buttonState = FALLING;
	}

	/* Asigno info */
	tempData.buttonIndex = TECLA3;
	tempData.buttonTime = xTaskGetTickCountFromISR();

	/* Envio info a cola */
	xQueueSendFromISR(queueButtons, &tempData, &pxHigherPriorityTaskWoken);

	/* Clear flag ISR */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));

	/* If xHigherPriorityTaskWoken was set to true you we should yield */
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void GPIO3_IRQHandler(void) {
	/*
	 * ISR TEC4
	 */

	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	buttonData_t tempData;

	/* Me fijo si la INT salto por flanco asc. o desc. */
	if(Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & 0x08) {
		tempData.buttonState = RAISING;
	} else {
		tempData.buttonState = FALLING;
	}

	/* Asigno info */
	tempData.buttonIndex = TECLA4;
	tempData.buttonTime = xTaskGetTickCountFromISR();

	/* Envio info a cola */
	xQueueSendFromISR(queueButtons, &tempData, &pxHigherPriorityTaskWoken);

	/* Clear flag ISR */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(3));

	/* If xHigherPriorityTaskWoken was set to true you we should yield */
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

static void isrConfig(void) {

	/* IRQ Enable to capture sensor echo pulse edges */
	/*
	 enableGPIOIrq(0, ultrasonicSensors[aSensor].irqConfig.gpioInit.port,
	 ultrasonicSensors[aSensor].irqConfig.gpioInit.pin, BOTH_EDGES);


	 LUEGO en el handler hago un Chip_PININT_GetHighEnabled para obtener data
	 pero hay que hacerlo aplicando una mascara, ej 0x02, depende del channel?
	 Hacer lo mismo con Chip_PININT_GetLowEnabled(LPC_PIN_INT_T *pPININT) ???

	 O MEJOR LEER con Chip_PININT_GetIntStatus(LPC_PIN_INT_T *pPININT)
	 de cual salto la interrupcion!!!!
	 */

	/*
	 * CONFIGURO ISR (1 HANDLER PARA TODAS LAS GPIO):
	 * "y un sistema que nos permite controlar el alabeo con las teclas 2 y 3
	 * de la EDU-CIAA"
	 */

	/*
	 * Seteo la interrupción para el flanco descendente y ascendente para TEC1
	 *                channel, GPIOx, [y]    <- no es la config del pin, sino el nombre interno de la señal
	 *                      |  |      |
	 *                      |  |      |    */
	Chip_SCU_GPIOIntPinSel(0, 0, 4);
	// Borra el pending de la IRQ
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));   // INT0 (canal 0 -> GPIO0_IRQHandler)
	// Selecciona activo por flanco
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(0));
	// Selecciona activo por flanco DESCENDENTE y ASCENDENTE
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(0));
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(0));

	/* Interrupcion TEC2 flanco ASCENDENTE y DESCENDENTE p/ channel 1 (INT1) */
	Chip_SCU_GPIOIntPinSel(1, 0, 8);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(1));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(1));
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(1));

	/* Interrupcion TEC3 flanco ASCENDENTE y DESCENDENTE p/ channel 2 (INT2) */
	Chip_SCU_GPIOIntPinSel(2, 0, 9);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(2));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(2));
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(2));

	/* Interrupcion TEC4 flanco ASCENDENTE y DESCENDENTE p/ channel 3 (INT3) */
	Chip_SCU_GPIOIntPinSel(3, 1, 9);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(3));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(3));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(3));
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(3));

	/* Defino prioridades */
	NVIC_SetPriority(PIN_INT0_IRQn, 4);
	NVIC_SetPriority(PIN_INT1_IRQn, 4);
	NVIC_SetPriority(PIN_INT2_IRQn, 4);
	NVIC_SetPriority(PIN_INT3_IRQn, 4);

	/* Borra el clear pending de la IRQ y lo activa */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT2_IRQn);
	NVIC_EnableIRQ(PIN_INT2_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT3_IRQn);
	NVIC_EnableIRQ(PIN_INT3_IRQn);
}

static void enableGPIOIrq(uint8_t irqChannel, uint8_t port, uint8_t pin, uint8_t edge) {
	/*
	 * Select irq channel to handle a GPIO interrupt, using its port and pin to specify it
	 * From EduCiaa pin out spec: GPIO1[9] -> port 1 and pin 9
	 */
	Chip_SCU_GPIOIntPinSel(irqChannel, port, pin);
	/* Clear actual configured interrupt status */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(irqChannel));
	/* Set edge interrupt mode */
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(irqChannel));

	if(edge == RAISING) {
		/* Enable high edge gpio interrupt */
		Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(irqChannel));
	} else if(edge == FALLING) {
		/* Enable low edge gpio interrupt */
		Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(irqChannel));
	} else {
		/* Enable high and low edge */
		Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(irqChannel));
		Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(irqChannel));
	}

	/* Clear pending irq channel interrupts */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn + irqChannel);
	/* Enable irqChannel interrupt */
	NVIC_EnableIRQ(PIN_INT0_IRQn + irqChannel);
}

/*==================[fin del archivo]========================================*/
