/*
 * global.h
 *
 *  Created on: 30/8/2018
 *      Author: gastonvallasciani
 */

#ifndef MISPROYECTOS_RTOSEXAMEN_INC_GLOBAL_H_
#define MISPROYECTOS_RTOSEXAMEN_INC_GLOBAL_H_

#define DEBOUNCE_TIME 					20
#define LARGO_COLA_SYSTEM_LOG			10
#define LARGO_COLA_BUTTONS_INFO 		10
#define LARGO_COLA_MEDICION_TIEMPO		10
#define LARGO_COLA_LAUNCH_LEDS			10
#define DELAY_LED_BLINK					200

typedef enum {
	NONE, FALLING, RAISING
} buttonState_t;

typedef enum {
	IDLE, FIRST_FALLING, FIRST_RAISING
} buttonFirstEvent_t;

typedef enum {
	TECLA1, TECLA2, TECLA3, TECLA4, _NONE
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
	buttonNum_t x;
	buttonNum_t y;
	TickType_t tiempoEntreEventos;
} systemLogData_t;

#endif /* MISPROYECTOS_RTOSEXAMEN_INC_GLOBAL_H_ */
