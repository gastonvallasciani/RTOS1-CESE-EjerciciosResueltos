//CONFIGURACION

    /* CONFIGURO ISR (2 HANDLERS PARA EL MISMO PIN) */

    /*Seteo la interrupción para el flanco descendente
     *                channel, GPIOx, [y]    <- no es la config del pin, sino el nombre interno de la señal
     *                      |  |      |
     *                      |  |      |    */
    Chip_SCU_GPIOIntPinSel( 0, 0,     4 );

    //Borra el pending de la IRQ
    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( 0 ) ); // INT0 (canal 0 -> hanlder GPIO0)

    //Selecciona activo por flanco
    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( 0 ) ); // INT0

    //Selecciona activo por flanco descendente
    Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH( 0 ) ); // INT0
  
    /*Seteo la interrupción para el flanco descendente
     *                channel, GPIOx, [y]    <- no es la config del pin, sino el nombre interno de la señal
     *                      |  |      |
     *                      |  |      |    */
    Chip_SCU_GPIOIntPinSel( 1, 0,     4 );

    //Borra el pending de la IRQ
    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( 1 ) ); // INT1  (canal 1 -> hanlder GPIO1)

    //Selecciona activo por flanco
    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( 1 ) ); // INT1

    //Selecciona activo por flanco descendente
    Chip_PININT_EnableIntHigh( LPC_GPIO_PIN_INT, PININTCH( 1 ) ); // INT1


 

    //Borra el clear pending de la IRQ y lo activa
    NVIC_ClearPendingIRQ( PIN_INT0_IRQn );
    NVIC_EnableIRQ( PIN_INT0_IRQn );

    NVIC_ClearPendingIRQ( PIN_INT1_IRQn );
    NVIC_EnableIRQ( PIN_INT1_IRQn );

	
	