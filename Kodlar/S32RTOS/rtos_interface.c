/*
 * rtos_interface.c
 *
 *  Created on: 5 Aðu 2022
 *      Author: Ali
 */

#include "rtos_interface.h"
void delay(volatile int cycles)
{
    /* Delay function - do nothing for a number of cycles */
    while(cycles--);
}


void vTaskBlink( void * pvParameters )
{
	PINS_DRV_ClearPins(GPIO_PORT, 1 << LED1_PIN);
    for( ;; )
    {
    	PINS_DRV_TogglePins(GPIO_PORT, 1 << LED1_PIN);
    	delay(7200000);
        /* Task code goes here. */
    }
}


void start_rtos(void){
	status_t error;
	/* Configure clocks for PORT */
	error = CLOCK_DRV_Init(&clockMan1_InitConfig0);
	DEV_ASSERT(error == STATUS_SUCCESS);

	/* Set pins as GPIO */
	error = PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);
	DEV_ASSERT(error == STATUS_SUCCESS);

	xTaskCreate( vTaskBlink, "Task1", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY, NULL );

	vTaskStartScheduler();

	for( ;; );
}
