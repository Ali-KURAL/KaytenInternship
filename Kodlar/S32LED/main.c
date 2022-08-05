/*!
** Copyright 2020 NXP
** @file main.c
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


/* Including necessary configuration files. */
#include "sdk_project_config.h"

#define PCC_CLOCK	PCC_PORTE_CLOCK
#define LED0_PORT PTE
#define LED0_PIN  21
#define LED1_PORT PTE
#define LED1_PIN  23

void delay(volatile int cycles)
{
    /* Delay function - do nothing for a number of cycles */
    while(cycles--);
}



volatile int exit_code = 0;
/* User includes */

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
	 status_t error;
	 /* Configure clocks for PORT */
	 error = CLOCK_DRV_Init(&clockMan1_InitConfig0);
	 DEV_ASSERT(error == STATUS_SUCCESS);
	 /* Set pins as GPIO */
	 error = PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);
	 DEV_ASSERT(error == STATUS_SUCCESS);

	 /* Set Output value LED0 & LED1 */
	 PINS_DRV_SetPins(LED0_PORT, 1 << LED0_PIN);
	 PINS_DRV_ClearPins(LED1_PORT, 1 << LED1_PIN);
    for(;;)
    {
        PINS_DRV_TogglePins(LED0_PORT, 1 << LED0_PIN);
        PINS_DRV_TogglePins(LED1_PORT, 1 << LED1_PIN);
        if(exit_code != 0)
        {
            break;
        }
        delay(7200000);
    }
    return error;
}

/* END main */
/*!
** @}
*/
