/*
 * rtos_interface.h
 *
 *  Created on: 5 Aðu 2022
 *      Author: Ali
 */

#ifndef RTOS_INTERFACE_H_
#define RTOS_INTERFACE_H_

#include "sdk_project_config.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* SDK includes. */
#include "interrupt_manager.h"

// Definition of pins
#define PCC_CLOCK	PCC_PORTE_CLOCK
#define GPIO_PORT PTE
#define LED0_PIN  21
#define LED1_PIN  22
#define LED2_PIN  23

#define mainTASK_PRIORITY		( tskIDLE_PRIORITY + 0 )

void delay(volatile int cycles);
void vTaskCode( void * pvParameters );
void start_rtos(void);



#endif /* RTOS_INTERFACE_H_ */
