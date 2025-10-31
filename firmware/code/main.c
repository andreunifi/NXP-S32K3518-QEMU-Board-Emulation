#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "uart.h"
#include "adc.h"

#define mainTASK_PRIORITY    ( tskIDLE_PRIORITY + 2 )

void vTaskFunction(void *pvParameters);

int main(int argc, char **argv){

	(void) argc;
	(void) argv;
	UART_init();
	ADC_init();

	xTaskCreate(
		// Function which implements the task
		vTaskFunction,
		// Name of the task (debug purposes, not used by the kernel)
		"Task1",
		// Stack to allocate to the task
		configMINIMAL_STACK_SIZE,
		// Parameter passed to the task. Not needed for Hello World example
		NULL,
		// Priority assigned to the task
		mainTASK_PRIORITY,
		// Task handle. Not required
		NULL
	);

	// Give control to the scheduler
	vTaskStartScheduler();

	// If everything ok should never reach here
    for( ; ; );

}

/* Task Function */
void vTaskFunction(void *pvParameters) {

	// Avoid warning about unused pvParameters
	(void) pvParameters;
	char message[32];

    for (;;) {

        // Task code: print a message
		sprintf(message, "prova %ld\n", ADC_read());
        UART_printf(message);

        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
