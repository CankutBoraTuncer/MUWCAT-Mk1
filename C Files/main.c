#include "project_header.h"

int main(void){	
	WDT_init();                                        // Watchdog init
	Systick_init();                                    // Systick init
	__enable_irq();                                    // Enable interrupts
	PWM_init();                                        // PWM init
	Motor_init();                                      // Motor init
	UART_init();                                       // UART init
	while (1){	
		Service_COP_WDT();                             // Service the watchdog
	}	
}



