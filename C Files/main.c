#include "project_header.h"

int main(void){	
	WDT_init();
	Systick_init();
	__enable_irq();
	PWM_init();
	motor_init();
	UART_init();
	while (1){	
		Service_COP_WDT();
	}	
}	



