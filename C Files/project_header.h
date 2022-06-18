#include "MKL25Z4.h"                    // Device header
#include <stdio.h>
#include "math.h"

#define DEFAULT_SYSTEM_CLOCK 20971520u /* Default System clock value */
#define servo_mod 3276
#define motor_mod 333

void PWM_init(void);
int cnv_calculator(float duty, int mod);
void servo_drive(int servo_no, int angle);
void pwm_servo(int servo_no, int duty);
void motor_init(void);
void motor_drive(int direction, int speed);
int speed_to_duty(int speed);
void UART_init(void);
int UART_Rx(void);
void UART_Tx(int c);
void instruction_parse(int rx);
void LED_init(void);
void WDT_init(void);
void Service_COP_WDT(void);
void Systick_init(void);

volatile int serial_rx;
volatile int serial_count = 0;

volatile int motor_speed = 1;
volatile int motor_direction = 1;

const static int BOTTOM_SERVO_1_LIST[] = {0, 67, 135, 202, 270}; // 0
const static int KNEE_SERVO_6_LIST[] = {0, 60, 120, 180, 240};    // 1
const static int ANKLE_SERVO_3_LIST[] = {0, 60, 120, 180, 240};    // 2
const static int NECK_SERVO_2_LIST[] = {0, 60, 120, 180, 240}; 			 // 3
const static int CLAW_SERVO_5_LIST[] = {0, 240}; 	 // 4

void instruction_parse(int rx){
	int inst, data;
	inst = (rx & 0xE0)>>5;
	data = (rx & 0x1C)>>2;
	
	switch(inst){
		
		case 1: // Motor Direction
			motor_direction = data;
			motor_drive(motor_direction, motor_speed);
			break;
		
		case 2: // CLAW		
			servo_drive(4, data);
			break;

		case 3: // NECK		
			//LED_init();
			servo_drive(2, data);
			break;
		
		case 4: // ANKLE		
			servo_drive(3, data);
			break;
		
		case 5: // KNEE		
			servo_drive(6, data);
			break;
		
		case 6: // BOTTOM		
			servo_drive(5, data);
			break;

		case 7: // SPEED
			motor_speed = data;	
			motor_drive(motor_direction, motor_speed);
			break;
		
		default:
			return;
	}	
}

void UART_init(){
	
	int divisor;
	int temp;
	
	
	SIM_SCGC4 |=  SIM_SCGC4_UART0_MASK;
	SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
	SIM_SOPT2 |=  SIM_SOPT2_UART0SRC(1);
	
	divisor = (DEFAULT_SYSTEM_CLOCK / 115200 ) / 16;
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;	// Turn on clock to A module
	PORTA_PCR1 = PORT_PCR_MUX(2);		// Set PTA1 to mux 2 [TX]
	PORTA_PCR2 = PORT_PCR_MUX(2);		// Set PTA2 to mux 2 [RX]
	
	UART0->C2 &=~ (UART_C2_TE_MASK | UART_C2_RE_MASK);
	UART0->C1 = 0;

	temp = UART0->BDH & ~(UART_BDH_SBR(0x1F));
	UART0->BDH = ( temp |  UART_BDH_SBR(((divisor & 0x1F00) >> 8)) );
	UART0->BDL = (uint16_t)(divisor & UART_BDL_SBR_MASK);

	UART0->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
	
	NVIC_SetPriority(UART0_IRQn, 2);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
	UART0->C2 |= UART_C2_RIE(1);

}



int UART_Rx(void){
	//while(!(UART0_S1 & 0x20));
	while(!(UART0->S1 & UART0_S1_RDRF_MASK));
	return UART0 -> D;
}
void UART_Tx(int c){
	while((UART0->S1 & UART0_S1_TDRE_MASK) != UART0_S1_TDRE_MASK);
	UART0->D = c;
}
void motor_init(void){

	SIM->SCGC5 |=  SIM_SCGC5_PORTB_MASK; /* enable clock to Port B */
	
	//PTB-0
  PORT_PCR_REG(PORTB_BASE_PTR,0) = PORT_PCR_MUX(3); /* PTB0 used by TPM1 */
	//PTB-1
	PORT_PCR_REG(PORTB_BASE_PTR,1) = PORT_PCR_MUX(3); /* PTB1 used by TPM1 */	
	
	//PTB-2
  PORT_PCR_REG(PORTB_BASE_PTR,2) = PORT_PCR_MUX(3); /* PTB2 used by TPM2 right motor */
	//PTB-3
	PORT_PCR_REG(PORTB_BASE_PTR,3) = PORT_PCR_MUX(3); /* PTB3 used by TPM2 */	
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; /* enable clock to TPM1 */
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK; /* enable clock to TPM2 */
	
	SIM->SOPT2 |= 0x01000000; /* use MCGFLLCLK as timer counter clock */
	
	TPM1->SC = 0; /* disable timer */
	TPM2->SC = 0; /* disable timer */
	
	TPM1->CONTROLS[0].CnSC = 0x20 | 0x08;
	TPM1->CONTROLS[1].CnSC = 0x20 | 0x08; 
	
	TPM2->CONTROLS[0].CnSC = 0x20 | 0x08;
	TPM2->CONTROLS[1].CnSC = 0x20 | 0x08; 
}

void motor_drive(int direction, int speed){
	
	int duty;
	int cnv,cnv2,cnv3,cnv4;
	duty = speed_to_duty(speed);
	TPM1->MOD = motor_mod;
	TPM2->MOD = motor_mod;
	switch(direction){
		
		case 0: // Go Forward
			
			//Motor Right
			cnv = cnv_calculator(duty*1.15, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Left
			cnv3 = cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;
		
		case 2: // Go L
			
			//Motor Right
			cnv = cnv_calculator(duty, motor_mod); //F
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(0, motor_mod); //B
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Left
			cnv3 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;
		
		case 6: // Go R
			
			//Motor R
			cnv = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor L
			cnv3 = cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;
		
		case 4: // Go Backwards
			
			//Motor Left
			cnv = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Right
			cnv3 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF;
			TPM2->SC = 0xF;	
			break;
		
		case 7: // Go NorthWest
			
			//Motor Left
			cnv = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Right
			cnv3 = cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;	
		
		case 1: // Go NorthEast
			
			//Motor R
			cnv = cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor L
			cnv3 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 	
			TPM2->SC = 0xF;	
			break;
		
		case 5: // Go SouthWest
			
			//Motor R
			cnv = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor L
			cnv3 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;
		
		case 3: // Go SouthEast
			
			//Motor Left
			cnv = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Right
			cnv3 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;
			break;
		
		case 10:
			//Motor Left
			cnv = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Right
			cnv3 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF;
			TPM2->SC = 0xF;		
			break; 
		default: // STOP
			
			//Motor Left
			cnv = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Right
			cnv3 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF;
			TPM2->SC = 0xF;		
	}
}

int speed_to_duty(int speed){

	if(speed == 1){return 50;}
	else if(speed == 2){return 75;}
	else if(speed == 3){return 100;}
	else{return 0;}
}


void PWM_init(void){
	int x;
	SIM->SCGC5 |= 0x1000; /* enable clock to Port D */
	
	//PTD-0
  PORTD->PCR[0] = 0x0400; /* PTD0 used by TPM0 */
	//PTD-1
	PORTD->PCR[1] = 0x0400; /* PTD1 used by TPM0 */	
	//PTD-2
	PORTD->PCR[2] = 0x0400; /* PTD2 used by TPM0 */
	//PTD-3
	PORTD->PCR[3] = 0x0400; /* PTD3 used by TPM0 */
	//PTD-4
	PORTD->PCR[4] = 0x0400; /* PTD4 used by TPM0 */
	//PTD-5
	PORTD->PCR[5] = 0x0400; /* PTD5 used by TPM0 */	

	SIM->SCGC6 |= 0x01000000; /* enable clock to TPM0 */
	SIM->SOPT2 |= 0x01000000; /* use MCGFLLCLK as timer counter clock */
	TPM0->SC = 0; /* disable timer */
	
	TPM0->CONTROLS[0].CnSC = 0x20 | 0x08;
	TPM0->CONTROLS[1].CnSC = 0x20 | 0x08; 
	TPM0->CONTROLS[2].CnSC = 0x20 | 0x08;
	TPM0->CONTROLS[3].CnSC = 0x20 | 0x08;
	TPM0->CONTROLS[4].CnSC = 0x20 | 0x08;
	TPM0->CONTROLS[5].CnSC = 0x20 | 0x08;
	/*
	for(x=0; x<2; x++){
		servo_drive(4, x);
		Systick_init();
	}*/
	servo_drive(5, 2);   //bottom
	servo_drive(6, 0);   //knee
	servo_drive(3, 0);   //ankle
	//servo_drive(2, 2);   //neck
	//servo_drive(4, 0);   //neck*/

}
void pwm_servo(int servo_no, int cnv){
	
	TPM0->MOD = servo_mod; 
	if(servo_no==1){TPM0->CONTROLS[0].CnV = cnv;}
	else if(servo_no==2){TPM0->CONTROLS[1].CnV = cnv;}
	else if(servo_no==3){TPM0->CONTROLS[2].CnV = cnv;}
	else if(servo_no==4){TPM0->CONTROLS[3].CnV = cnv;}
	else if(servo_no==5){TPM0->CONTROLS[4].CnV = cnv;}
	else if(servo_no==6){TPM0->CONTROLS[5].CnV = cnv;}
	TPM0->SC = 0xF; 
}
	
int cnv_calculator(float duty, int mod){
	int cnv;
	cnv = (duty*(mod+1.))/100.0;
	return cnv;
}

void servo_drive(int servo_no, int data){
	float duty = 0.0;
	int cnv = 0, angle;
		switch(servo_no){
			
			case 5:
					angle = BOTTOM_SERVO_1_LIST[data];
					break;
			case 2:
					angle = NECK_SERVO_2_LIST[data];
					break;
			case 3:
					angle = ANKLE_SERVO_3_LIST[data];
					break;
			case 4:
					angle = CLAW_SERVO_5_LIST[data];
					break;
			case 6:
					angle = KNEE_SERVO_6_LIST[data];
					break;
		} 
	duty = (5.*angle / 180 ) + 5 ;
	cnv = cnv_calculator(duty, servo_mod);
	pwm_servo(servo_no,cnv);
}

void UART0_IRQHandler(void){	
    if ((UART0->S1 & UART_S1_RDRF_MASK))
    {		
				Service_COP_WDT();
        serial_rx = UART0->D;
				instruction_parse(serial_rx);
    }
}
void WDT_init(void){
	SIM->COPC = SIM_COPC_COPT(3) &~ SIM_COPC_COPW_MASK &~ SIM_COPC_COPCLKS_MASK ; //LOW POWER CLOCK
}

void Service_COP_WDT(void){
	SIM->SRVCOP = 0x55;
	SIM->SRVCOP = 0xaa;
}
void Systick_init(void){
	SysTick->LOAD = (DEFAULT_SYSTEM_CLOCK/16);
	SysTick->VAL=0;
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	while(1){
		if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)){	
			return;
		}
	}
	
}
