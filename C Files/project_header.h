#include "MKL25Z4.h"                     // Device header
#include <stdio.h>
#include "math.h"

#define DEFAULT_SYSTEM_CLOCK 20971520u  // System clock rate 
#define servo_mod 1638
#define motor_mod 167

void PWM_init(void);
int Cnv_calculator(float duty, int mod);
void Servo_drive(int servo_no, int angle);
void Pwm_servo(int servo_no, int duty);
void Motor_init(void);
void Motor_drive(int direction, int speed);
int Speed_to_duty(int speed);
void UART_init(void);
int UART_Rx(void);
void UART_Tx(int c);
void Instruction_parse(int rx);
void LED_init(void);
void WDT_init(void);
void Service_COP_WDT(void);
void Systick_init(void);

volatile int serial_rx;
volatile int serial_count = 0;

volatile int motor_speed = 1;
volatile int motor_direction = 1;

const static int BOTTOM_SERVO_1_LIST[] = {0, 45, 90, 135, 180}; 
const static int KNEE_SERVO_6_LIST[] = {0, 45, 90, 135, 180};    
const static int ANKLE_SERVO_3_LIST[] = {0, 45, 90, 135, 180};   
const static int NECK_SERVO_2_LIST[] = {0, 45, 90, 135, 180}; 			 
const static int CLAW_SERVO_5_LIST[] = {0, 180}; 	 

void Instruction_parse(int rx){
	int inst, data;
	inst = (rx & 0xE0)>>5;                            // Masking the instruction
	data = (rx & 0x1C)>>2;                            // Masking the data
	
	switch(inst){
		
		case 1:                                       // Motor Direction
			motor_direction = data;
			Motor_drive(motor_direction, motor_speed);
			break;
		
		case 2:                                       // Claw		
			Servo_drive(4, data);
			break;

		case 3:                                       // Neck		
			Servo_drive(2, data);
			break;
		
		case 4:                                       // Ankle		
			Servo_drive(3, data);
			break;
		
		case 5:                                       // Knee		
			Servo_drive(6, data);
			break;
		
		case 6:                                       // Bottom		
			Servo_drive(5, data);
			break;

		case 7:                                       // Speed
			motor_speed = data;	
			Motor_drive(motor_direction, motor_speed);
			break;
		
		default:
			return;
	}	
}

void UART_init(){
	
	int divisor;
	int temp;

	SIM_SCGC4 |=  SIM_SCGC4_UART0_MASK;                // Enable clock to UART0
	SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;             // Clock source selection
	SIM_SOPT2 |=  SIM_SOPT2_UART0SRC(1);
	
	divisor = (DEFAULT_SYSTEM_CLOCK / 115200 ) / 16;   // BAUD rate calculation
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;	               // Turn on clock to A module
	PORTA_PCR1 = PORT_PCR_MUX(2);		                   // Set PTA1 to mux 2 [TX]
	PORTA_PCR2 = PORT_PCR_MUX(2);		                   // Set PTA2 to mux 2 [RX]
	
	UART0->C2 &=~ (UART_C2_TE_MASK | UART_C2_RE_MASK); // Register configuration
	UART0->C1 = 0;

	temp = UART0->BDH & ~(UART_BDH_SBR(0x1F));
	UART0->BDH = ( temp |  UART_BDH_SBR(((divisor & 0x1F00) >> 8)) );
	UART0->BDL = (uint16_t)(divisor & UART_BDL_SBR_MASK);

	UART0->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
	
	NVIC_SetPriority(UART0_IRQn, 2);                   // Mid Level Priority for UART Interrupt
	NVIC_ClearPendingIRQ(UART0_IRQn);                  // Clear UART Interrupt Requests
	NVIC_EnableIRQ(UART0_IRQn);                        // Enable UART Interrupt
	UART0->C2 |= UART_C2_RIE(1);                       // Start UART

}  



int UART_Rx(void){
	while(!(UART0->S1 & UART0_S1_RDRF_MASK));                      // Check if the recieve flag is HIGH
	return UART0 -> D;                                             // return the UART data
}
void UART_Tx(int c){
	while((UART0->S1 & UART0_S1_TDRE_MASK) != UART0_S1_TDRE_MASK); // Check if the transmit flag is HIGH
	UART0->D = c;                                                  // Transmit the data over the UART
}
void Motor_init(void){

	SIM->SCGC5 |=  SIM_SCGC5_PORTB_MASK;                // Enable clock to Port B
	
	
  PORT_PCR_REG(PORTB_BASE_PTR,0) = PORT_PCR_MUX(3);   // Configuring PTB-0 Mux (TPM1)

	PORT_PCR_REG(PORTB_BASE_PTR,1) = PORT_PCR_MUX(3);   // Configuring PTB-1 Mux (TPM1)	
	
  PORT_PCR_REG(PORTB_BASE_PTR,2) = PORT_PCR_MUX(3);   // Configuring PTB-2 Mux (TPM2)

	PORT_PCR_REG(PORTB_BASE_PTR,3) = PORT_PCR_MUX(3);   // Configuring PTB-0 Mux (TPM2)	
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;                  // Enable clock to TPM1
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;                  // Enable clock to TPM2
	
	SIM->SOPT2 |= 0x01000000;                           // Clock source selection
	
	TPM1->SC = 0;                                       // Disable Timers
	TPM2->SC = 0; 
	
	TPM1->CONTROLS[0].CnSC = 0x20 | 0x08;               // center-aligned, non inverted
	TPM1->CONTROLS[1].CnSC = 0x20 | 0x08; 
	
	TPM2->CONTROLS[0].CnSC = 0x20 | 0x08;
	TPM2->CONTROLS[1].CnSC = 0x20 | 0x08; 
}

void Motor_drive(int direction, int speed){
	
	int duty;
	int cnv,cnv2,cnv3,cnv4;
	duty = Speed_to_duty(speed);                       // Maps speed to duty
	TPM1->MOD = motor_mod;                             // Writes MOD value 333 to TPM1 & TPM2
	TPM2->MOD = motor_mod;
	switch(direction){
		
		case 0:                                        // Go Forward
			
			                                           // Motor Right
			cnv = Cnv_calculator(duty*1.15, motor_mod);// Converts duty to the CnV value
			TPM1->CONTROLS[0].CnV = cnv;               // Writes the CnV value to Channel 0
			cnv2 = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;              // Writes the CnV value to Channel 1
		
			                                           // Motor Left
			cnv3 = Cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF;                            // Starts TPM1 & TPM2
			TPM2->SC = 0xF;	
			break;
		
		case 2:                                        // Go Left
			

			cnv = Cnv_calculator(duty, motor_mod); 
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(0, motor_mod); 
			TPM1->CONTROLS[1].CnV = cnv2;
		

			cnv3 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;
		
		case 6:                                        //Go Right                                        // Go Right
			

			cnv = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		

			cnv3 = Cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;
		
		case 4:                                        // Go Backwards
			

			cnv = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		

			cnv3 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF;
			TPM2->SC = 0xF;	
			break;
		
		case 7:                                        // Go NorthWest
			

			cnv = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		

			cnv3 = Cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;	
		
		case 1:                                        // Go NorthEast
			

			cnv = Cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		

			cnv3 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 	
			TPM2->SC = 0xF;	
			break;
		
		case 5:                                        // Go SouthWest


			cnv = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;


			cnv3 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(duty, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;	
			break;
		
		case 3:                                        // Go SouthEast
			

			cnv = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(duty, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		

			cnv3 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF; 
			TPM2->SC = 0xF;
			break;
		
		case 10:                                        // STOP

			cnv = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		

			cnv3 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF;
			TPM2->SC = 0xF;		
			break; 
		default:                                        // STOP
			
			//Motor Left
			cnv = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[0].CnV = cnv;
			cnv2 = Cnv_calculator(0, motor_mod);
			TPM1->CONTROLS[1].CnV = cnv2;
		
			//Motor Right
			cnv3 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[0].CnV = cnv3;
			cnv4 = Cnv_calculator(0, motor_mod);
			TPM2->CONTROLS[1].CnV = cnv4;
		
			TPM1->SC = 0xF;
			TPM2->SC = 0xF;		
	}
}

int Speed_to_duty(int speed){

	if(speed == 1){return 50;}                         // Half of the speed
	else if(speed == 2){return 75;}                    // 3/4 of the speed
	else if(speed == 3){return 100;}                   // Full speed
	else{return 0;}                                    // Stop

}

void PWM_init(void){

	int x;
	SIM->SCGC5 |= 0x1000;                             // Enable clock to Port D

  PORTD->PCR[0] = 0x0400;                           // Configuring PTD-0 Mux (TPM0)

	PORTD->PCR[1] = 0x0400;                           // Configuring PTD-1 Mux (TPM0)

	PORTD->PCR[2] = 0x0400;                           // Configuring PTD-2 Mux (TPM0)

	PORTD->PCR[3] = 0x0400;                           // Configuring PTD-3 Mux (TPM0)

	PORTD->PCR[4] = 0x0400;                           // Configuring PTD-4 Mux (TPM0)

	PORTD->PCR[5] = 0x0400;                           // Configuring PTD-5 Mux (TPM0)	

	SIM->SCGC6 |= 0x01000000;                         // Enable clock to TPM0
	SIM->SOPT2 |= 0x01000000;                         // Clock source selection
	TPM0->SC = 0;                                     // Disable timer

	TPM0->CONTROLS[0].CnSC = 0x20 | 0x08;             // Center-aligned, non inverted
	TPM0->CONTROLS[1].CnSC = 0x20 | 0x08;             
	TPM0->CONTROLS[2].CnSC = 0x20 | 0x08;
	TPM0->CONTROLS[3].CnSC = 0x20 | 0x08;
	TPM0->CONTROLS[4].CnSC = 0x20 | 0x08;
	TPM0->CONTROLS[5].CnSC = 0x20 | 0x08;
                                                      // Initialize the servos
	Servo_drive(5, 2);                                // Bottom
	Servo_drive(6, 0);                                // Knee
	Servo_drive(3, 0);                                // Ankle
	Servo_drive(2, 2);                                // Neck
	Servo_drive(4, 0);                                // Claw

}
  
void Pwm_servo(int servo_no, int cnv){
	TPM0->MOD = servo_mod;                             // Write the MOD value 1638 to TPM0
	if(servo_no==1){TPM0->CONTROLS[0].CnV = cnv;}      // Writes the CnV values to the channels
	else if(servo_no==2){TPM0->CONTROLS[1].CnV = cnv;}
	else if(servo_no==3){TPM0->CONTROLS[2].CnV = cnv;}
	else if(servo_no==4){TPM0->CONTROLS[3].CnV = cnv;}
	else if(servo_no==5){TPM0->CONTROLS[4].CnV = cnv;}
	else if(servo_no==6){TPM0->CONTROLS[5].CnV = cnv;}
	TPM0->SC = 0xF;                                    // Start the timer
}    
	
int Cnv_calculator(float duty, int mod){
	int cnv;
	cnv = (duty*(mod))/100.0;                         // Calculate the CnV given the duty cycle and MOD
	return cnv;
}

void Servo_drive(int servo_no, int data){
	float duty = 0.0;
	int cnv = 0, angle;
		switch(servo_no){
			
			case 5:                                    // Bottom
					angle = BOTTOM_SERVO_1_LIST[data]; // Get the angle from the lookup table
					break;

			case 2:                                    // Neck
					angle = NECK_SERVO_2_LIST[data];   
					break;

			case 3:                                    // Ankle 
					angle = ANKLE_SERVO_3_LIST[data];
					break;

			case 4:                                    // Claw
					angle = CLAW_SERVO_5_LIST[data];
					break;

			case 6:
					angle = KNEE_SERVO_6_LIST[data];   // Knee
					break;
		} 
	duty = (5.*angle / 180 ) + 5 ;                     // Normalize angle to duty cycle
	cnv = Cnv_calculator(duty, servo_mod);             // Maps duty to CnV
	Pwm_servo(servo_no,cnv);                           // Sends the PWM signal to servos
}

void UART0_IRQHandler(void){	
    if ((UART0->S1 & UART_S1_RDRF_MASK))             // Check the reciever flag 
    {		
	    Service_COP_WDT();                             // Service watchdog
      serial_rx = UART0->D;                          // Save the UART data
		  Instruction_parse(serial_rx);                  // Send the data to instructon parser
    }
}
void WDT_init(void){
	SIM->COPC = SIM_COPC_COPT(3) &~ SIM_COPC_COPW_MASK &~ SIM_COPC_COPCLKS_MASK ; // Watchdog configuration
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
