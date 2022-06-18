
# 🚀MUWCAT Mk1©️

This repo includes Cankut Bora Tuncer's EEE212 2021 Term Project Multipurpose Wifi Camera Tank MUWCAT📌. This project contains two main parts: Software💻 and Hardware⚙️. For the software it is mainly used C language📌. The main code resides at KL25Z Freedom board📌. ESP32 communicates with the mobile app (wifi) and KL25Z (UART)📌. With the mobile app the user can control the 2 12V DC motors and 5 5V servo motors📌. Moreover, ESP32 sends the camera feed to the mobile app where the video can be seen📌. Last but not least, an Arduino Nano is used for initiating the QC3 handshake used in power banks📌. Feel free to contact me via email bora.tuncer2002@gmail.com for further questions📌.  

![alt text](https://github.com/CankutBoraTuncer/MUWCAT-Mk1/blob/main/Misc/Img/IMG-20220530-WA0001.jpg?raw=true?=250x250)
## 📚Acknowledgements

 - [Cankut Bora Tuncer - Github](https://github.com/CankutBoraTuncer)
 - [Cankut Bora Tuncer - LinkedIn](https://www.linkedin.com/in/cankut-bora-tuncer-8317741b8)
 - [Cankut Bora Tuncer - Youtube](https://www.youtube.com/channel/UC07zxHsGqEjZgMl7d5t4L_Q)


## 💻Software 

The KL25Z Freedom Board is coded with Keil uVision 5 environment. As for ESP32 and Arduino Nano, the Arduino Programming Environment is used. Mobile App is created on MIT App Inventor.
The software part of this project can be divided into 4 parts:

   📍 KL25z Freedom Board Programming 
 
   📍 ESP32 Programming 

   📍 MIT App Inventor Programming

   📍 Arduino Nano Programming
## 💻KL25z Freedom Board Programming
The brain of the project resides at the Kl25z Freedom Board. It communicates with the ESP32 and drives the DC and servo motors.
## 📍Main
The main program is quite simplistic. The code is written in an interrupt fashion for portability. In the main program, only the initialization occurs and waits for the UART interrupt.
```bash
int main(void){	
	WDT_init();                                        // Watchdog init
	Systick_init();                                    // Systick init
	__enable_irq();                                    // Enable interrupts
	PWM_init();                                        // PWM init
	motor_init();                                      // Motor init
	UART_init();                                       // UART init
	while (1){	
		Service_COP_WDT();                             // Service the watchdog
	}	
}
```
## 📍UART
### 🔗UART initialization:
```bash
void UART_init(){
	
	int divisor;
	int temp;

	SIM_SCGC4 |=  SIM_SCGC4_UART0_MASK;                // Enable clock to UART0
	SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;             // Clock source selection
	SIM_SOPT2 |=  SIM_SOPT2_UART0SRC(1);
	
	divisor = (DEFAULT_SYSTEM_CLOCK / 115200 ) / 16;   // BAUD rate calculation
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;	               // Turn on clock to A module
	PORTA_PCR1 = PORT_PCR_MUX(2);		               // Set PTA1 to mux 2 [TX]
	PORTA_PCR2 = PORT_PCR_MUX(2);		               // Set PTA2 to mux 2 [RX]
	
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
```
### 🔗UART Interrupt:
```bash
void UART0_IRQHandler(void){	
    if ((UART0->S1 & UART_S1_RDRF_MASK))               // Check the reciever flag 
    {		
	    Service_COP_WDT();                             // Service watchdog
        serial_rx = UART0->D;                          // Save the UART data
		instruction_parse(serial_rx);                  // Send the data to instructon parser
    }
}
```
## 📍Intruction Parse
The first 3 bits from left contains the insturction, the other 3 contains the data. 

### 🔗Instruction Parse:
```bash
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
```
## 📍Servo Driver
### 🔗Servo (PWM) initialization:
```bash
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
}  
```
The servo drive function drives the servos according to the incoming data from UART

### 🔗Servo Drive:
```bash
void servo_drive(int servo_no, int data){
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
```
### 🔗PWM Servo:
```bash
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
```
### 🔗CnV Calculator:
```bash
int Cnv_calculator(float duty, int mod){
	int cnv;
	cnv = (duty*(mod))/100.0;                         // Calculate the CnV given the duty cycle and MOD
	return cnv;
}
```
## 📍DC Motor Driver
### 🔗Motor initialization:
```bash
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

```
Motor drive function send the PWM signal to the motor drivers. There are 8 different directions: 

- NW, N, NE, E, W, SW, S, SE

### 🔗Motor Drive:
```bash
void Motor_drive(int direction, int speed){
	
	int duty;
	int cnv,cnv2,cnv3,cnv4;
	duty = speed_to_duty(speed);                       // Maps speed to duty
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
```
### 🔗Speed To Duty:
```bash
int Speed_to_duty(int speed){

	if(speed == 1){return 50;}                         // Half of the speed
	else if(speed == 2){return 75;}                    // 3/4 of the speed
	else if(speed == 3){return 100;}                   // Full speed
	else{return 0;}                                    // Stop

}
```
## 📍Extras
The purpose of the watchdog is to reset the system if a software error or any error that halts the system occurs. In a decided interval the watchdog has to be serviced. Otherwise the watchdog will reset the system. Watchdog can be used when the hardware is open for errors and the hardware is fragile.
### 🔗 Watchdog Init
```bash
void WDT_init(void){
	SIM->COPC = SIM_COPC_COPT(3) &~ SIM_COPC_COPW_MASK &~ SIM_COPC_COPCLKS_MASK ; // Watchdog configuration
}
```
### 🔗 Watchdog Service
```bash
void Service_COP_WDT(void){
	SIM->SRVCOP = 0x55;
	SIM->SRVCOP = 0xaa;
}
```
Other than using the TPM's Systick timer can be used to create delay.
### 🔗 Systick Init
```bash
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
```
The UART can also check the flags in a polling fashion.
### 🔗 Uart Recieve
```bash
int UART_Rx(void){
	while(!(UART0->S1 & UART0_S1_RDRF_MASK));                      // Check if the recieve flag is HIGH
	return UART0 -> D;                                             // return the UART data
}
```
### 🔗 Uart Transmit
```bash
void UART_Tx(int c){
	while((UART0->S1 & UART0_S1_TDRE_MASK) != UART0_S1_TDRE_MASK); // Check if the transmit flag is HIGH
	UART0->D = c;                                                  // Transmit the data over the UART
}
```
## 💻ESP32 Programming
The other crucial element of this project is the wifi capability📌. The onboard wifi module on the ESP 32
is used to communicate with the mobile app📌. The web server is created by the ESP32 and mobile app sends the 
request to that server📌. The written code is modified version of the available example on the Arduino software, ESP32 -> Camera -> Camera Web Server📌.
In order to program the board, the necessary libraries has to downloaded📌. The selected board is the 'ESP32 Wrover Module'📌. The changes on the code are made on CameraWebServer.ino and app_httpd.cpp📌.
### 📍Camera Web Server
The SSID and Password has to be changed. I used the hotspot feature on my phone. Note: You have to change your MAC adress type from 'Randomized MAC' to 'Phone MAC'.
```bash
const char* ssid = "Bora's Phone";              // The visible name
const char* password = "********";              // The password
```
### 📍App Httpd 
The brain of the WebServer is here. I changed the html of the website and included buttons to send request at.
```bash
<br>
    <section id="buttons">
        <div id="controls" class="control-container">
            <table>
            <tr><td align="center"><button class="button button6" id="get-still">Image</button></td><td align="center"><button id="toggle-stream">Start</button></td><td></td></tr>
            <tr><td align="center"><button class="button button2" id="NW" onclick="fetch(document.location.origin+'/control?var=car&val=7');">NW</button></td><td align="center"><button class="button button2" id="N" onclick="fetch(document.location.origin+'/control?var=car&val=0');">N</button></td><td align="center"><button class="button button2" id="NE" onclick="fetch(document.location.origin+'/control?var=car&val=1');">NE</button></td><td></td></tr>
            <tr><td align="center"><button class="button button2" id="W" onclick="fetch(document.location.origin+'/control?var=car&val=6');">W</button></td><td align="center"></td><td align="center"><button class="button button2" id="E" onclick="fetch(document.location.origin+'/control?var=car&val=2');">E</button></td></tr>
            <tr><td align="center"><button class="button button2" id="SW" onclick="fetch(document.location.origin+'/control?var=car&val=5');">SW</button></td><td align="center"><button class="button button2" id="S" onclick="fetch(document.location.origin+'/control?var=car&val=4');">S</button></td><td align="center"><button class="button button2" id="SE" onclick="fetch(document.location.origin+'/control?var=car&val=3');">SE</button></td><td></td></tr>

            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=speed&val=0');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=speed&val=1');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=speed&val=2');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=speed&val=3');">Speed</button></td></tr>

            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=0');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=1');">Speed</button></td></tr>
                            
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=2');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=3');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=4');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=5');">Speed</button></td></tr>                  
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=6');">Speed</button></td></tr>

            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=7');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=8');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=9');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=10');">Speed</button></td></tr>                  
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=11');">Speed</button></td></tr>
            
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=12');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=13');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=14');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=15');">Speed</button></td></tr>                  
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=16');">Speed</button></td></tr>

            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=17');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=18');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=19');">Speed</button></td></tr>
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=20');">Speed</button></td></tr>                  
            <tr><td align="center"><button class="button button2" id="motor_speed" onclick="fetch(document.location.origin+'/control?var=arm&val=21');">Speed</button></td></tr>

            </table>       
        </div>
    </section>    
<br>
```
Furthermore, I added the code to encode the recieved request into the 8 bits of the UART.
```bash
if (!strcmp(variable, "speed"))
  { 
    serial |= (7 << 5);       //type
    serial |= (val << 2);     //data       
    Serial.write(serial);
    serial = 0;
    val = 0;
  }
  if (!strcmp(variable, "car")) {
    if((val<=8)&(val>=0)){
      serial |= (1 << 5);
      serial |= (val << 2); 
      Serial.write(serial);
      serial = 0;
      val = 0;
      }
  }
  if(!strcmp(variable, "arm")){
    if ((val >= 0) & (val <= 1)) {
      serial |= (2 << 5);       //type
      serial |= (val << 2);     //data       
      Serial.write(serial);
      serial = 0;
      val = 0;

    }
    else if ((val >= 2) & (val <= 6)) {
      val -= 2;
      
      serial |= (3 << 5);       //type
      serial |= (val << 2);     //data    
         
      Serial.write(serial);
      serial = 0;
      val = 0;
    }
    else if ((val >= 7) & (val <= 11)) {
      val -= 7;
      serial |= (4 << 5);       //type
      serial |= (val << 2);     //data       
      Serial.write(serial);
      serial = 0;
      val = 0;
    }
    else if ((val >= 12) & (val <= 16)) {
      val -= 12;
      serial |= (5 << 5);       //type
      serial |= (val << 2);     //data       
      Serial.write(serial);
      serial = 0;
      val = 0;
    }
    else if ((val >= 17) & (val <= 21)) {
      val -= 17;
      serial |= (6 << 5);       //type
      serial |= (val << 2);     //data       
      Serial.write(serial);
      serial = 0;
      val = 0;
    }          
  }
```
## 💻MIT App Inventor Programming
The app is created from the MIT App Inventor. It is fully customizable and you can change the project layout and
software easily. One crucial 
![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)
