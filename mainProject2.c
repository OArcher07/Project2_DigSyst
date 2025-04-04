/*
 * Project2_us.c
 *
 * Created: 11/03/2025 17:37:30
 * Author : Ciaran.MacNamee
 * Modified By: Amelia Humphrey & Olivia Archer
 */ 


 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <avr/cpufunc.h>
 
 #define F_CPU 20000000
 #define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
 #define TARGET_BAUD_RATE 115200
 
 #define TCB_INTERVAL 1000000
 #define TCB_TOP 50000
 #define TCB_COUNT (TCB_INTERVAL/TCB_TOP)
 
 #include <util/delay.h>
 #include <stdio.h>
 
 // define voltages
 #define ZERO_5V 102
 #define ONE_0V	205
 #define ONE_5V	307
 #define TWO_0V	409
 #define TWO_5V	512
 #define THREE_0V 614
 #define THREE_5V 716
 #define FOUR_0V	818
 #define FOUR_5V 921
 #define FIVE_0V 1023


 /* Use a struct to make the association between PORTs and bits connected to the LED array more explicit */
 struct LED_BITS
 {
	PORT_t *LED_PORT;
	uint8_t bit_mapping;
 };

 struct LED_BITS LED_Array[10] = {
 {&PORTC, PIN5_bm}, {&PORTC, PIN4_bm}, {&PORTA, PIN0_bm}, {&PORTF, PIN5_bm}, {&PORTC, PIN6_bm}, {&PORTB, PIN2_bm}, {&PORTF, PIN4_bm}, {&PORTA, PIN1_bm}, {&PORTA, PIN2_bm}, {&PORTA, PIN3_bm}
 };
 
 uint8_t qcntr = 0, sndcntr = 0;
 unsigned char queue[50];

 uint8_t newDistanceData, newTimeData, newADC0Data;
 uint16_t adc_reading;		/* ADC0 RES has 10-bits, read it into a 16-bit variable */
 uint8_t ServoFollowADC;    /* Servo position based on ADC0 RES value */


void CLOCK_init (void);
void InitialiseLED_PORT_bits(void);
void Initialise_TCA0_SS_PWM(void);
void Initialise_EVSYS (void);
void Initialise_TCB0_ICP_PW(void);
void Initialise_TCB2_ICP_PWFRQ(void);
void Set_Clear_Ports(uint8_t set);
void USART3_init(void);
void ADC0_init(void);
void sendmsg(char* s); 

/* Later add TCB1 initialisation to detect 555 oscillation stopped */

 void CLOCK_init (void)
 {
	/* Disable CLK_PER Prescaler */
	ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp));
	/* If set from the fuses during device programming, the CPU will now run at 20MHz (default is /6) */
 }

 void InitialiseLED_PORT_bits()
 {
	PORTC.DIRSET = PIN6_bm | PIN5_bm | PIN4_bm;  /*(1<<6) | (1<<5) | (1<<4); 0x70;*/		/* PC4-UNO D1 (TXD1), PC5-UNO D0 (RXD1), PC6 - UNO D4  */
	PORTA.DIRSET = PIN3_bm | PIN2_bm | PIN1_bm | PIN0_bm; /*(1<<1) | (1<<0);   0x0f; */      /* PA1-UNO D7, PA0 - UNO D2, PA2- LED8, PA3 - LED9  */
	PORTB.DIRSET = PIN2_bm; /*0x04;*/		/* PB2 - UNO D5 */
	PORTF.DIRSET = PIN5_bm | PIN4_bm; /*(1<<5) | (1<<4);   0x30; */		/* PF5 - UNO D3, PF4 UNO D6 */
	/* Later use PIN6_bm etc */
 }

void USART3_init(void) {
	PORTB.DIRCLR = PIN5_bm;
	PORTB.DIRSET = PIN4_bm;
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(TARGET_BAUD_RATE);
	USART3.CTRLB = (USART_TXEN_bm | USART_RXEN_bm);
	PORTMUX.USARTROUTEA |= PORTMUX_USART3_ALT1_gc;
	USART3.CTRLA = USART_TXCIE_bm;
}

void Initialise_TCA0_SS_PWM()
{
	/* Make PORTA Bit 0 an output (may be done in InitialiseLED_PORT_bits())
	Set TCA0 to Single Slope PWM (CTRLB)
	Set TCA.SINGLE.PER or PERBUF for 50Hz PWM frequency (24999)
	Set TCA0.SINGLE.CMP0 for nominal -90degrees initial position – On time = 1ms
	Timer/Counter TCA0 Clock Source: CLK_PER divided by 16 and TCA0 enabled (CTRLA)
	(These are suggested settings – you may use your own if you can make them work) */
	
	PORTA.DIRSET = 0b00000001; // enables bit 0 as output
	TCA0.SINGLE.CTRLA = 0b0001001; // DIV16 selected and TCA0 enabled
	TCA0.SINGLE.CTRLB = 0b00000011; // Single Slope PWM
	TCA0.SINGLE.PER = 24999; // 50Hz PWM frequency --> 20ms
	TCA0.SINGLE.CMP0 = 1250; // (20ms * 0.1) = 1ms --> -90 degrees
	
}
 void Initialise_EVSYS()
 {
	/* Set Port B Pin 0 as input event this is on Channel 0 */
	PORTB.DIRCLR = 0b00000001; // clears bit 0, setting PIN0 to input
	/* Connect user to event channel 0  */
	EVSYS.CHANNEL0 = 0b01001000;
	/* TCB0 is the Channel 0 User */
	EVSYS.USERTCB0 = 0b00000001;
	
	/* Set Port E Pin 3 (PE3) as input event this is on Channel 4 */
	PORTE.DIRCLR = 0b00000011; // clears bits 0 and 1, setting PIN3 to input
	/* Connect user to event channel 4 */
	EVSYS.CHANNEL4 = 0b01000011;
	/* TCB2 is the Channel 4 user */
	EVSYS.USERTCB2 = EVSYS_CHANNEL_CHANNEL4_gc;
	
	/* Set TCB3 as the Generator for any other Channel */
	EVSYS.CHANNEL3 = 0b10100110;
	/* ADC0 is the user of the Channel selected for TCB3 Generator */
	EVSYS.USERADC0 = 0b00000100;    // Channel selected is n-1
									// 0b00000011 selects Channel 2
									// 0b00000100 selects Channel 3
	/* TCB3 starts ADC0  */
	/* Assuming enable when TCB3 is enabled */
}

 void Initialise_TCB0_ICP_PW()
 {
	/* Enable TCB0 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	TCB0.CTRLA = 0b00000010; // not enabled yet
	/* Configure TCB0 in Input Capture Pulse Width mode */
	TCB0.CTRLB = 0b00000100; // PW CNTMODE selected
 	/* Enable Capture or Timeout interrupt */
	TCB0.INTCTRL = 0b00000001;
 	/* Enable Event Input and Event Edge, Rising Edge selected */
	TCB0.EVCTRL = 0b00000001; // bit 4 = 0 for positive edge event input capture
						      // bit 0 = 1 enables capture event input
	/* Hint: consult TCB0_ICP_PW_Time_Ex.c */
	
	/* Enables TCB0 */
	TCB0.CTRLA |= 0b00000001;
 }
 
 void Initialise_TCB2_ICP_PWFRQ()
 {
	 /* Enable TCB2 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	 TCB2.CTRLA = 0b00000010; // not yet enabled
	 /* Configure TCB0 in Input Capture Clock Frequency Measurement mode */
	 TCB2.CTRLB = 0b00000011; // FRQ CNTMODE selected
	 /* Enable Capture or Timeout interrupt */
	 TCB2.INTCTRL = 0b00000001;
	 /* Enable Event Input and Event Edge, Rising Edge selected */
	 TCB2.EVCTRL = 0b00000001; // bit 4 = 0 for positive edge event input capture
							   // bit 0 = 1 enables capture event input
	 /* Hint: consult TCB0_ICP_PWFr_Time_Ex.c */
	 
	 /* Enables TCB2 */
	 TCB2.CTRLA |= 0b00000001;
 }
 
 void TCB3_init(void)
 {
	 /* enable overflow interrupt */
	 TCB3.INTCTRL = 0b00000001;
	 /* PER divided by 2 and Enable the TCB3 */
	 TCB3.CTRLA = 0b00000010; // not yet enabled
	 /* Periodic Interrupt Mode */
	 TCB3.CTRLB = 0b00000000;
	 /* Set TCB3.CCMP for 5ms interrupt rate */
	 // f(TCB) = 20MHz / 2 = 10MHz
	 // 5ms = 5000us, 5000us / (1/10MHz) = 50000us
	 TCB3.CCMP = 50000;
	 /* Enable the interrupt */
	 TCB3.INTCTRL = 0b00000001;
	 
	 /* Enables TCB3 */
	 TCB3.CTRLA |= 0b00000001;
 }

 void ADC0_init(void)
 {
	 /* CTRLA: 10-bit resolution selected, Free Running Mode NOT selected, ADC0 not enabled yet */
	 ADC0.CTRLA = 0b00000000;
	 /* CTRLB: Simple No Accumulation operation selected, this line could be omitted */
	 ADC0.CTRLB = 0b00000000;
	 /* CTRLC: SAMPCAP=1; REFSEL: VDD; PRESC set to DIV128 */
	 ADC0.CTRLC = 0b01010110;
	 /* CTRLD: INITDLY set to 16 CLK_ADC cycles */
	 ADC0.CTRLD = 0b00100000;
	 /* MUXPOS: Select AIN3 (shared with PORTD3), decision based on the Shield and adapters we use */
	 ADC0.MUXPOS = 0b00000011;
	 /* EVCTRL: STARTEI set to 1  */
	 ADC0.EVCTRL = (1<<0);
	 /* INTCTRL: Enable an interrupt when conversion complete (RESRDY) */
	 ADC0.INTCTRL = (1<<0);
	 /* Enable ADC0 and leave the other CTRLA bits unchanged, note |= */
	 ADC0.CTRLA |= 0b00000001;	
	 //ADC0.COMMAND = 1;
}
 
 

 int main(void)
 {
	char ch;
	char str_buffer[60];
	 
	uint8_t continuousDistance = 0;
	uint8_t continuousTime = 0;
	uint8_t continuousVolts = 0;
	 
	ServoFollowADC = 0;	 
	 
	newDistanceData = 0;
	newTimeData = 0;
	newADC0Data = 0;
	 
	CLOCK_init();
 
	/* set UNO D0-D7 to all outputs, also LED8 and LED9  */
	InitialiseLED_PORT_bits();
 
	Set_Clear_Ports(0);		/* Initialise LEDS to all OFF */
 
	Initialise_TCA0_SS_PWM();
	Initialise_EVSYS();
//	Initialise_TCB0_ICP_PW();
	USART3_init();
//	Initialise_TCB2_ICP_PWFRQ();
	TCB3_init();
	ADC0_init();
	
	sei(); /* Enable Global Interrupts */
 
	while (1) {
		if (USART3.STATUS & USART_RXCIF_bm) {
			ch = USART3.RXDATAL;
			
			switch (ch) {
				case 'a': 
				case 'A':
					sprintf(str_buffer, "ADC0 RES = %d\n", adc_reading);
					sendmsg(str_buffer);
					break;
				case 'v': 
				case 'V':
					/* Calculate milliVolts using integer arithmetic and send to user */
					break;
				case 't': 
				case 'T':
					/* Calculate 555 Time Period in us and send to user */
					break;
				case 'H': 
				case 'h':
					/* Calculate 555 High Pulse time in us and send to user */
					break;
				case 'L': 
				case 'l':
					/* Calculate 555 Low Pulse time in us and send to user */
					break;
				case 'd': 
				case 'D':
					/* Calculate distance for HC-SR04 Sensor to and object and 
						report to user */
					break;
				case 's': 
				case 'S':
					continuousDistance = 1;
					sprintf(str_buffer, "Continuous Distance ON\n");
					sendmsg(str_buffer);
					break;
				case 'u': 
				case 'U':
					continuousDistance = 0;
					sprintf(str_buffer, "Continuous Distance OFF\n");
					sendmsg(str_buffer);
					break;
				case 'c': 
				case 'C':
					continuousTime = 1;
					sprintf(str_buffer, "Continuous Time ON\n");
					sendmsg(str_buffer);
					break;
				case 'e': 
				case 'E':
					continuousTime = 0;
					sprintf(str_buffer, "Continuous Time OFF\n");
					sendmsg(str_buffer);
					break;
				case 'm': 
				case 'M':
					continuousVolts = 1;
					sprintf(str_buffer, "Continuous Volts ON\n");
					sendmsg(str_buffer);
					break;
				case 'n': 
				case 'N':
					continuousVolts = 0;
					sprintf(str_buffer, "Continuous Volts OFF\n");
					sendmsg(str_buffer);
					break;
				case 'f':
				case 'F':
					ServoFollowADC = 1;
					/* Set the Servo mode to follow ADC0 RES */
					break;
				case 'g':
				case 'G':
					ServoFollowADC = 0;
					/* Set the Servo mode move at a user selected */
					break;
				case '0':
					/* Stop the Servomotor from moving */
					break;
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
					/* Set the servomotor speed based on the specification table */
					break;
				default:
					sprintf(str_buffer, "Unrecognized input: %c\n", ch);
					sendmsg(str_buffer);
				break;
			}
		}
		if (continuousDistance) {
			/* If new distance data available, report distance to the user */
		}
		else if (continuousTime) {
			/* If new timer2 data available, report the 555 time period to the user */
		}
		else if (continuousVolts) {
			/* if new ADC0 data available, calculate the voltage in mV 
			and report the value to the user */
		}
	}
 }


/* sendmsg function */
void sendmsg(char* s) {
	 if (qcntr == sndcntr) {
		 qcntr = 0;
		sndcntr = 1;
		while (*s)
			queue[qcntr++] = *s++;
		USART3.TXDATAL = queue[0];
	 }
 }
 
 
 /* Function to set or clear all LED port bits, 1 - set, 0 - clear */
void Set_Clear_Ports(uint8_t set) {
 
	uint8_t i;
 
	for (i = 0; i <= 9; i += 1)
	{
		if (set)
			LED_Array[i].LED_PORT->OUTSET = LED_Array[i].bit_mapping;
		else
			LED_Array[i].LED_PORT->OUTCLR = LED_Array[i].bit_mapping;
	}
 }
 
 
/* ****************************************************************/ 
/* Interrupt Service Routines */
/* ****************************************************************/  
ISR(TCB0_INT_vect)
 {
	TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
 
	/* Use this ISR to capture the HC-SR04 Pulse Width, which can be used 
	   to calculate the distance to an object */
	
	// newDistanceData = 1;
 }
 
 
 ISR(TCB2_INT_vect)
 {
	 TCB2.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */

	/* Use this ISR to capture the 555 Period and High Pulse Width number of clocks, 
		which can be used to calculate the Period and high and low pulse times */
	 
	// newTimeData = 1;
}
  
 ISR(TCB3_INT_vect)
 {
	TCB3.INTFLAGS = TCB_CAPT_bm;	/* Software clears the INTFLAG */
	LED_Array[4].LED_PORT->OUTSET = LED_Array[4].bit_mapping;
	
	/* Use a software counter to send a trigger pulse on PORTC bit 6 (LED_Array[4])*/
	static uint16_t softCount = 0;
	static uint16_t servCount = 0;
	if(softCount%2 == 1){
		LED_Array[4].LED_PORT->OUTSET = LED_Array[4].bit_mapping;
		softCount++;
		/* Set the Port bit high, delay 10 us (use a software delay loop (_delay_us(10)
	   the Set the Port bit low again */
		_delay_ms(10);
		LED_Array[4].LED_PORT->OUTCLR = LED_Array[4].bit_mapping;		
	}
	/* If ServoFollowADC == 0, Use a second software counter to see whether to move 
	   the Servo motor to its next position. The software counter should count to the 
	   value set by the numbers '1' to '9'. '0' is a special case */
	if(ServoFollowADC == 0){
		if(servCount == 1){
		
		} else if(servCount == 2){
	
		} else if(servCount == 3){
			
		} else if(servCount == 4){
			
		} else if(servCount == 5){
			
		} else if(servCount == 6){
			
		} else if(servCount == 7){
			
		} else if(servCount == 8){
			
		} else if(servCount == 9){
			
		}
		
		servCount++;
	}
	/* Set the new servo position using TCA0.SINGLE.CMP0BUF */
 }
 
 ISR(ADC0_RESRDY_vect)
 {
	 adc_reading = ADC0.RES;
	 
	 newADC0Data = 1;
	 /* set the LED[7] on/off based on the adc_reading */
	 if(adc_reading > THREE_5V) {
		 LED_Array[7].LED_PORT->OUTSET = LED_Array[7].bit_mapping;
	 } else {
		 LED_Array[7].LED_PORT->OUTCLR = LED_Array[7].bit_mapping;
	 }
	 /* If ServoFollowADC == 1 set the servomotor position to a position based on the 
		adc_reading value */
	 /* Set the servo position using TCA0.SINGLE.CMP0BUF */
	 
	 LED_Array[7].LED_PORT->OUTSET = LED_Array[7].bit_mapping;
	 
	 if(ServoFollowADC) {
		TCA0.SINGLE.CMP0BUF = 1250 + adc_reading;
	 }
}
 


ISR(USART3_TXC_vect) {
	  USART3.STATUS |= USART_TXCIF_bm;
	  if (qcntr != sndcntr)
	  USART3.TXDATAL = queue[sndcntr++];
 }
