/* ##################################################################

# MILESTONE: 4B
# PROGRAM: 4
# PROJECT: Lab4
# GROUP: 6

# NAME 1: Kyle, Erickson, V00996431
# NAME 2: Ellis, Evans, V00987129
# DESC:
# DATA
# REVISED ############################################################### */
#include <stdlib.h> // the header of the general-purpose standard library of C programming language
#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"


// define the global variables that can be used in every function ==========
volatile unsigned char ADC_result;  //Volatile Prevents compiler from optimizing away shared variables
volatile unsigned int ADC_result_flag;
volatile int direction;
volatile int killswitch;

// (ISR -> interrups service routine) (ADC -> analog to digital converter)
void main()  //interrupts sei, cli, ISR's Let hardware trigger code without polling
{
	//CPU clock prescale resister (CLKPR):
	CLKPR = 0x80; //"unlocks" the scaler
	CLKPR = 0x01; //sets the prescaler 16MHz/2 = 8MHz
	TCCR1B |=_BV(CS11); //Prescales Timer 1 to 8 MHz
	PWM();
	
	DDRC = 0b11111111;
	PORTC = 0b11111111;
	
	//initialize the lcd display
	InitLCD(LS_ULINE);   // or LS_BLINK
	LCDClear();
	LCDWriteStringXY(0,0,"DC Motor Ready");
	LCDWriteStringXY(0,1,":)");
	mTimer(2000);
	LCDClear();
	
	mTimer(2000);
	//PORTC = 0b00000000;
	
	cli(); // disable all of the interrupt ==========================
	
	// config the external interrupt ======================================
	EIMSK |= (_BV(INT2)); // enable external interrupt INT2. INT2 is on **pin PD2**
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt. Every rising edge on PD2 
										// (e.g button press) will trigger ISR(INT_vect)
	
	//config external interrupts for buttons for kill button and change direction
	//When button is pressed, jump to ISR
	EIMSK |= (_BV(INT0));//kill switch on **pin PD0**
	EICRA |= (_BV(ISC01) | _BV(ISC00));
	EIMSK |= (_BV(INT1));//change direction on **pin PD1**
	EICRA |= (_BV(ISC11) | _BV(ISC10));
	
	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0)
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	
	ADCSRA |= _BV(ADPS2) | _BV(ADPS0);  // set ADC prescaler = 32 (~488 kHz)
	
	ADMUX |= _BV(ADLAR) | _BV(REFS0); 
	/*
	Bit 7:6 are reference selection bits that select the voltage reference for the ADC.  
	Setting REFS0 selects AVCC (=5V) with external capacitor at AREF pin. If these bits are 
	changed during a conversion, the change will not go in effect until this conversion 
	is complete
	*/
	/*
	Bit 5 is ADC left adjust result. The ADLAR bit affects the presentation of the ADC 
	conversion result in the ADC Data Register. Write one to ADLAR to left adjust the 
	result. Otherwise, the result is right adjusted. Changing the ADLAR bit will affect
	the ADC Data Register immediately, regardless of any ongoing conversions.
	ONLY FOR 8 BIT PRECISION
	*/
	
	DDRC = 0xff;// set the PORTC as output to display the ADC result ==================
	
	sei();// sets the Global Enable for all interrupts ==========================
	
	ADCSRA |= _BV(ADATE);  // Enable auto-trigger
	ADCSRB = 0x00;  // Free-running mode
	
	ADCSRA |= _BV(ADSC);// initialize the ADC, start one conversion at the beginning ==========
						// when its done it will trigger ISR(ADC_vect)
	
	//motor implementation
	//DDRC = 0xFF;
	DDRB = 0b11111111;
	direction = 1;
	
	killswitch = 0;
	
	OCR0A = ADC_result; // Maps ADC to duty cycle for the PWM
	PORTB = 0b00001011;
	mTimer(1000);
	
				//this is a polling loop the waits until the ADC interrupt sets ADC_result_flag = 1
	while (1){ //then displays the reading on PORTC
		if (ADC_result_flag){
			OCR0A = ADC_result; // Maps ADC to duty cycle for the PWM
			PORTC = ADC_result;
			ADC_result_flag = 0x00;
			
			
			int duty = (ADC_result * 100) / 255; // this turns the 8 bit ADC reading into a precentage 0-100
												
			//int duty = ADC_result;

			LCDClear();
			LCDWriteStringXY(0,0,"ADC:");
			LCDWriteInt(ADC_result,3);

			LCDWriteStringXY(8,0,"PWM:");
			LCDWriteInt(duty,3);
			LCDWriteStringXY(15,0,"%");
			
			
			LCDWriteStringXY(0,1,"Dir:");
			if (direction)
			LCDWriteString("CCW ");
			else
			LCDWriteString("CW");
			
			LCDWriteStringXY(8,1,"KS:");
			if(killswitch)
			LCDWriteString("ON");
			else
			LCDWriteString(" ");
		}
	}
} // end main

//kill switch
ISR(INT0_vect){
	killswitch = 1;
	PORTB = 0b00001111;//brake
	cli();//disable interrupts
}

//switch direction
ISR(INT1_vect){
	if(direction == 1){
		direction = 0;//update direction
		killswitch = 0;
		PORTB = 0b00001111;//brake
		mTimer(5);
		PORTB = 0b00000111;//move CCW
		mTimer(5);
	}
	else{
		direction = 1;//update direction
		killswitch = 0;
		PORTB = 0b00001111;//brake
		mTimer(5);
		PORTB = 0b00001011;//move CW
		mTimer(5);
	}
}

// sensor switch: Active HIGH starts AD converstion =======
ISR(INT2_vect){ // when there is a rising edge, we need to do ADC =====================
	ADCSRA |= _BV(ADSC); // enable interrupt of ADC
}

// the interrupt will be trigured if the ADC is done ========================
ISR(ADC_vect){
	ADC_result = ADCH; //reads the 8-bit result 
	ADC_result_flag = 1; //signals that new data is ready
}

void PWM (){
	TCCR0A |= 0b00000011;//step 1
	
	//TIMSK0 |= TIMSK0 | 0b00000010;//step 2 - this enables an interrupt****
	/*
	Enable output compare interrupt enable (OCIE0A) for Timer 0 (See Page 131 and onward) in the
	Timer/Counter Interrupt Mask register (TIMSK0). Use timer/counter0 output compare match A interrupt
	enable. Read the description to find out what this does, and set the required
	*/
	
	TCCR0A |= 0b10000000;//step 3
	
	TCCR0B |= 0b00000010;//the ranges of PWM that the Pololu motor driver can accept is up to 20kHz
	/*
	Set the Clock Select bits (clock pre-scaler) in the Timer/Counter Control Register B (pg 129 and
	onward) to a reasonable value for the period of the PWM (T) signal for now, and experiment by changing
	this value later. Note, the ranges of PWM that the Pololu motor driver can accept is up to 20kHz
	*/
	
	OCR0A = 255/2;
	/*
	Set the value of the Output Compare Register A (OCRA) to the value you want the PWM signal
	to be reset at (sets the duty cycle of the PWM signal   See Fig. 5
	*/
	
	DDRB = 0b11111111;//set pin on PORTB that hosts OC0A to be an output for the motor
}

void mTimer (int count)
{
   /***
      Setup Timer1 as a ms timer
	  Using polling method not Interrupt Driven
   ***/
	  
   int i;
   i = 0;

   //TCCR1B |= _BV (CS11);  // Set prescaler (/8) clock 16MHz/8 -> 2MHz
   /* Set the Waveform gen. mode bit description to clear
     on compare mode only */
   TCCR1B |= _BV(WGM12);

   /* Set output compare register for 2000 cycles, 1ms */
   OCR1A = 0x03E8;//0x07D0;
 
   /* Initialize Timer1 to zero */
   TCNT1 = 0x0000;

   /* Enable the output compare interrupt */
   //TIMSK1 |= _BV(OCIE1A);  //remove if global interrups is set (sei())
   
   // this is the Timer Interrupt Mask Register
   //TIMSK1 = TIMSK1 | 0b00000010; // -> this enables an interrupt that has no ISR  
								//(interrupt service routine) so it jumps to an undefined address.
   
  
  
   /* Clear the Timer1 interrupt flag and begin timing */
   TIFR1 |= _BV(OCF1A);

   /* Poll the timer to determine when the timer has reached 1ms */
   while (i < count)
   {
      if((TIFR1 & 0x02) == 0x02){
	
	   /* Clear the interrupt flag by WRITING a ONE to the bit */
	   TIFR1 |= _BV(OCF1A);
	   i++;
	  }
   }
   //TCCR1B &= ~_BV (CS11);  //  disable prescalar, no Clock
   return;
}  /* mTimer */
