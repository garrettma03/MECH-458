#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"
#include "myutils.h"

// define the global variables that can be used in every function ==========
volatile unsigned char ADC_result;
volatile unsigned int ADC_result_flag;
volatile unsigned int dir = 1;
volatile unsigned int killswitch = 0;
volatile unsigned int change_dir_req = 0;

// Function Definitions
void mTimer (int count);	/* This is a millisecond timer, using Timer1 */
void pwmSetup();

int main() {
	cli(); // disable all of the interrupt ==========================

	CLKPR = 0x80; //Enable bit 7 (clock prescale enable/disable
	CLKPR = 0x01; //Set division factor to be 2

	/* Used for debugging purposes only LEDs on PORTC */
	DDRC = 0xFF; //Set PORTC to output
	DDRA = 0xFF; //Set PORTA to output
    DDRB = 0xFF; // Step 6

	//Start PWM in the background
	pwmSetup();

	// Init LCD
	InitLCD(LS_BLINK);
	LCDClear();
	LCDWriteStringXY(0,0,"DC Motor Ready");
	LCDWriteStringXY(0,1,"Hi");
	mTimer(2000);
	LCDClear();

	// config the external interrupt ======================================
	EIMSK |= (_BV(INT2)); // enable INT2
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt

	// Config external interrupts for buttons for kill button and change direction
	// When button is pressed, jump to ISR
	EIMSK |= (_BV(INT0)); // Kill switch on PD0
	EICRA |= (_BV(ISC01) | _BV(ISC00));
	EIMSK |= (_BV(INT1)); // Change direction on PD1
	EICRA |= (_BV(ISC11) | _BV(ISC10));

	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(ADLAR) | _BV(REFS0); // ADLAR sets the upper 2 bits of ADCL
									  // to be the lower 2 of bits of the now 10-bit ADCH
									  // REFS0 is bit 6 of the ADMUX register and sets
									  // AVCC with external capacitor at AREF pin
	ADCSRA |= 0x04; // set ADC prescaler = 16 (~488 kHz)
	
	// sets the Global Enable for all interrupts ==========================
	sei();
	
	// initialize the ADC, start one conversion at the beginning ==========
	ADCSRA |= 0x20; // Enable auto-trigger enable inside ADC status register
	ADCSRB |= 0x00; // Set ADC Auto Trigger Source to Free Running Mode
	ADCSRA |= _BV(ADSC);
	
	//Motor implementation
	dir = 1;
	killswitch = 0;
	
	OCR0A = ADC_result; // Maps ADC to duty cycle for the PWM
	PORTB = 0b00001110; // Start clockwise
	
	while (1){

		if(killswitch){
			PORTB = 0x0F; // Brake
			cli(); // Clear all interrupts
			LCDClear();
			LCDWriteStringXY(0,0,"KILLSWITCH ON");
			LCDWriteStringXY(0,1,"MOTOR STOPPED");
			return 0;
		}

        if (change_dir_req) {
            mTimer(20); // debounce delay
            if (PIND & _BV(PD1)) { // still pressed
                PORTB = 0x0F; // Brake
                if (dir) {
                    dir = 0;
                    PORTB = 0x0D;
                } else {
                    dir = 1;
                    PORTB = 0x0E;
                }
                // Wait until button is released
                while (PIND & _BV(PD1));
				mTimer(20);
            }
            change_dir_req = 0;
            EIMSK |= _BV(INT1); // re-enable INT1
        }
		
        if (ADC_result_flag){

            // Clear the screen first
            LCDClear();
        
            OCR0A = ADC_result; // Maps ADC to duty cycle for the PWM
            PORTC = ADC_result;
            ADC_result_flag = 0x00;
            ADCSRA |= _BV(ADSC);
            
            int duty = (ADC_result * 100) / 255; // Turns the 8 bit ADC reading into a percentage 0-100
			
			// Write ADC value to LCD
            LCDWriteStringXY(0,0,"ADC:");
            LCDWriteIntXY(4,0,ADC_result,3);

            // Write PWM duty cycle to LCD
            LCDWriteStringXY(8,0,"PWM:");
            LCDWriteIntXY(12,0,duty,3);
            LCDWriteStringXY(15,0,"%");
            
            // Write Direction to LCD
            LCDWriteStringXY(0,1,"Dir:");
            if (dir){
                LCDWriteStringXY(4,1,"CCW ");
            } else {
                LCDWriteStringXY(4,1,"CW  ");
            }
            
            // Write Kill Switch status to LCD
            LCDWriteStringXY(8,1,"KS:");
            if(killswitch){
                LCDWriteStringXY(11,1,"ON ");
            } else {
                LCDWriteStringXY(11,1,"OFF");
            }
        }
	}
	
	return 0;
} // end main

ISR(INT0_vect){ // Kill Switch
	killswitch = 1;
}

ISR(INT1_vect){ 
	// Disable INT1 to avoid bounces triggering repeatedly
    EIMSK &= ~_BV(INT1);
    change_dir_req = 1;
}

// sensor switch: Active HIGH starts AD converstion =======
ISR(INT2_vect) { // when there is a rising edge, we need to do ADC =====================
	ADCSRA |= _BV(ADSC);
}

// the interrupt will be trigured if the ADC is done ========================
ISR(ADC_vect) {
	ADC_result = ADCH;
	ADC_result_flag = 1;
}

// mTimer function
void mTimer (int count)
{
   /***
      Setup Timer1 as a ms timer
	  Using polling method not Interrupt Driven
   ***/
	  
   int i;

   i = 0;

   TCCR1B |= _BV (CS11);  // Set prescaler (/8) clock 16MHz/8 -> 2MHz
   /* Set the Waveform gen. mode bit description to clear
     on compare mode only */
   TCCR1B |= _BV(WGM12);

   /* Set output compare register for 2000 cycles, 1ms */
   OCR1A = 0x03E8;
 
   /* Initialize Timer1 to zero */
   TCNT1 = 0x0000;

   /* Enable the output compare interrupt */
   //TIMSK1 |= _BV(OCIE1A);  //remove if global interrups is set (sei())

   /* Clear the Timer1 interrupt flag and begin timing */
   TIFR1 |= _BV(OCF1A);

   /* Poll the timer to determine when the timer has reached 1ms */
   while (i < count)
   {
      while ((TIFR1 & 0x02) != 0x02);
	
	   /* Clear the interrupt flag by WRITING a ONE to the bit */
	   TIFR1 |= _BV(OCF1A);
	   i++;
   } /* while */
   TCCR1B &= ~_BV (CS11);  //  disable prescalar, no Clock
   return;
}  /* mTimer */

void pwmSetup(){
	PORTC = 0xFF;
		
	TCCR0A |= 0b00000011; // Set TCCR0A to Fast PWM mode and clear upon reaching compare match STEP 1
		
	TCCR0A |= 0b10000000; // Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode) STEP 3
		
	TCCR0B |= 0x02; // Set to no prescaling to 64 STEP 4
		
	OCR0A = 128; // Step 5
}

//10-bit backwards
#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"
#include "myutils.h"

// define the global variables that can be used in every function ==========
volatile unsigned int ADC_result;
volatile unsigned int ADC_result_flag;
volatile unsigned int dir = 1;
volatile unsigned int killswitch = 0;
volatile unsigned int change_dir_req = 0;
volatile uint8_t high_byte = 0;
volatile uint8_t low_byte = 0;
volatile uint8_t PWM_8bit = 0;
volatile uint8_t end_gate = 0;

// Function Definitions
void mTimer (int count);	/* This is a millisecond timer, using Timer1 */
void pwmSetup();

int main() {
	cli(); // disable all of the interrupt ==========================

	CLKPR = 0x80; //Enable bit 7 (clock prescale enable/disable
	CLKPR = 0x01; //Set division factor to be 2

	/* Used for debugging purposes only LEDs on PORTC */
	DDRC = 0xFF; //Set PORTC to output
	DDRA = 0x00; //Set PORTA to input
    DDRB = 0xFF; // Step 6

	//Start PWM in the background
	pwmSetup();

	// Init LCD
	InitLCD(LS_BLINK);
	LCDClear();
	LCDWriteStringXY(0,0,"DC Motor Ready");
	LCDWriteStringXY(0,1,"Hi");
	mTimer(2000);
	LCDClear();

	// config the external interrupt ======================================
	EIMSK |= (_BV(INT2)); // enable INT2
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt

	// Config external interrupts for buttons for kill button and change direction
	// When button is pressed, jump to ISR
	EIMSK |= (_BV(INT0)); // Kill switch on PD0
	EICRA |= (_BV(ISC01) | _BV(ISC00));
	EIMSK |= (_BV(INT1)); // Change direction on PD1
	EICRA |= (_BV(ISC11) | _BV(ISC10));
    EIMSK |= (_BV(INT3)); // End gate on PD3
	EICRA |= (_BV(ISC31) | _BV(ISC30));

	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(ADLAR) | _BV(REFS0); // ADLAR sets the upper 2 bits of ADCL
									  // to be the lower 2 of bits of the now 10-bit ADCH
									  // REFS0 is bit 6 of the ADMUX register and sets
									  // AVCC with external capacitor at AREF pin
	ADCSRA |= 0x04; // set ADC prescaler = 16 (~488 kHz)
	
	// sets the Global Enable for all interrupts ==========================
	sei();
	
	// initialize the ADC, start one conversion at the beginning ==========
	ADCSRA |= 0x20; // Enable auto-trigger enable inside ADC status register
	ADCSRB |= 0x00; // Set ADC Auto Trigger Source to Free Running Mode
	ADCSRA |= _BV(ADSC);
	
	//Motor implementation
	dir = 1;
	killswitch = 0;
	
	OCR0A = (ADC_result >> 2); // Maps ADC to duty cycle for the PWM
	PORTB = 0b00001110; // Start clockwise
	
	while (1){

		if(killswitch){
			PORTB = 0x0F; // Brake
			cli(); // Clear all interrupts
			LCDClear();
			LCDWriteStringXY(0,0,"KILLSWITCH ON");
			LCDWriteStringXY(0,1,"MOTOR STOPPED");
			return 0;
		}
		
		if(end_gate){
			end_gate = 0;
		}

        if (change_dir_req) {
            mTimer(20); // debounce delay
            if (PIND & _BV(PD1)) { // still pressed
                PORTB = 0x0F; // Brake
                if (dir) {
                    dir = 0;
                    PORTB = 0x0D;
                } else {
                    dir = 1;
                    PORTB = 0x0E;
                }
                // Wait until button is released
                while (PIND & _BV(PD1));
				mTimer(20);
            }
            change_dir_req = 0;
            EIMSK |= _BV(INT1); // re-enable INT1
        }
		
        if (ADC_result_flag){
			
			//Clear interrupt flag
			ADC_result_flag = 0;

            // Clear the screen first
            LCDClear();
			
			PWM_8bit = (ADC_result >> 2);
        
            OCR0A = PWM_8bit; // Lose the lower 2 bits
            
            int duty = (ADC_result * 100) / 1023; // Turns the 10-bit ADC reading into a percentage 0-100
			
			// Write ADC value to LCD
            LCDWriteStringXY(0,0,"ADC:");
            LCDWriteIntXY(4,0,ADC_result,3);

            // Write PWM duty cycle to LCD
            LCDWriteStringXY(8,0,"PWM:");
            LCDWriteIntXY(12,0,duty,3);
            LCDWriteStringXY(15,0,"%");
            
            // Write Direction to LCD
            LCDWriteStringXY(0,1,"Dir:");
            if (dir){
                LCDWriteStringXY(4,1,"CCW ");
            } else {
                LCDWriteStringXY(4,1,"CW  ");
            }
            
            // Write Kill Switch status to LCD
            LCDWriteStringXY(8,1,"KS:");
            if(killswitch){
                LCDWriteStringXY(11,1,"ON ");
            } else {
                LCDWriteStringXY(11,1,"OFF");
            }
        }
	}
	
	return 0;
} // end main

ISR(INT0_vect){ // Kill Switch
	killswitch = 1;
}

ISR(INT1_vect){ 
	// Disable INT1 to avoid bounces triggering repeatedly
    EIMSK &= ~_BV(INT1);
    change_dir_req = 1;
}

// sensor switch: Active HIGH starts AD converstion =======
ISR(INT2_vect) { // when there is a rising edge, we need to do ADC =====================
	ADCSRA |= _BV(ADSC);
}

ISR(INT3_vect){ // ISR for end gate active low
	PORTB = 0x0F; // Brake
	end_gate = 1;
}

// the interrupt will be trigured if the ADC is done ========================
ISR(ADC_vect) {
	low_byte = ADCL;
    high_byte = ADCH;
	ADC_result = high_byte << 2 | (low_byte >> 6); // combine ADCH and ADCL for full 10-bit value
	ADC_result_flag = 1;
}

// mTimer function
void mTimer (int count)
{
   /***
      Setup Timer1 as a ms timer
	  Using polling method not Interrupt Driven
   ***/
	  
   int i;

   i = 0;

   TCCR1B |= _BV (CS11);  // Set prescaler (/8) clock 16MHz/8 -> 2MHz
   /* Set the Waveform gen. mode bit description to clear
     on compare mode only */
   TCCR1B |= _BV(WGM12);

   /* Set output compare register for 2000 cycles, 1ms */
   OCR1A = 0x03E8;
 
   /* Initialize Timer1 to zero */
   TCNT1 = 0x0000;

   /* Enable the output compare interrupt */
   //TIMSK1 |= _BV(OCIE1A);  //remove if global interrups is set (sei())

   /* Clear the Timer1 interrupt flag and begin timing */
   TIFR1 |= _BV(OCF1A);

   /* Poll the timer to determine when the timer has reached 1ms */
   while (i < count)
   {
      while ((TIFR1 & 0x02) != 0x02);
	
	   /* Clear the interrupt flag by WRITING a ONE to the bit */
	   TIFR1 |= _BV(OCF1A);
	   i++;
   } /* while */
   TCCR1B &= ~_BV (CS11);  //  disable prescalar, no Clock
   return;
}  /* mTimer */

void pwmSetup(){

	TCCR0A |= 0b00000011; // Set TCCR0A to Fast PWM mode and clear upon reaching compare match STEP 1
		
	TCCR0A |= 0b10000000; // Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode) STEP 3
		
	TCCR0B |= 0x02; // Set to no prescaling to 64 STEP 4
		
	OCR0A = 128; // Step 5
}