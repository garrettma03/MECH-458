/* Solution Set for the Final Project */
/* 	
	Course		: UVic Mechatronics 458
	Milestone	: 4A
	Title		: Data structures for MCUs and the Linked Queue Library

	Name 1:Garrett Ma					Student ID: V00976948
	Name 2:Nikolas Clarke-Abatis		Student ID:
	
	Description: You can change the following after you read it.  Lab3 Demo
	
	This main routine will only serve as a testing routine for now. At some point you can comment out
	The main routine, and can use the following library of functions in your other applications

	To do this...make sure both the .C file and the .H file are in the same directory as the .C file
	with the MAIN routine (this will make it more convenient)
*/

/* include libraries */
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include "lcd.h"
#include "myutils.h"
#include <avr/interrupt.h>
#include "LinkedQueue.h" 	/* This is the attached header file, which cleans things up */
							/* Make sure you read it!!! */

// define the global variables that can be used in every function ==========
volatile unsigned int ADC_result;
volatile unsigned int ADC_result_flag;
volatile unsigned int dir = 1;
volatile unsigned int killswitch = 0;
volatile unsigned int change_dir_req = 0;
volatile uint8_t high_byte = 0;
volatile uint8_t low_byte = 0;
volatile uint16_t calibrationValues[4] = {0,0,0,0};
volatile uint8_t end_gate = 0;
volatile unsigned int cur_value = 1024;
volatile int accSpeed = 20;
volatile int count = 0;
volatile unsigned int optical = 0;
volatile int stepperMotor[4] = {0b110000, 0b000110, 0b101000, 0b000101}; //half step
volatile int curr_bin; // start on black
int foundBlack = 0; // flag for hall effect sensor

// S-curve acceleration table (gentler)
const uint8_t s_curve[16] = {
      0,  2,  6, 12, 22, 35, 50, 70,
     70, 50, 35, 22, 12,  6,  2,  0
};

// Pointer
link *head; /* The ptr to the head of the queue */
link *tail; /* The ptr to the tail of the queue */
link *newLink; /* A ptr to a link aggregate data type (struct) */
link *rtnLink; /* same as the above */
element eTest; /* A variable to hold the aggregate data type known as
element */

// Function Definitions
void mTimer (int count);	/* This is a millisecond timer, using Timer1 */
void pwmSetup();
void calibration(void);
void findBlack();
void nTurn(int n, int direction);
void classify();
void rotateDish(int next_bin); 

typedef enum {
    WHITE = 0,
    ALUMINUM = 1,
    BLACK = 2,
    STEEL = 3,
    MATERIAL_COUNT
} Material;

const char *materialNames[MATERIAL_COUNT] = {
    "WHITE",
    "ALUM",
    "BLACK",
    "STEEL"
};

int main(){

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

    // Initialize queue once (uses the global head/tail)
    setup(&head, &tail);

    // Optionally reset rtnLink/newLink if you want
    rtnLink = NULL;
    newLink = NULL;

	// config the external interrupt ======================================
	EIMSK |= (_BV(INT0)); // Kill switch on PD0
	EICRA |= (_BV(ISC01) | _BV(ISC00));
	EIMSK |= (_BV(INT1)); // Change direction on PD1
	EICRA |= (_BV(ISC11) | _BV(ISC10));
	EIMSK |= (_BV(INT2)); // Optical Reflector on PD2 Pin 19
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt
    EICRB |= _BV(ISC41) | _BV(ISC40);  // rising edge for INT4

	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(REFS0); // ADLAR sets the upper 2 bits of ADCL
									  // to be the lower 2 of bits of the now 10-bit ADCH
									  // REFS0 is bit 6 of the ADMUX register and sets
									  // AVCC with external capacitor at AREF pin
	ADCSRA |= 0x04; // set ADC prescaler = 16 (~488 kHz)
	
	// sets the Global Enable for all interrupts ==========================
	sei();
	
	//Motor implementation
	dir = 1;
	killswitch = 0;

    //Spin Motor
    nTurn(50, 1); //Turn 90 degrees clockwise
    mTimer(2000);
    nTurn(100, 1); //Turn 180 degrees clockwise
    mTimer(2000);

    nTurn(50, -1); //Turn 60 degrees counter clockwise
    mTimer(2000);
    nTurn(100, -1); //Turn 180 degrees counter clockwise
    mTimer(2000);
	
	findBlack();
	while(!foundBlack){
		; // Wait until hall effect sensor finds black
	}
	
	OCR0A = 90; // Maps ADC to duty cycle for the PWM
	PORTB = 0b00001110; // Start clockwise

	calibration();
	// Display results
	mTimer(5000);
    curr_bin = BLACK;

    EIFR |= _BV(INTF3);
    EIMSK |= _BV(INT3);
	EICRA &= ~_BV(ISC30);
    EICRA |=  _BV(ISC31);

    
    while(1){

        if(killswitch){
			//PORTB = 0x0F; // Brake
			cli(); // Clear all interrupts
			LCDClear();
			LCDWriteStringXY(0,0,"KILLSWITCH ON");
			LCDWriteStringXY(0,1,"MOTOR STOPPED");
			return 0;
		}
		
		if(end_gate){
			end_gate = 0;

			link *item = NULL;

			// protect queue operation in case an ISR touches it
			cli();
			dequeue(&head, &item, &tail);
			sei();

			if (item != NULL) {
				int next_bin = (int)(item->e.itemCode); // extract payload
                rotateDish(next_bin);                    // rotate to the queued class
				free(item);                              // remove node
			} else {
				// queue empty — nothing to do (or handle default)
			}
            PORTB = 0b00001110; // Resume clockwise
		}

		if(optical){
			optical = 0;
			classify();
		}

        if (change_dir_req) {
            mTimer(20); // debounce delay
            if (PIND & _BV(PD1)) { // still pressed
                //PORTB = 0x0F; // Brake
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
    }

    return 0;
}

// nTurn function
void nTurn(int n, int direction)
{
    int start_delay = 25;     // slower start
    int min_delay   = 14;     // slower maximum speed

    // More accel/decel steps gives a smoother, safer ramp
    int accel_steps = n / 3;      
    int decel_steps = accel_steps;
    int cruise_steps = n - accel_steps - decel_steps;

    const int curve_len = 16;

    for (int i = 0; i < n; i++) {

        int delay;

        // ---------- ACCEL ----------
        if (i < accel_steps) {
            int idx = (i * (curve_len - 1)) / accel_steps;
            int easing = s_curve[idx];  // 0..70

            delay = start_delay -
                    ((start_delay - min_delay) * easing) / 70;
        }

        // ---------- CRUISE ----------
        else if (i < accel_steps + cruise_steps) {
            delay = min_delay;
        }

        // ---------- DECEL ----------
        else {
            int decel_i = i - (accel_steps + cruise_steps);
            int idx = (decel_i * (curve_len - 1)) / decel_steps;
            int easing = s_curve[idx];

            delay = min_delay +
                    ((start_delay - min_delay) * easing) / 70;
        }

        // ---------- STEP MOTOR ----------
        if (direction == 1) {    // CW
            count++;
            if (count > 3) count = 0;
        } else {                  // CCW
            count--;
            if (count < 0) count = 3;
        }

        PORTA = stepperMotor[count];
        mTimer(delay);
    }
}

void calibration(void){
    uint8_t samples_per_pass = 50;
    uint8_t i, j;

    LCDClear();
    LCDWriteStringXY(0,0,"Calibrating...");
    LCDWriteStringXY(0,1,"Waiting for 4 passes");

    for (i = 0; i < 4; i++) {
        cur_value = 1024;
        ADCSRA &= ~_BV(ADATE);   // one-shot ADC

        // Prepare for a NEW optical event 
        optical = 0;             // clear software flag

        EIFR |= _BV(INTF2);      // clear any stale INT2 flag
        EIMSK |= _BV(INT2);      // enable INT2

        // Wait until the optical ISR sets 'optical = 1'
        while (!optical) {
            ; // busy-wait; could add timeout if desired
        }

        // now object is in front of sensor; we can disable INT2
        EIMSK &= ~_BV(INT2);

        // --- take ADC samples and find min ---
        for (j = 0; j < samples_per_pass; j++) {
            ADCSRA |= _BV(ADSC);        // start conversion
            while (!ADC_result_flag) {
                ; // wait for ADC ISR
            }

            if (ADC_result < cur_value){
                cur_value = ADC_result;
            }
            ADC_result_flag = 0;
            mTimer(2);
        }

        calibrationValues[i] = cur_value;

        // show only this pass's min
        LCDClear();
        LCDWriteStringXY(0,0,"Pass:");
        LCDWriteIntXY(5,0, i+1, 1);
        LCDWriteStringXY(0,1,"Min:");
        LCDWriteIntXY(4,1, cur_value, 4);
        mTimer(500);
    }

    // summary display
    LCDClear();
    LCDWriteStringXY(0,0,"Cal:");
    LCDWriteIntXY(3,0,calibrationValues[0],4);
    LCDWriteIntXY(8,0,calibrationValues[1],4);
    LCDWriteStringXY(0,1,"Cal:");
    LCDWriteIntXY(3,1,calibrationValues[2],4);
    LCDWriteIntXY(8,1,calibrationValues[3],4);
}


void rotateDish(int next_bin) {
    
    int prev = curr_bin;                // remember starting bin
    int diff = next_bin - curr_bin;

    if (diff == 1 || diff == -3) {          // 90 CW
        nTurn(50, 1);
        curr_bin = next_bin;
    } else if (diff == -1 || diff == 3) {   // 90 CCW
        nTurn(50, -1);
        curr_bin = next_bin;
    } else if (abs(diff) == 2) {            // 180
        nTurn(100, 1);
        curr_bin = next_bin;
    } else {
        curr_bin = next_bin;                // no move
    }

    // normalize diff for display: -1, 0, 1, 2
    int disp = diff;
    if (disp == 3)  disp = -1;
    if (disp == -3) disp = 1;

    LCDClear();
    // Line 0: Cur and Nxt
    LCDWriteStringXY(0,0,"Cur:");
    LCDWriteIntXY(4,0,prev,1);
    LCDWriteStringXY(6,0,"Nxt:");
    LCDWriteIntXY(10,0,next_bin,1);

    // Line 1: Diff and material
    LCDWriteStringXY(0,1,"Dif:");
    LCDWriteIntXY(4,1,disp,2);
    LCDWriteStringXY(7,1, materialNames[next_bin]);

    mTimer(500);
}

void classify() {
    const uint8_t samples_per_pass = 40;
    uint16_t sample_min = 1024;
    uint8_t j;

    // Take several samples while the object is in front of the sensor
    for (j = 0; j < samples_per_pass; j++) {
        // Start single conversion
        ADCSRA |= _BV(ADSC);

        // Wait for ADC ISR to set the flag
        while (!ADC_result_flag) {
            ; // Wait for flag to be risen
        }

        // Copy and clear flag atomically
        cli();
        uint16_t sample = ADC_result;
        ADC_result_flag = 0;
        sei();

        if (sample < sample_min) {
            sample_min = sample;
        }

        mTimer(2);  // small delay between samples
    }

    // Nearest calibration value
    uint8_t best_class = 0;
    uint16_t best_diff = 1024;

    for (uint8_t i = 0; i < 4; i++) {
        uint16_t calib = calibrationValues[i];
        uint16_t diff;

        // Calculate absolute difference
        if (sample_min > calib) {
            diff = sample_min - calib;
        } else {
            diff = calib - sample_min;
        }

        // If this is the smallest difference so far, store it
        if (diff < best_diff) {
            best_diff = diff;
            best_class = i;
        }
    }

    // Re-arm optical interrupt for next object
    EIFR |= _BV(INTF2);     // clear any pending flag
    EIMSK |= _BV(INT2);     // enable INT2 again

    // Enqueue classification result using LinkedQueue
    link *newLink = NULL;
    initLink(&newLink);                 // allocate and initialize a new link
    if (newLink == NULL) {
        // allocation failed
        LCDClear();
        LCDWriteStringXY(0,0,"Alloc fail");
        mTimer(250);
    } else {
        newLink->e.itemCode = (uint8_t)best_class;

        // Block interrupts during queue operation
        cli();
        enqueue(&head, &tail, &newLink);
        sei();
    }

    // Debug output
    LCDClear();
    LCDWriteStringXY(0,0,"Val:");
    LCDWriteIntXY(4,0,sample_min,4);

    LCDWriteStringXY(0,1,"Class:");
    LCDWriteStringXY(6,1, materialNames[best_class]);
    mTimer(500);

}

void findBlack(){
    foundBlack = 0;

    // clear pending flag and enable INT4
    EIFR  |= _BV(INTF4);
    EIMSK |= _BV(INT4);

    LCDClear();
    LCDWriteStringXY(0,0,"Finding Black");

    // step in small increments so we can stop when foundBlack is set
    while (!foundBlack) {
        nTurn(1, 1);   // 1 step CW
    }

    // Found black, disable INT4
    EIMSK &= ~_BV(INT4);
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

ISR(INT0_vect){ // Kill Switch
	killswitch = 1;
}

ISR(INT1_vect){ 
	// Disable INT1 to avoid bounces triggering repeatedly
    EIMSK &= ~_BV(INT1);
    change_dir_req = 1;
}

// Optical reflector interrupt
ISR(INT2_vect) { // Trigger ADC conversion when object in optical sensor
    EIMSK &= ~_BV(INT2);   // disable INT2 to avoid retrigger/bounce
    EIFR  |= _BV(INTF2);   // clear any pending flag
    optical = 1;    
}

ISR(INT3_vect){ // ISR for end gate active low Pin 18
	PORTB = 0x0F; // Brake
	end_gate = 1;
}

ISR(INT4_vect){ // ISR for Hall Effect on PE4 Pin 2
    EIMSK &= ~_BV(INT4);   // disable INT4 to avoid retrigger/bounce
    EIFR  |= _BV(INTF4);   // clear any pending flag
    foundBlack = 1;
}

// the interrupt will be trigured if the ADC is done ========================
ISR(ADC_vect) {
	low_byte = ADCL;
    high_byte = ADCH;
	ADC_result = (high_byte << 8) | low_byte; // combine ADCH and ADCL for full 10-bit value
	ADC_result_flag = 1;
}