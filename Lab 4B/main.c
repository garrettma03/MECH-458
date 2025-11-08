/* Solution Set for the Lab 4A */
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
#include "LinkedQueue.h" 	/* This is the attached header file, which cleans things up */
							/* Make sure you read it!!! */
/* global variables */
/* Avoid using these */
volatile int stepperMotor[4] = {0b110000, 0b000110, 0b101000, 0b000101}; //half step
//volatile int stepperMotor[4] = {0b110110, 0b101101, 0b110110, 0b101101};   //full step?
volatile int count = 0;
volatile int accSpeed = 20;
volatile int calibrationValues[4];
volatile int curr_bin = 2; // start on black

// Function Definitions
void mTimer (int count);	/* This is a millisecond timer, using Timer1 */
void nTurn(int n, int direction);

// Node sturcture for storeing cylinder types
struct Node {
    int val;
    struct Node *next;
};

// Front and rear pointers for the queue
struct Node *front = NULL;
struct Node *rear = NULL;


int main(){

    /* Used for debugging purposes only LEDs on PORTC */
	DDRC = 0xFF;
	DDRA = 0xFF;		//Set PORTA to output

	CLKPR = 0x80; //Enable bit 7 (clock prescale enable/disable
	CLKPR = 0x01; //Set division factor to be 2


    // //start PWM and Belt
    // startBelt();
    
    // calibration();


    // while(1){

    //     addVal();

    //     //the function for stopping the belt goes here:
    //     stopBelt();

    //     rotateDish(dequeue());
    // }


    while(1){
        nTurn(50, 1); //Turn 90 degrees clockwise
        mTimer(2000);
        nTurn(100, 1); //Turn 180 degrees clockwise
        mTimer(2000);

        nTurn(50, -1); //Turn 60 degrees counter clockwise
        mTimer(2000);
        nTurn(100, -1); //Turn 180 degrees counter clockwise
        mTimer(2000);
    }

    return 0;
}

// nTurn function
void nTurn(int n, int direction){   // n is steps
    if(direction == 1){             //Turn clockwise
        accSpeed = 20;
        for(int i = 0; i < n; i++){
            if(i < 10 ){
                PORTA = stepperMotor[count]; //try i%4 instead of count
                mTimer(accSpeed);
                count++;
                if(count > 3){
                    count = 0;
                }
            }else if(i < n-19){                  // increase for more acc and held acc
                if(accSpeed > 1){               //accelerate
                    accSpeed -= 1;
                }
                PORTA = stepperMotor[count];    //holds if acc == 1
                mTimer(accSpeed);
                count++;
                if(count > 3){
                    count = 0;
                }
            }else{
                PORTA = stepperMotor[count];
                mTimer(accSpeed);
                accSpeed = accSpeed + 1;
                count++;
                if(count > 3){
                    count = 0;
                }
            }
        }
    }

    if(direction == -1){ //Turn Couter clockwise
    accSpeed = 20;
        for(int i = 0; i < n; i++){
            if(i < 10 ){
                PORTA = stepperMotor[count]; //try i%4 instead of count
                mTimer(accSpeed);
                count--;
                if(count < 0){
                    count = 3;
                }
            }else if(i < n-19){                  // increase for more acc and held acc
                if(accSpeed > 1){               //accelerate
                    accSpeed -= 1;
                }
                PORTA = stepperMotor[count];    //holds if acc == 1
                mTimer(accSpeed);
                count--;
                if(count < 0){
                    count = 3;
                }
            }else{
                PORTA = stepperMotor[count];
                mTimer(accSpeed);
                accSpeed = accSpeed + 1;
                count--;
                if(count < 0){
                    count = 3;
                }
            }
        }
    }
}

// Calibration function
void calibration(void){
    uint8_t samples_per_pass = 50;    // how many ADC reads to take per object
    uint8_t i, j;

    LCDClear();
    LCDWriteStringXY(0,0,"Calibrating...");
    LCDWriteStringXY(0,1,"Waiting for 4 passes");

    for (i = 0; i < 4; i++) {

		// Reset current value for this pass
		cur_value = 1024;

        // wait for optical event (set by ISR INT2)
        while (!optical){
			;
		}
        // consume the event
        optical = 0;

		// Make sure ADC is one-shot (not free-running)
		ADCSRA &= ~_BV(ADATE);

		// Clear any pending optical interrupt and enable INT2
		EIFR |= _BV(INTF2);
		EIMSK |= _BV(INT2);

        // get several ADC samples while object is present and keep the minimum
        for (j = 0; j < samples_per_pass; j++) {
            ADCSRA |= _BV(ADSC);               // start single conversion
            while (!ADC_result_flag) {  
				;      // wait for ADC ISR to set flag
            }
            ADC_result_flag = 0;
            if (ADC_result < cur_value){
				cur_value = ADC_result;
			}
            mTimer(2); // short spacing between reads (adjust as needed)
        }
		
        calibrationValues[i] = cur_value; // store lowest reading for this pass

        // update LCD with values collected so far
        LCDClear();
        LCDWriteStringXY(0,0,"Cal:");
        LCDWriteIntXY(3,0,calibrationValues[0],4);
        LCDWriteIntXY(8,0,calibrationValues[1],4);
        LCDWriteStringXY(0,1,"Cal:");
        LCDWriteIntXY(3,1,calibrationValues[2],4);
        LCDWriteIntXY(8,1,calibrationValues[3],4);

        // re-arm interrupt for next pass (ISR(INT2) disables it)
        EIFR |= _BV(INTF2);
        EIMSK |= _BV(INT2);

        // optional small delay to avoid false retrigger
        mTimer(25);
    }
}

void addVal(void){                       //fix
    uint8_t samples_per_pass = 50;    // how many ADC reads to take per object //garrett debounce?
    uint8_t i, j;

    for (i = 0; i < 4; i++) {
		// Reset current value for this pass
		cur_value = 1024;
        // wait for optical event (set by ISR INT2)
        while (!optical){
			;
		}
        // consume the event
        optical = 0;
		// Make sure ADC is one-shot (not free-running)
		ADCSRA &= ~_BV(ADATE);
		// Clear any pending optical interrupt and enable INT2
		EIFR |= _BV(INTF2);
		EIMSK |= _BV(INT2);
        // get several ADC samples while object is present and keep the minimum
        for (j = 0; j < samples_per_pass; j++) {
            ADCSRA |= _BV(ADSC);               // start single conversion
            while (!ADC_result_flag) {  
				;      // wait for ADC ISR to set flag
            }
            ADC_result_flag = 0;
            if (ADC_result < cur_value){
				cur_value = ADC_result;
			}
            mTimer(2); // short spacing between reads (adjust as needed)
        }

        enqueue(cur_value); //is this right??????????????????????????????????????

        // re-arm interrupt for next pass (ISR(INT2) disables it)
        EIFR |= _BV(INTF2);
        EIMSK |= _BV(INT2);

        // optional small delay to avoid false retrigger
        mTimer(25);
    }
}

void enqueue(int value){ // todo fix edge case where calibrationValues[1] does not fall in bounds
    //todo
        if(value > calibrationValues[1]*0.9 && value < calibrationValues[1]*1.1){ // aluminium? 

            struct Node *newNode = (struct Node *)malloc(sizeof(struct Node));
                if (newNode == NULL) {
                    printf("Memory allocation failed\n");
                    return;
                }
                newNode->val = 1; // aluminium
                newNode->next = NULL;

                if (rear == NULL) {
                    // Queue is empty
                    front = rear = newNode;
                } else {
                    rear->next = newNode;
                    rear = newNode;
                }
        }
        if(value > calibrationValues[2]*0.9 && value < calibrationValues[2]*1.1){ // black

            struct Node *newNode = (struct Node *)malloc(sizeof(struct Node));
                if (newNode == NULL) {
                    printf("Memory allocation failed\n");
                    return;
                }
                newNode->val = 2; // black
                newNode->next = NULL;

                if (rear == NULL) {
                    // Queue is empty
                    front = rear = newNode;
                } else {
                    rear->next = newNode;
                    rear = newNode;
                }
        }
        if(value > calibrationValues[3]*0.9 && value < calibrationValues[3]*1.1){ // steel

            struct Node *newNode = (struct Node *)malloc(sizeof(struct Node));
                if (newNode == NULL) {
                    printf("Memory allocation failed\n");
                    return;
                }
                newNode->val = 3; // steel
                newNode->next = NULL;

                if (rear == NULL) {
                    // Queue is empty
                    front = rear = newNode;
                } else {
                    rear->next = newNode;
                    rear = newNode;
                }
        }
        if(value > calibrationValues[0]*0.9 && value < calibrationValues[0]*1.1){ // white 

            struct Node *newNode = (struct Node *)malloc(sizeof(struct Node));
                if (newNode == NULL) {
                    printf("Memory allocation failed\n");
                    return;
                }
                newNode->val = 0; // white
                newNode->next = NULL;

                if (rear == NULL) {
                    // Queue is empty
                    front = rear = newNode;
                } else {
                    rear->next = newNode;
                    rear = newNode;
                }
        }
}

int dequeue() {
    if (front == NULL) { // todo -------------------------------figure out
        return -1;  // Sentinel value for empty queue
    }

    struct Node *temp = front;
    int value = temp->val;  // Save the value to return

    front = front->next;
    if (front == NULL) {
        rear = NULL;  // Queue became empty
    }

    free(temp);
    return value;
}

void rotateDish(int next_bin) {
    printf("Rotating dish to position based on value: %d\n", next_bin);

// Constants
int WHITE = 0;
int ALUMINUM = 1;
int BLACK = 2;
int STEEL = 3;
int diff = next_bin - curr_bin;

    if (diff == 1 || diff == -3) {          // and include edge case, rotate WHITE to STEEL
        // rotate 90 CW
        nTurn(50, 1); //Turn 90 degrees clockwise
        curr_bin = next_bin;                //check with lewis on this
    } else if (diff == -1 || diff == 3) {   // and includes edge case, rotate STEEL to WHITE
        // rotate 90 CCW
        nTurn(50, -1);
        curr_bin = next_bin;
    } else if (abs(diff) == 2) {
        // rotate 180
        nTurn(100, 1);
        curr_bin = next_bin;
    } else{
        // diff is 0, do nothing
        //yata
        curr_bin = next_bin;
    }

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