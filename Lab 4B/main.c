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
//volatile int stepperMotor[4] = {0b110000, 0b000110, 0b101000, 0b000101}; //half step
volatile int stepperMotor[4] = {0b110110, 0b101101, 0b110110, 0b101101};   //full step?
volatile int count = 0;
volatile int accSpeed = 20;

// Function Definitions
void mTimer (int count);	/* This is a millisecond timer, using Timer1 */
void nTurn(int n, int direction);

int main(){

    /* Used for debugging purposes only LEDs on PORTC */
	DDRC = 0xFF;
	DDRA = 0xFF;		//Set PORTA to output

	CLKPR = 0x80; //Enable bit 7 (clock prescale enable/disable
	CLKPR = 0x01; //Set division factor to be 2
    while(1){
        nTurn(90, 1); //Turn 90 degrees clockwise
        mTimer(2000);
        nTurn(180, 1); //Turn 180 degrees clockwise
        mTimer(2000);

        nTurn(90, -1); //Turn 60 degrees counter clockwise
        mTimer(2000);
        nTurn(180, -1); //Turn 180 degrees counter clockwise
        mTimer(2000);
    }

    return 0;
}

// nTurn function
void nTurn(int n, int direction){
    int steps;
    if(direction == 1){ //Turn clockwise
        switch(n){
            case 90:
                steps = 50;
                for(int i = 0; i < steps; i++){
                    PORTA = stepperMotor[count];
                    mTimer(20);
                    count++;
                    if(count > 3){
                        count = 0;
                    }
                }
                break;
                
            case 180:
                accSpeed = 20;
                steps = 100;
                for(int i = 0; i < steps; i++){
                    if(i<15){
                        PORTA = stepperMotor[count];
                        mTimer(accSpeed);
                        count++;
                        if(count > 3){
                            count = 0;
                        }
                    }else if(i<50){
                        if(accSpeed > 10){
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            accSpeed = accSpeed - 1;
                            count++;
                            if(count > 3){
                                count = 0;
                            }
                        }else{
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            count++;
                            if(count > 3){
                                count = 0;
                            }
                        } 
                    }else{
                        if(accSpeed < 21){
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            accSpeed = accSpeed + 1;
                            count++;
                            if(count > 3){
                                count = 0;
                            }
                        }else{
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            count++;
                            if(count > 3){
                                count = 0;
                            }
                        } 
                    }
                }
            break;

        }

    }

    if(direction == -1){ //Turn clockwise
        switch(n){
            case 90:
                steps = 50;
                for(int i = 0; i < steps; i++){
                    PORTA = stepperMotor[count];
                    mTimer(20);
                    count--;
                    if(count < 0){
                        count = 3;
                    }
                }
                
            case 180:
                accSpeed = 20;
                steps = 100;
                for(int i = 0; i < steps; i++){
                    if(i<15){
                        PORTA = stepperMotor[count];
                        mTimer(accSpeed);
                        count--;
                        if(count > 3){
                            count = 0;
                        }
                    }else if(i<50){
                        if(accSpeed > 10){
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            accSpeed = accSpeed - 1;
                            count--;
                            if(count > 3){
                                count = 0;
                            }
                        }else{
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            count--;
                            if(count > 3){
                                count = 0;
                            }
                        } 
                    }else{
                        if(accSpeed < 21){
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            accSpeed = accSpeed + 1;
                            count--;
                            if(count > 3){
                                count = 0;
                            }
                        }else{
                            PORTA = stepperMotor[count];
                            mTimer(accSpeed);
                            count--;
                            if(count > 3){
                                count = 0;
                            }
                        } 
                    }
                }
            break;
        }

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