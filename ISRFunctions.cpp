/*
 * NAME: ISRFunctions.cpp
 *
 */

#include <Arduino.h>

// We create variables for the time width value of the PWM input signal
volatile unsigned long counter_1 = 0L;
volatile byte last_PWM_state = 0;

// To store the 1000us to 2000us value we create variables
volatile int PWM_INPUT = 0;      //In my case PWM_IN pin is D8 of the Arduino

extern byte sequence_step;
extern void set_next_step();

/*
 * This is the interruption routine on pin change
 * in this case for digital pin D8 which is the PWM input
 */
ISR(PCINT0_vect) {
  //First we take the current count value in micro seconds using the micros() function
  unsigned long current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_PWM_state == 0) {                        //If the last state was 0, then we have a state change...
      last_PWM_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_PWM_state == 1) {                     //If pin 8 is LOW and the last state was HIGH then we have a state change
    last_PWM_state = 0;                              //Store the current state into the last state for the next loop
    PWM_INPUT = current_count - counter_1;           //We make the time difference. PWM_INPUT is current_time - counter_1 in micro-seconds.
  }
}

/*
 * Interrupt vector for the Analog Comparator (AC)
 * Def: AC Output (ACO) is 1, if voltage on positive input pin of AC (=AIN0) is
 *      higher than voltage on negative input pin (=AIN1).
 * AIN0(=D6) is tied to common ground of all 3 phases.
 * AIN1(multiplexed) to: A2 for phase A, A1 for phase B, A0 for phase C
 * To detect a  rising BEMF, AIN1 goes above common ground (AIN0 < AIN1), so ACO is 0.
 * To detect a falling BEMF, AIN1 goes below common ground (AIN0 > AIN1), so ACO is 1.
 */
ISR (ANALOG_COMP_vect) {
  for (int i=0; i<10; i++) {                // We check the comparator 10 times just to be sure
    if (sequence_step & 1) {                // If step is odd (IRQ on falling BEMF), ACO should be 1
      if (0 == (ACSR & B00100000)) i -= 1;  // ACO = 0 (Analog Comparator Output = 0), try again
    } else {                                // If step is even (IRQ on rising BEMF), ACO should be 0
      if (0 != (ACSR & B00100000)) i -= 1;  // ACO = 1 (Analog Comparator Output = 1), try again
    }
  }
Serial.print("IRQ for seq="); Serial.println(sequence_step);
///  set_next_step();                    //set the next step of the sequence
}
